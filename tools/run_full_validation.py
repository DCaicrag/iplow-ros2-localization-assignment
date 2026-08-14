#!/usr/bin/env python3

import argparse
import math
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


SEGMENT_DURATION_S = 10.0


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference_deg(a, b):
    delta = math.atan2(
        math.sin(a - b),
        math.cos(a - b),
    )
    return abs(math.degrees(delta))


def distance_xyz(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


class FullValidationMonitor(Node):

    def __init__(self, report_path, bag_path):
        super().__init__("iplow_full_validation_monitor")

        self.report_path = report_path
        self.bag_path = bag_path
        self.report = report_path.open(
            "w",
            encoding="utf-8",
            buffering=1,
        )

        sensor_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_count = 0
        self.odom_count = 0
        self.tf_count = 0
        self.cloud_count = 0

        self.pose_by_stamp = {}
        self.odom_by_stamp = {}
        self.tf_by_stamp = {}

        self.pose_odom_position_errors = []
        self.pose_odom_yaw_errors = []
        self.pose_tf_position_errors = []
        self.pose_tf_yaw_errors = []

        self.first_pose_time = None
        self.last_pose_time = None

        self.first_position = None
        self.last_position = None
        self.previous_position = None
        self.previous_yaw = None

        self.total_path = 0.0
        self.max_pose_step = 0.0
        self.max_yaw_step = 0.0
        self.motion_updates = 0

        self.cloud_frames = set()

        self.segment_number = 1
        self.segment_start_time = None
        self.segment_start_position = None
        self.segment_path = 0.0
        self.segment_max_step = 0.0
        self.segment_pose_count = 0
        self.segment_cloud_count = 0

        self.bag_seen = False

        self.create_subscription(
            PoseStamped,
            "/robot_pose",
            self.pose_callback,
            100,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            100,
        )

        self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            100,
        )

        self.create_subscription(
            PointCloud2,
            "/mid360_filtered",
            self.cloud_callback,
            sensor_qos,
        )

        self.write("=" * 76)
        self.write("iPLOW FULL-RUN RUNTIME VALIDATION")
        self.write("=" * 76)
        self.write(f"Bag: {self.bag_path}")
        self.write(f"Segment duration: {SEGMENT_DURATION_S:.0f} s")
        self.write("")
        self.write("Waiting for experiment to start...")

    def write(self, text=""):
        print(text, flush=True)
        self.report.write(text + "\n")

    def pose_callback(self, msg):
        t = stamp_seconds(msg.header.stamp)
        key = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

        self.pose_count += 1
        self.segment_pose_count += 1

        position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

        yaw = yaw_from_quaternion(
            msg.pose.orientation
        )

        self.pose_by_stamp[key] = msg

        if self.first_pose_time is None:
            self.first_pose_time = t
            self.segment_start_time = t

            self.first_position = position
            self.segment_start_position = position

            self.write("")
            self.write(
                f"First localization sample at "
                f"bag time {t:.3f}"
            )

        self.last_pose_time = t
        self.last_position = position

        if self.previous_position is not None:
            step = distance_xyz(
                position,
                self.previous_position,
            )

            if step > 1e-6:
                self.total_path += step
                self.segment_path += step
                self.motion_updates += 1

                self.max_pose_step = max(
                    self.max_pose_step,
                    step,
                )

                self.segment_max_step = max(
                    self.segment_max_step,
                    step,
                )

        if self.previous_yaw is not None:
            yaw_step = angle_difference_deg(
                yaw,
                self.previous_yaw,
            )

            self.max_yaw_step = max(
                self.max_yaw_step,
                yaw_step,
            )

        self.previous_position = position
        self.previous_yaw = yaw

        self.compare_outputs(key)

        if (
            self.segment_start_time is not None
            and t - self.segment_start_time
            >= SEGMENT_DURATION_S
        ):
            self.print_segment(t)

        self.prune(t)

    def odom_callback(self, msg):
        key = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

        self.odom_count += 1
        self.odom_by_stamp[key] = msg
        self.compare_outputs(key)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if (
                transform.header.frame_id == "odom"
                and transform.child_frame_id == "base"
            ):
                key = (
                    transform.header.stamp.sec,
                    transform.header.stamp.nanosec,
                )

                self.tf_count += 1
                self.tf_by_stamp[key] = transform
                self.compare_outputs(key)

    def cloud_callback(self, msg):
        self.cloud_count += 1
        self.segment_cloud_count += 1
        self.cloud_frames.add(msg.header.frame_id)

    def compare_outputs(self, key):
        pose = self.pose_by_stamp.get(key)

        if pose is None:
            return

        pose_position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )

        pose_yaw = yaw_from_quaternion(
            pose.pose.orientation
        )

        odom = self.odom_by_stamp.get(key)

        if odom is not None:
            odom_position = (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            )

            self.pose_odom_position_errors.append(
                distance_xyz(
                    pose_position,
                    odom_position,
                )
            )

            self.pose_odom_yaw_errors.append(
                angle_difference_deg(
                    pose_yaw,
                    yaw_from_quaternion(
                        odom.pose.pose.orientation
                    ),
                )
            )

        transform = self.tf_by_stamp.get(key)

        if transform is not None:
            tf_position = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )

            self.pose_tf_position_errors.append(
                distance_xyz(
                    pose_position,
                    tf_position,
                )
            )

            self.pose_tf_yaw_errors.append(
                angle_difference_deg(
                    pose_yaw,
                    yaw_from_quaternion(
                        transform.transform.rotation
                    ),
                )
            )

    def prune(self, current_time):
        cutoff_sec = int(current_time - 5.0)

        for mapping in (
            self.pose_by_stamp,
            self.odom_by_stamp,
            self.tf_by_stamp,
        ):
            old_keys = [
                key
                for key in mapping
                if key[0] < cutoff_sec
            ]

            for key in old_keys:
                mapping.pop(key, None)

    def print_segment(self, current_time):
        start = self.segment_start_position
        end = self.last_position

        elapsed = (
            current_time
            - self.segment_start_time
        )

        net_motion = distance_xyz(start, end)

        pose_rate = (
            self.segment_pose_count / elapsed
            if elapsed > 0
            else 0.0
        )

        cloud_rate = (
            self.segment_cloud_count / elapsed
            if elapsed > 0
            else 0.0
        )

        relative_start = (
            self.segment_start_time
            - self.first_pose_time
        )

        relative_end = (
            current_time
            - self.first_pose_time
        )

        self.write("")
        self.write(
            f"========== SEGMENT "
            f"{self.segment_number:02d} =========="
        )

        self.write(
            f"Relative bag time: "
            f"{relative_start:7.1f} -> "
            f"{relative_end:7.1f} s"
        )

        self.write(
            f"Start XYZ:       "
            f"({start[0]:7.3f}, "
            f"{start[1]:7.3f}, "
            f"{start[2]:7.3f}) m"
        )

        self.write(
            f"End XYZ:         "
            f"({end[0]:7.3f}, "
            f"{end[1]:7.3f}, "
            f"{end[2]:7.3f}) m"
        )

        self.write(
            f"Segment path:    "
            f"{self.segment_path:7.3f} m"
        )

        self.write(
            f"Net displacement:"
            f" {net_motion:7.3f} m"
        )

        self.write(
            f"Max pose step:   "
            f"{self.segment_max_step:7.3f} m"
        )

        self.write(
            f"Pose rate:       "
            f"{pose_rate:7.2f} Hz"
        )

        self.write(
            f"Cloud rate:      "
            f"{cloud_rate:7.2f} Hz"
        )

        self.segment_number += 1
        self.segment_start_time = current_time
        self.segment_start_position = end
        self.segment_path = 0.0
        self.segment_max_step = 0.0
        self.segment_pose_count = 0
        self.segment_cloud_count = 0

    def rosbag_running(self):
        names = self.get_node_names()

        running = (
            "rosbag2_player" in names
        )

        if running:
            self.bag_seen = True

        return running

    def final_summary(self):
        self.write("")
        self.write("=" * 76)
        self.write(
            "iPLOW FULL-RUN CONSISTENCY SUMMARY"
        )
        self.write("=" * 76)

        if self.first_pose_time is None:
            self.write(
                "ERROR: No localization samples "
                "were received."
            )
            return

        duration = (
            self.last_pose_time
            - self.first_pose_time
        )

        net_displacement = distance_xyz(
            self.first_position,
            self.last_position,
        )

        self.write(
            f"Analyzed duration:          "
            f"{duration:.2f} s"
        )

        self.write(
            f"Pose messages:              "
            f"{self.pose_count}"
        )

        self.write(
            f"Odometry messages:          "
            f"{self.odom_count}"
        )

        self.write(
            f"Dynamic TF messages:        "
            f"{self.tf_count}"
        )

        self.write(
            f"PointCloud messages:        "
            f"{self.cloud_count}"
        )

        self.write("")

        if duration > 0:
            self.write(
                f"Pose mean rate:             "
                f"{self.pose_count / duration:.2f} Hz"
            )

            self.write(
                f"Odometry mean rate:         "
                f"{self.odom_count / duration:.2f} Hz"
            )

            self.write(
                f"TF mean rate:               "
                f"{self.tf_count / duration:.2f} Hz"
            )

            self.write(
                f"PointCloud mean rate:       "
                f"{self.cloud_count / duration:.2f} Hz"
            )

        self.write("")
        self.write(
            f"Total pose path:            "
            f"{self.total_path:.3f} m"
        )

        self.write(
            f"Net displacement:           "
            f"{net_displacement:.3f} m"
        )

        self.write(
            f"Maximum pose step:          "
            f"{self.max_pose_step:.3f} m"
        )

        self.write(
            f"Maximum yaw step:           "
            f"{self.max_yaw_step:.3f} deg"
        )

        self.write(
            f"Unique motion updates:      "
            f"{self.motion_updates}"
        )

        self.write(
            f"PointCloud frame(s):        "
            f"{sorted(self.cloud_frames)}"
        )

        self.write("")
        self.write(
            "--- SAME-TIMESTAMP CONSISTENCY ---"
        )

        if self.pose_odom_position_errors:
            self.write(
                f"Pose/Odom max position err: "
                f"{max(self.pose_odom_position_errors):.9f} m"
            )

            self.write(
                f"Pose/Odom max yaw error:    "
                f"{max(self.pose_odom_yaw_errors):.9f} deg"
            )

        if self.pose_tf_position_errors:
            self.write(
                f"Pose/TF max position err:   "
                f"{max(self.pose_tf_position_errors):.9f} m"
            )

            self.write(
                f"Pose/TF max yaw error:      "
                f"{max(self.pose_tf_yaw_errors):.9f} deg"
            )

        self.write("")
        self.write("--- SANITY CHECKS ---")

        checks = [
            (
                "Trajectory contains motion",
                self.total_path > 1.0,
            ),
            (
                "No >1 m localization jump",
                self.max_pose_step < 1.0,
            ),
            (
                "PointCloud frame remains base",
                self.cloud_frames == {"base"},
            ),
        ]

        if self.pose_odom_position_errors:
            checks.append(
                (
                    "Pose and Odom are consistent",
                    max(
                        self.pose_odom_position_errors
                    ) < 1e-6,
                )
            )

        if self.pose_tf_position_errors:
            checks.append(
                (
                    "Pose and TF are consistent",
                    max(
                        self.pose_tf_position_errors
                    ) < 1e-6,
                )
            )

        for name, passed in checks:
            self.write(
                f"{'PASS' if passed else 'REVIEW':6s}"
                f" - {name}"
            )

        self.write("")

        if all(value for _, value in checks):
            self.write(
                "RESULT: No runtime consistency "
                "failure detected."
            )
        else:
            self.write(
                "RESULT: One or more checks "
                "require review."
            )

        self.write("=" * 76)

    def close_report(self):
        self.report.flush()
        self.report.close()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Run the complete iPlow launch and record "
            "runtime consistency metrics."
        )
    )

    parser.add_argument(
        "--bag-path",
        required=True,
        help="Absolute path to the ROS2 bag directory.",
    )

    args = parser.parse_args()

    bag_path = str(
        Path(args.bag_path).expanduser().resolve()
    )

    repo = Path(__file__).resolve().parents[1]

    results = repo / "validation_results"
    results.mkdir(exist_ok=True)

    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    report_path = (
        results
        / f"full_validation_{timestamp}.txt"
    )

    rclpy.init()

    node = FullValidationMonitor(
        report_path,
        bag_path,
    )

    launch_command = [
        "ros2",
        "launch",
        "robot_bringup",
        "bag_localize.launch.py",
        f"bag_path:={bag_path}",
    ]

    node.write("")
    node.write(
        "Starting complete iPlow experiment..."
    )

    launch_process = subprocess.Popen(
        launch_command
    )

    try:
        # Wait until rosbag2_player actually appears.
        start_wait = time.monotonic()

        while (
            rclpy.ok()
            and not node.rosbag_running()
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if launch_process.poll() is not None:
                raise RuntimeError(
                    "Launch exited before "
                    "rosbag2_player started."
                )

            if (
                time.monotonic()
                - start_wait
                > 20.0
            ):
                raise RuntimeError(
                    "Timed out waiting for rosbag2_player."
                )

        node.write(
            "rosbag2_player detected."
        )
        node.write(
            "Monitoring until bag playback finishes..."
        )

        # Monitor automatically until bag playback ends.
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if (
                node.bag_seen
                and not node.rosbag_running()
            ):
                node.write("")
                node.write(
                    "rosbag2_player finished."
                )
                break

        # Allow final queued ROS messages to arrive.
        end_wait = time.monotonic() + 1.0

        while (
            rclpy.ok()
            and time.monotonic() < end_wait
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

    except KeyboardInterrupt:
        node.write("")
        node.write(
            "Validation interrupted by user."
        )

    except Exception as exc:
        node.write("")
        node.write(
            f"VALIDATION ERROR: {exc}"
        )

    finally:
        node.final_summary()

        if launch_process.poll() is None:
            node.write("")
            node.write(
                "Stopping ROS2 launch..."
            )

            launch_process.send_signal(
                signal.SIGINT
            )

            try:
                launch_process.wait(
                    timeout=10
                )
            except subprocess.TimeoutExpired:
                launch_process.terminate()

        node.write("")
        node.write(
            f"Report saved to:"
        )
        node.write(
            str(report_path)
        )

        node.close_report()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
