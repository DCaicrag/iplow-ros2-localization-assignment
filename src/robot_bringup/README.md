# iPlow ROS2 Localization and Visualization

ROS2 Humble implementation for the iPlow Robot technical assignment.

This package reconstructs the robot pose from recorded GPS position and heading data when dynamic odometry is not available in the ROS2 bag. It also provides the TF tree, robot model, launch orchestration, and RViz configuration required to visualize the moving LiDAR point cloud in a global frame.

## Overview

The localization pipeline uses:

* `/ublox_gps_node/fix` for robot position.
* `/navheading` for robot heading.
* The first valid GPS fix as the local reference origin.
* A WGS84-based conversion from geodetic coordinates to local East-North-Up (ENU) coordinates.
* The quaternion contained in `/navheading` to recover robot yaw.
* A dynamic `odom -> base` transform published at approximately 10 Hz.

The localization node publishes:

* `/robot_pose` — `geometry_msgs/PoseStamped`
* `/odom` — `nav_msgs/Odometry`
* dynamic TF `odom -> base`

The package additionally provides:

* a URDF/Xacro robot model;
* static robot frame geometry;
* `robot_state_publisher`;
* `joint_state_publisher`;
* a static identity transform `map -> odom`;
* an integrated ROS2 launch file;
* a preconfigured RViz visualization.

## Architecture

```text
map
└── odom
    └── base
        └── body
            ├── gps
            └── livox_frame
                └── livox_imu
```

`map -> odom` is an identity static transform.

`odom -> base` is generated dynamically from:

```text
GPS
 ↓
WGS84 geodetic coordinates
 ↓
local ENU position
 ↓
x = East
y = North
z = Up
```

and:

```text
/navheading
 ↓
orientation quaternion
 ↓
yaw
```

The remaining robot transforms are provided by the URDF through `robot_state_publisher`.

## Package Structure

```text
robot_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── bag_localize.launch.py
├── nodes/
│   ├── localization_math.py
│   └── localization_node.py
├── rviz/
│   └── config.rviz
├── test/
│   └── test_localization_math.py
├── urdf/
│   └── robot.urdf.xacro
└── README.md
```

## Localization Design

### GPS Reference

The first valid GPS fix is stored as:

```text
(ref_lat, ref_lon, ref_alt)
```

Subsequent GPS measurements are expressed relative to this reference.

Messages without a valid GPS fix or with non-finite coordinates are ignored.

### GPS to ENU

The conversion implemented in `localization_math.py` follows the WGS84 approximation provided in the assignment.

The resulting local coordinates follow the ENU convention:

```text
x = East
y = North
z = Up
```

### Heading

`/navheading` is a `sensor_msgs/Imu` message.

The orientation is read from:

```text
msg.orientation
```

ROS quaternion ordering is handled explicitly as:

```text
(x, y, z, w)
```

The quaternion is converted to yaw using a direct mathematical implementation based on `atan2`, avoiding an additional runtime dependency.

### Pose Publication

Once both position and heading are available, the node publishes at approximately 10 Hz:

```text
/robot_pose
/odom
odom -> base
```

The same timestamp and pose estimate are used for all three outputs.

## Sensor QoS

Sensor subscriptions use:

```text
ReliabilityPolicy.BEST_EFFORT
DurabilityPolicy.VOLATILE
```

to improve compatibility with sensor topics recorded in ROS2 bags.

## Robot Model

The URDF reproduces the frame hierarchy specified in the assignment:

```text
base
└── body
    ├── gps
    └── livox_frame
        └── livox_imu
```

Frame translations use the values provided as ground truth by the assignment.

Approximate visual geometry is included for:

* chassis;
* wheels;
* Livox LiDAR;
* GPS receiver.

### Current Static-TF Assumption

The assignment document explicitly provides the frame translations but does not provide the corresponding frame rotations.

Therefore, the initial URDF uses:

```text
rpy="0 0 0"
```

for the static joints.

These rotations must be compared against the actual `/tf_static` messages in the provided ROS2 bag during runtime validation.

## Launch File

`bag_localize.launch.py` is designed to start:

1. ROS2 bag playback with `--clock`;
2. `robot_state_publisher`;
3. `joint_state_publisher`;
4. the custom localization node;
5. static identity transform `map -> odom`;
6. RViz2 with the provided configuration.

The bag location is supplied as a launch argument rather than being hardcoded.

Expected usage:

```bash
ros2 launch robot_bringup bag_localize.launch.py \
  bag_path:=/absolute/path/to/bag
```

The launch file defaults to:

```text
use_sim_time:=true
```

because the bag is played using `/clock`.

## RViz Configuration

The supplied RViz configuration contains:

* Fixed Frame: `map`
* Grid
* TF
* PointCloud2:

  * topic `/mid360_filtered`
* Pose:

  * topic `/robot_pose`
* RobotModel:

  * description `/robot_description`

The final visualization parameters may be adjusted after runtime validation with the provided bag.

## Offline Validation Completed

The following checks have already been completed without the final bag integration:

* Python syntax validation.
* WGS84 GPS-to-ENU unit tests.
* Quaternion-to-yaw unit tests.
* ENU origin test.
* ENU altitude-axis test.
* yaw `0°` test.
* yaw `+90°` test.
* yaw `-90°` test.
* yaw `180°` test.
* 6/6 localization mathematics tests passing.
* URDF XML parsing.
* URDF tree validation using `check_urdf`.
* `package.xml` XML validation.
* RViz YAML parsing.
* repository whitespace validation.

## Runtime Validation Pending

The following items require validation against the actual ROS2 bag and ROS2 Humble runtime:

* clean `colcon build`;
* installed Python module resolution;
* actual bag metadata;
* topic types and frequencies;
* recorded QoS profiles;
* actual `frame_id` values;
* actual `/tf_static` rotations;
* GPS validity behavior;
* `/navheading` convention;
* heading sign and angular reference;
* `odom -> base` runtime behavior;
* `/robot_pose` publication;
* `/odom` publication;
* TF tree consistency;
* RViz configuration loading;
* moving `/mid360_filtered` point cloud;
* coherent global point-cloud mapping;
* complete launch execution.

The final runtime validation is intentionally kept separate from the offline validation so that assumptions about the dataset are not presented as confirmed behavior.

## Build and Run

The final tested build and run commands will be documented after validation on the target ROS2 Humble / Ubuntu 22.04 environment.

Expected workflow:

```bash
cd ros2_ws

source /opt/ros/humble/setup.bash

# Install dependencies if required.
# rosdep commands will be documented after validation.

colcon build

source install/setup.bash

ros2 launch robot_bringup bag_localize.launch.py \
  bag_path:=/absolute/path/to/bag
```

## Testing

Offline localization mathematics tests can be executed with:

```bash
python3 -m pytest \
  src/robot_bringup/test/test_localization_math.py \
  -v
```

Current result:

```text
6 passed
```

## Design Principles

The implementation intentionally prioritizes the scoped requirements of the assignment.

It does not currently introduce:

* SLAM;
* FAST-LIO;
* LIO-SAM;
* EKF sensor fusion;
* advanced IMU fusion;
* external mapping frameworks.

The optional `/mid360_imu` input is not currently used because GPS and `/navheading` are sufficient for the required baseline solution.

Additional localization or sensor-fusion methods should only be considered after the required solution is validated.

## Known Assumptions

Current assumptions that require confirmation with the recorded dataset:

1. Static-frame rotations are currently initialized to zero because only translations were explicitly provided in the assignment document.
2. The yaw extracted from `/navheading` is provisionally used directly as the robot orientation in the ENU frame.
3. The exact relationship between the heading convention and ENU axes must be verified against robot motion in the bag.
4. The RViz configuration has been validated as YAML but still requires runtime validation in RViz2.

These assumptions will be updated after inspecting the recorded ROS2 data.

## Target Environment

```text
Ubuntu 22.04
ROS2 Humble
Python 3.10
```
