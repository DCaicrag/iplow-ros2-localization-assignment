# iPlow ROS2 Localization and Visualization

ROS2 Humble implementation for the iPlow Robot technical assignment.

This project reconstructs the robot pose from recorded GPS position and heading data, since dynamic odometry is not available in the provided ROS2 bag.

**Implements:**
- GPS-to-ENU localization
- Heading extraction from `/navheading`
- Dynamic `odom -> base` TF publication
- Robot URDF/Xacro and static TF hierarchy
- ROS2 launch orchestration
- RViz visualization of the moving LiDAR point cloud
- Unit tests + full-run runtime validation

The implementation intentionally focuses on the requested baseline solution rather than introducing SLAM or sensor-fusion frameworks.

---

## Quick Start

**Target environment:** Ubuntu 22.04 · ROS2 Humble · Python 3.10

### Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robot_bringup
source install/setup.bash
```

### Run

```bash
ros2 launch robot_bringup bag_localize.launch.py \
  bag_path:=/absolute/path/to/bag
```

This launches, in order: bag playback (`--clock`), `robot_state_publisher`, `joint_state_publisher`, the localization node, a static identity transform `map -> odom`, and RViz2 with the supplied configuration. The bag path is a launch argument, not hardcoded. The stack runs on simulated ROS time since the bag publishes `/clock`. The first valid GPS fix automatically becomes the local ENU origin.

---

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

- `map -> odom`: identity static transform
- `odom -> base`: published dynamically at ~10 Hz, from GPS (WGS84 → local ENU: x=East, y=North, z=Up) for position, and `/navheading`'s orientation quaternion for yaw
- Remaining frames come from the URDF via `robot_state_publisher`

**Outputs:** `/robot_pose` (`PoseStamped`), `/odom` (`Odometry`), dynamic TF `odom -> base` — all sharing the same pose estimate and timestamp.

---

## Repository Structure

```text
.
├── src/robot_bringup/
│   ├── CMakeLists.txt, package.xml, README.md
│   ├── launch/bag_localize.launch.py
│   ├── nodes/localization_math.py, localization_node.py
│   ├── rviz/config.rviz
│   ├── test/test_localization_math.py
│   └── urdf/robot.urdf.xacro
├── tools/run_full_validation.py
└── validation_results/full_validation_reference.txt
```

---

## Localization Design

**GPS reference:** the first valid fix is stored as `(ref_lat, ref_lon, ref_alt)`; all later fixes are expressed relative to it. Messages with missing or non-finite coordinates are ignored.

**GPS → ENU:** implemented in `localization_math.py` following the WGS84 approximation from the assignment, kept separate from ROS2 runtime logic so it can be unit tested independently.

**Heading:** `/navheading` is a `sensor_msgs/Imu` message; orientation is read from `msg.orientation` (ROS ordering `x, y, z, w`) and converted to yaw via `atan2` — no external quaternion library needed. Yaw is used directly as robot orientation, per the assignment's requirement.

Alternative heading conventions (e.g. `-yaw`, `yaw ± pi/2`, `yaw + pi`) were checked against GPS displacement and recorded velocity direction as sanity checks only, since the dataset has no independent dynamic orientation ground truth. No convention error was clearly evidenced, so the direct quaternion-to-yaw extraction was kept.

**Sensor QoS:** subscriptions use `BEST_EFFORT` reliability and `VOLATILE` durability for compatibility with bag-replayed topics.

---

## Robot Model and Static TF

The URDF reproduces the hierarchy `base -> body -> {gps, livox_frame -> livox_imu}`, with approximate visual geometry for the chassis, wheels, LiDAR, and GPS receiver (geometry is illustrative; the TF hierarchy and sensor-frame relationships are what matter for localization).

Static transforms were confirmed directly from the recorded `/tf_static` messages:

| Transform | x | y | z | roll | pitch | yaw |
|---|---|---|---|---|---|---|
| `base -> body` | 0.0569 | -0.0028 | 0.2234 | 0 | 0 | 0 |
| `body -> gps` | -0.6500 | 0.2000 | 0.1700 | 0 | 0 | 0 |
| `body -> livox_frame` | -0.1201 | 0.0026 | 0.9655 | π | 0 | π/2 |
| `livox_frame -> livox_imu` | 0 | 0 | 0 | 0 | 0 | 0 |

The `body -> livox_frame` rotation was recovered from the recorded quaternion and used in the URDF, replacing an initial development assumption of zero rotation.

---

## RViz Configuration

`config.rviz` is preconfigured with Fixed Frame `map`, Grid, TF, PointCloud2, Pose, and RobotModel displays:

- **PointCloud2:** topic `/mid360_filtered`, recorded frame `base`
- **Pose:** topic `/robot_pose`
- **RobotModel:** description `/robot_description`

The LiDAR point cloud is transformed into the global visualization frame through `map -> odom -> base`. Runtime visualization confirmed coherent robot movement and point-cloud behavior during bag playback.

---

## Dataset Notes

```text
Bag duration:           ~347.6 s
GPS samples:            347   (~1 Hz)
Heading samples:        347   (~1 Hz)
PointCloud2 samples:    3138  (~9 Hz)
```

First valid GPS fix: `lat 42.48871220, lon -83.55145510, alt 260.696 m` → becomes ENU origin `(0, 0, 0)`.

The bag also contains optional topics using `ublox_msgs`, `nmea_msgs`, and `rtcm_msgs`. These aren't required by the baseline pipeline, which only relies on `/ublox_gps_node/fix`, `/navheading`, `/mid360_filtered`, and `/joint_states`; if the optional packages aren't installed, `rosbag2_player` simply warns and skips those topics.

---

## Testing

**Localization math** (pure functions, testable without ROS2): ENU reference origin, East/North/altitude displacement, and quaternion→yaw at 0°, +90°, -90°, 180°.

**ROS2 package tests:**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select robot_bringup
colcon test-result --verbose
```

Result: `0 errors, 0 failures, 0 skipped`.

---

## Full-Run Runtime Validation

`tools/run_full_validation.py` starts before bag playback, launches `bag_localize.launch.py`, and monitors `/robot_pose`, `/odom`, dynamic TF, and PointCloud2 throughout playback in ~10 s segments, then writes a report to `validation_results/`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/run_full_validation.py --bag-path /absolute/path/to/bag
```

**Reference run** (346.50 s):

```text
Pose / Odometry / Dynamic TF messages:   3466 each
PointCloud messages:                     3138

Rates:
  /robot_pose            10.00 Hz
  /odom                  10.00 Hz
  odom -> base TF        10.00 Hz
  PointCloud2             9.06 Hz

Trajectory:
  Total path length       62.865 m
  Net displacement         2.242 m
  Max pose step             0.443 m
  Unique motion updates       342

Pose/Odom/TF cross-consistency: 0.000000000 m and 0.000000000 deg error
```

Sanity checks: trajectory contains motion; no jump >1 m; point-cloud frame stays `base`; pose/odom/TF consistent. **No runtime consistency failure detected.** Full report: `validation_results/full_validation_reference.txt`.

### Validation levels

1. **Mathematical correctness** — unit-tested GPS→ENU and quaternion→yaw, independent of ROS2.
2. **Dataset consistency** — GPS trajectory inspected numerically; no implausible discontinuities.
3. **ROS runtime consistency** — `/robot_pose`, `/odom`, and TF stayed internally consistent for the full bag.
4. **Visualization** — RobotModel, TF, pose, and point cloud inspected together in RViz; trajectory and point-cloud motion looked coherent.

**Limitation:** the dataset has no independent dynamic ground truth (mocap, survey trajectory, independent odometry), so absolute metrics like RMSE/ATE/RPE cannot be computed meaningfully. Validation here covers mathematical correctness, dataset continuity, and runtime/TF/visual consistency — not absolute localization accuracy.

---

## Notable Engineering Decisions

- **RViz over remote X11:** during one long remote validation run, RViz crashed with a graphical segfault (exit code -11) partway through, while `rosbag2_player`, the localization node, TF publication, and the validation monitor all completed successfully (3138 point-cloud messages, all consistency checks passed). Treated as a remote graphical-session issue, not a localization failure. RViz runs normally on local Ubuntu execution.

---

## Assumptions

1. The first valid GPS fix is an appropriate local ENU reference origin.
2. `/navheading.msg.orientation` is the heading quaternion intended by the assignment.
3. Direct quaternion-to-yaw extraction is the appropriate baseline heading implementation.
4. The recorded `/tf_static` transforms represent the required static sensor geometry.
5. Since `/mid360_filtered` is recorded in `base`, global point-cloud visualization depends primarily on the reconstructed `map -> odom -> base` chain.
6. GPS + `/navheading` are sufficient for the requested baseline localization.
7. The LiDAR IMU is optional and not fused into the baseline estimator.

---

## Scope and Possible Extensions

Out of scope for this baseline: SLAM, FAST-LIO, LIO-SAM, EKF sensor fusion, factor graphs, advanced IMU integration, scan matching, external mapping frameworks.

Possible future extensions: synchronized GPS/heading processing, IMU-assisted orientation estimation, EKF-based state estimation, LiDAR odometry, local/global SLAM, ground-truth comparison (if an independent reference trajectory becomes available), and automated trajectory plotting/quantitative evaluation.

---

## Final Status

```text
GPS -> ENU                  PASS
Quaternion -> yaw           PASS
Dynamic odom -> base TF     PASS
/robot_pose publication     PASS
/odom publication           PASS
Static TF / URDF            PASS
ROS2 Humble build           PASS
ROS2 tests                  PASS
PointCloud2 reception       PASS
TF runtime consistency      PASS
RViz visualization          PASS
Integrated launch           PASS
Full bag runtime test       PASS
```

The solution is considered complete for the requested baseline scope.
