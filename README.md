# 3D LiDAR Odometry

C++17 / ROS 2 LiDAR odometry for the University of Stuttgart Advanced Mobile
Robotics project. The package subscribes to `sensor_msgs/PointCloud2`, estimates
the sensor's 6DoF pose, and publishes `odom <- cloud_frame` through TF2.

The implementation uses robust ICP against a bounded local map, with an
Eigen-only core and a ROS 2 adapter for messages, TF and trajectory output.
Deterministic tests cover registration, tracking and input decoding. CI builds
the ROS wrapper on Humble and Jazzy and runs a synthetic end-to-end smoke test.
Dataset files are excluded from version control.

## Build

Use a Linux ROS 2 Humble or Jazzy environment (Ubuntu 22.04 or 24.04 respectively).
On macOS, the core can be tested independently; the full package needs a ROS 2
environment, such as a Linux machine or VM.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
git clone https://github.com/ece-akdeniz/3D-LiDAR-Odometry.git
cd ~/lidar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If `rosdep` has not been initialized on this machine, follow the ROS installation
instructions for `rosdep init` / `rosdep update` first. The package name is
`st3821671`; verify this matches your student number before submission.

## Run

```bash
ros2 launch st3821671 odom.launch.py points_topic:=/points rviz:=true
```

For each dataset, inspect its topic names before launching. Start odometry first,
then play the bag in a second terminal with the workspace sourced:

```bash
ros2 bag info /path/to/dataset1
ros2 launch st3821671 odom.launch.py \
  points_topic:=/actual/cloud/topic use_sim_time:=true rviz:=true \
  trajectory_path:=/tmp/dataset1-estimate.txt
# Second terminal:
ros2 bag play /path/to/dataset1 --clock --rate 0.5
```

Use a new trajectory path for every run: output **appends** to an existing file.
Restart the node before replaying or looping a bag, because non-increasing scan
timestamps are dropped. If a bag also publishes a transform for this same sensor
frame, avoid running two publishers for the same TF edge. Select the cloud topic
manually in RViz when using a custom topic set inside YAML instead of the launch
topic argument. The RViz fixed frame is `odom` by default.

Launch arguments:

- `params_file`: installed `config/odom.yaml` by default.
- `points_topic`: remaps `/points` to the bag's cloud topic.
- `rviz`: `false` by default.
- `use_sim_time`: `false` by default; enable with `ros2 bag play --clock`.
- `save_trajectory`: `true` by default.
- `trajectory_path`: `./estimate.txt` by default, relative to the launch directory.

The launch-level save/path arguments take precedence over the YAML file. All
algorithm parameters are loaded at startup; change YAML and restart to retune.
Runtime parameter changes are explicitly rejected, except `use_sim_time`.

## Outputs and coordinate convention

- `/tf`: `T_odom_cloud`, transforming points from the incoming cloud frame into
  `odom`. The first valid scan defines the origin. The child frame is read from
  `header.frame_id`; a changed frame is rejected instead of mixing sensors.
- `/odometry`: `nav_msgs/Odometry`, stamped with the incoming cloud timestamp.
- `/trajectory`: `nav_msgs/Path`, capped at `max_path_poses` for bounded memory.
- TUM text: `timestamp tx ty tz qx qy qz qw`, with nanosecond timestamp precision.

Odometry pose covariance is a conservative placeholder, **not a calibrated
uncertainty estimate**. Twist is not estimated. Rejected alignments publish the
held pose with a large covariance, retain the previous map, and log a warning.
Their held poses are also written to TUM output. No TF or trajectory is published
until the first usable scan initializes the map. Malformed messages and wrong
frames are dropped.

## Algorithm

1. Decode PointCloud2 fields safely, including organized clouds, row padding,
   mixed numeric field types, byte order, and unaligned offsets.
2. Remove invalid coordinates, filter by range and intensity, cap input sampling,
   and compute voxel centroids.
3. Align each scan to a local map using point-to-point ICP and an exact balanced
   kd-tree. Weighted SVD solves the rigid transform; Huber weights and distance
   trimming reduce outlier influence. The previous accepted pose initializes ICP.
4. Accept only converged alignments with sufficient overlap, low residual, and
   plausible motion. Collinear correspondences are rejected. A rejected scan
   cannot contaminate the map.
5. Add keyframes after enough translation or rotation. Retain a fixed number of
   recent keyframes, with a capped voxel map and capped display path.

The core depends only on Eigen and the C++ standard library; it uses no PCL,
Open3D, Ceres, or ROS types. The ROS adapter handles messages, TF and files.

## Tuning

Defaults are in [`st3821671/config/odom.yaml`](st3821671/config/odom.yaml).
Distances are meters and angles are radians.

- `min_range`, `max_range`, `intensity_threshold`: sensor filtering. A missing
  intensity field is allowed at the default zero threshold; a positive threshold
  requires that field. Intensity units depend on the sensor.
- `voxel_size`, `max_input_points`, `max_scan_points`: trade detail for speed.
- `max_correspondence_distance`: maximum nearest-neighbor match distance.
- `huber_delta`, `trim_fraction`: robust weighting and retained match fraction.
- `min_correspondences`, `min_overlap`, `max_rmse`: alignment quality gates.
- `max_iterations`, `translation_epsilon`, `rotation_epsilon`: ICP stopping rules.
- `max_translation_step`, `max_rotation_step`: maximum change from the last accepted
  pose. A long dropout can require wider gates or a restart.
- `keyframe_translation`, `keyframe_rotation`, `max_keyframes`, `max_map_points`:
  map insertion thresholds and memory limits.

Playback speed changes processing load, not the geometric displacement between
consecutive recorded scans. Slower replay helps when processing cannot keep up;
the subscription has a short best-effort queue to avoid accumulating stale scans.

Limitations: no IMU fusion, scan deskewing, motion prediction, loop closure,
global relocalization, or built-in ground-truth evaluation. Fast motion, repeated
geometry, moving objects, corridors and low overlap can cause drift or tracking
loss. Point-to-point correspondence rank checks cannot detect every environmental
ambiguity. Holding the last pose limits individual failures but does not prove a
globally bounded or accurate trajectory.

## Tests

Eigen-only build, no ROS required:

```bash
cmake -S st3821671 -B build/core -DBUILD_ROS2=OFF -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/core --parallel
ctest --test-dir build/core --output-on-failure
```

ROS tests, after building and sourcing the workspace:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select st3821671
colcon test-result --verbose
python3 src/3D-LiDAR-Odometry/scripts/ros_smoke_test.py
```

The core suite checks exact kd-tree search against brute force, identity and
known 6DoF transforms, outliers, degeneracy and motion gates, a 20-frame trajectory,
map limits, failed-scan recovery, reset, and malformed/endian/padded PointCloud2
layouts. The ROS smoke test launches the installed package with a remapped input,
publishes 20 synthetic scans, and checks pose accuracy, TF direction, path length,
quaternion normalization and TUM output.

## Dataset evaluation

Keep dataset files outside Git (`data/` and bag extensions are ignored). The
course evaluation sequences cover one room, one floor, and three floors. For
each bag, inspect the fields and topic, run at a rate your machine can sustain,
check that TF follows the sensor frame, inspect the trajectory in RViz, and record
rejections, discontinuities, processing time and final drift. Compare against
ground truth if supplied. Tune on the room sequence first, then verify the floor
and three-floor sequences. Report measured accuracy and runtime separately from
the synthetic test suite.

The lecture PDF and course datasets are excluded from this public repository.
This source is MIT licensed; that license does not cover course materials or data.

References: [ROS PointCloud2 definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/PointCloud2.msg),
[TF2 broadcaster API](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros/include/tf2_ros/transform_broadcaster.hpp),
[Eigen](https://eigen.tuxfamily.org/).
