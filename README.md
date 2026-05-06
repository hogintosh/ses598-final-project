# Mars RVIO Evaluation Pipeline

This package contains the final SES 598 Range-Visual-Inertial Odometry testbed.
It runs a PX4/Gazebo Mars terrain drone mission and evaluates an OpenVINS-based
RVIO estimate against Gazebo ground truth.

The current supported estimator path is OpenVINS with range-feature support. The
old Python EKF-lite, cylinder landing, ArUco, RTAB-Map, and spiral trajectory
assignment code has been pruned from this package.

## What Runs

The main launch starts:

- PX4 SITL with the `x500_ingenuity_mars` model.
- Gazebo with `mars_px4.sdf`.
- Micro XRCE-DDS Agent for PX4 ROS 2 topics.
- ROS-Gazebo bridges for the downward mono camera, 1D rangefinder, clock, and
  Gazebo ground-truth odometry.
- The RVIO test mission.
- The PX4 IMU to OpenVINS IMU bridge.
- The OpenVINS-on-SURVEY launcher.
- The RVIO evaluator and live `rqt_plot` helpers.

Primary sensor topics:

```text
/drone/down_mono
/drone/down_mono/camera_info
/drone/down_rangefinder
/openvins/imu
/sim/ground_truth/vehicle_odometry
```

Primary RVIO outputs:

```text
/rvio/odometry
/rvio/pose
/rvio/path
/rvio/debug_image
/rvio/points_slam
/rvio/points_msckf
```

Primary live metric topics:

```text
/rvio_plot/estimated_x
/rvio_plot/ground_truth_x
/rvio_plot/estimated_y
/rvio_plot/ground_truth_y
/rvio_plot/estimated_vx
/rvio_plot/ground_truth_vx
/rvio_plot/estimated_vy
/rvio_plot/ground_truth_vy
/rvio_eval/xy_rmse
/rvio_eval/velocity_rmse
/rvio_eval/drift_percent
```

## Build

From the ROS 2 workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select terrain_mapping_drone_control --symlink-install
source install/setup.bash
```

## Deploy PX4 Assets

Copy the custom airframe, Gazebo model, and Mars world into PX4-Autopilot:

```bash
cd ~/ros2_ws/src/ses598-final
./scripts/deploy_px4_model.sh -p ~/PX4-Autopilot
```

The current PX4 model is:

```text
x500_ingenuity_mars
```

The current PX4 world is:

```text
mars_px4
```

## Run

Use the visible Gazebo run when you want to inspect the vehicle and terrain:

```bash
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py \
  mission_mode:=rvio_test \
  rvio_backend:=openvins \
  headless:=0 \
  require_preflight_checks:=false \
  start_rvio:=true \
  start_rvio_evaluator:=true \
  start_rvio_rqt_plots:=true
```

Despite the historical filename, `cylinder_landing.launch.py` is now the main
Mars RVIO evaluation launch. It no longer spawns cylinder targets by default or
starts the old object-landing pipeline.

## Evaluation Frame

The evaluator compares relative motion. When `/mission/state` reaches `SURVEY`,
Gazebo ground-truth x/y is zeroed at the current drone pose. RVIO x/y is then
compared in the same local mission frame.

This avoids comparing an estimator-local coordinate system against raw Gazebo
world offsets.

Altitude is treated separately. The downward lidar is a 1D range ray, not a
Gazebo world-z measurement and not a dense depth image. The landing-relevant
plots focus on lateral position and velocity:

```bash
ros2 run rqt_plot rqt_plot \
  /rvio_plot/ground_truth_x/data /rvio_plot/estimated_x/data

ros2 run rqt_plot rqt_plot \
  /rvio_plot/ground_truth_y/data /rvio_plot/estimated_y/data
```

## RVIO/OpenVINS Configuration

The main estimator configuration is:

```text
config/openvins_mars_estimator.yaml
```

Range-feature support subscribes to `/drone/down_rangefinder`, time-gates the
latest lidar sample against the camera frame, tags features near the configured
lidar pixel, and initializes those feature depths with the lidar range. The
remaining pose correction is handled by OpenVINS feature reprojection updates
and covariance propagation.

The delayed launcher is:

```text
terrain_mapping_drone_control/openvins_on_state_launcher.py
```

It waits for `/mission/state == SURVEY` before starting OpenVINS.

## Report

The project report is in:

```text
systemreport.md
```
