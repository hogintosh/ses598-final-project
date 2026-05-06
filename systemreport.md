# RVIO System Report

## Project Motivation

This project built a proof-of-concept Range-Visual-Inertial Odometry pipeline for a Mars terrain drone in PX4/Gazebo. The goal was to study whether a downward monocular camera, IMU, and 1D downward lidar could estimate lateral velocity and lateral position over terrain where simple optical-flow assumptions are fragile.

The motivation is closely tied to NASA Ingenuity's final flight. During Flight 72, Ingenuity was operating over steep, relatively feature-poor sand ripples. NASA's accident investigation describes the navigation system as relying on tracked visual surface features to estimate position, velocity, and attitude; when feature tracking became unreliable, the vehicle touched down with excessive lateral velocity and damaged its rotor blades. That failure mode is exactly the kind of landing-risk signal we wanted to expose in simulation: if lateral velocity or lateral position is wrong during descent, the landing can fail even when altitude control appears reasonable.

Our simulation is not a full reproduction of Ingenuity or the complete Mars Science Helicopter navigation stack. Instead, it is an engineering testbed for the core RVIO question: can a range-constrained visual-inertial estimator use a 1D lidar measurement to improve the scale and geometry of monocular feature tracks, especially over structurally invariant or non-planar terrain?

## What We Built

The final system runs a PX4/Gazebo Mars-terrain mission and evaluates an RVIO estimate against Gazebo ground truth in real time. The estimator output is exposed as:

```text
/rvio/odometry
/rvio/pose
/rvio/path
/rvio/debug_image
```

The metrics are exposed as live ROS 2 topics for `rqt_plot`, including:

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

The intended visual result is simple and useful: plot estimated x against ground-truth x over time, plot estimated y against ground-truth y over time, and inspect whether the estimate diverges, converges, or remains biased during the survey and landing sequence. In the final representation we were trying to obtain, the local x/y estimate returns toward zero after an arbitrary flight path, showing that the estimator can recover the vehicle's relative motion in a local mission frame.

## Simulation Pipeline

The simulation uses PX4 SITL, Gazebo, ROS 2, and a custom Mars terrain world. The main launch entry point is:

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

The launch file starts the major pieces:

- PX4 SITL with the `x500_ingenuity_mars` model.
- Gazebo with the Mars terrain world.
- Micro XRCE-DDS Agent for PX4 ROS 2 topic transport.
- ROS-Gazebo bridges for the downward monocular camera, downward lidar, clock, and Gazebo ground-truth odometry.
- The terrain-relative mission node.
- The OpenVINS-based RVIO backend.
- The RVIO evaluator and live `rqt_plot` windows.

The drone has:

- A downward-facing monocular camera published on `/drone/down_mono`.
- A downward-facing 1D lidar/rangefinder published on `/drone/down_rangefinder`.
- PX4 IMU data from `/fmu/out/sensor_combined`, converted to `sensor_msgs/Imu` on `/openvins/imu`.
- Gazebo ground-truth odometry on `/sim/ground_truth/vehicle_odometry`.

The downward camera and lidar are treated as co-aligned near the image center. This is a critical assumption because the range measurement is only valid along one ray. The lidar does not produce a dense depth image, and it should not be treated as a generic altimeter when the vehicle is pitched or rolled. Its value is a range along the lidar beam to the terrain surface.

## Mission Design and Reasoning

Early tests showed that taking metrics immediately at spawn polluted the estimator and evaluator. The vehicle starts near or above structures and terrain handoff artifacts, and the lidar can return misleading readings before the test segment begins. We therefore separated the mission into phases:

1. Startup and PX4/Gazebo synchronization.
2. Takeoff and climb.
3. A 5 m relative climb before RVIO metrics start.
4. A `SURVEY` phase where RVIO and the evaluator are activated.
5. A controlled horizontal survey path.
6. Return and descent.

The reason for the 5 m climb is practical: we wanted the estimator to begin after the vehicle was clearly above the terrain and after the platform/spawn transients were gone. This made the lidar data more representative of terrain range and kept the metrics focused on lateral odometry rather than launch artifacts.

We also added a single-axis survey mode. The reason was diagnostic clarity. When the vehicle moves only along x or only along y, a bad estimator becomes easier to diagnose:

- If the drone moves in y and estimated y does not change, the visual geometry or coordinate transform is wrong.
- If the estimated velocity is much slower than ground truth, the depth or feature scaling is wrong.
- If the estimate drifts during constant velocity, the IMU prediction is dominating without enough visual/range correction.

This single-axis test was used to expose the weakness of the earlier optical-flow shortcut and to motivate moving closer to a real MSCKF/OpenVINS formulation.

## Ground Truth and Metrics

Gazebo publishes the drone pose in the simulation world frame. However, the raw Gazebo world frame contains offsets that are not meaningful for the RVIO local estimate. RVIO estimates relative motion from its own local initialization frame, so comparing raw Gazebo x/y directly to RVIO x/y would produce false errors.

The evaluator therefore uses a relative local frame:

- When `/mission/state` reaches `SURVEY`, the evaluator locks the current Gazebo x/y pose as the ground-truth origin.
- Ground-truth x/y are then published relative to that origin.
- RVIO x/y are interpreted in the estimator's local frame.
- The two local trajectories are compared over time.

This lets the plots answer the actual question: from the start of the survey, did RVIO estimate the same lateral displacement and velocity that Gazebo reports?

The evaluator publishes:

- Horizontal position error.
- Horizontal velocity error.
- XY RMSE.
- Velocity RMSE.
- Drift percentage relative to ground-truth path length.
- AGL error against the downward lidar.

Altitude is handled carefully. Gazebo z is the world-frame height of the model, while the downward lidar measures range-to-terrain along the sensor ray. Over non-planar Mars terrain these are not the same quantity. For this reason, the main landing-relevant metrics emphasize x/y position and x/y velocity, while lidar range is used as local AGL/range information.

## First RVIO Implementation: Python Proof of Concept

The first implementation was a Python RVIO proof-of-concept node. It fused IMU, optical flow, attitude, lidar, and ground truth for debugging. This was useful because it gave us quick visibility into the sensor streams and let us publish `/rvio/debug_image`, `/rvio/odometry`, and metric topics quickly.

The early approach included a simplified "EKF-lite" style estimator:

- IMU acceleration propagated velocity and position.
- Downward camera optical flow estimated horizontal motion.
- The downward lidar was used to scale visual motion.
- PX4 attitude was used to compensate some rotation and align the camera/lidar geometry.

This was valuable as a stepping stone, but it was not close enough to the paper. The main problem was that optical flow plus a single range value becomes fragile over non-planar or structurally invariant terrain. If the feature being tracked is not actually located along the lidar ray, using the lidar as that feature's depth is wrong. When the vehicle pitches, rolls, or flies over uneven terrain, treating range as simple altitude causes incorrect lateral scale and can cause severe x/y drift or underestimation.

The Python implementation helped identify the core failure mode: the system was behaving too much like a visual-inertial/optical-flow shortcut, not like a range-feature estimator.

## Move Toward the Paper's Geometry

The paper, "Structure-Invariant Range-Visual-Inertial Odometry," motivates using sparse range measurements to constrain the depth of visual features rather than assuming a planar scene. That is the essential distinction:

- A planar optical-flow method assumes one dominant ground plane or one representative altitude.
- RVIO uses range-associated visual features so that individual feature depths can be constrained.
- The estimator should maintain pose, velocity, IMU biases, camera/IMU geometry, feature states, and covariance relationships.

The important idea is not merely "use lidar for altitude." The important idea is "use the 1D lidar range to collapse the depth uncertainty of visual features that are geometrically associated with the lidar ray." Once a feature has a range-constrained depth, the visual reprojection residual becomes a stronger constraint on camera motion and therefore vehicle pose.

This matters over structurally invariant terrain because monocular vision alone has weak scale observability, and simple flow-to-velocity conversion is unreliable when the terrain is not planar. Range-feature association adds metric scale to selected visual tracks without requiring a dense depth camera or a full 3D lidar.

## OpenVINS Integration

To get closer to a real visual-inertial estimator, we integrated OpenVINS as the RVIO backend. OpenVINS provides the MSCKF-style VIO foundation that the Python proof-of-concept did not:

- IMU propagation of a full 3D state.
- Camera clone management.
- KLT feature tracking.
- Monocular feature update machinery.
- Feature representations including anchored inverse depth.
- Covariance propagation and Kalman updates across the state.

The launch argument `rvio_backend:=openvins` selects this backend. The OpenVINS node publishes its odometry under the RVIO namespace:

```text
odomimu -> /rvio/odometry
poseimu -> /rvio/pose
pathimu -> /rvio/path
trackhist -> /rvio/debug_image
points_slam -> /rvio/points_slam
points_msckf -> /rvio/points_msckf
```

We added `openvins_on_state_launcher.py` so OpenVINS does not start immediately at simulation boot. Instead, it waits for the mission state to become `SURVEY`, then launches `ov_msckf run_subscribe_msckf`. The reason is that OpenVINS initialization and the metrics should happen during the controlled test segment, not during spawn, takeoff, or transient climb behavior.

We also added `px4_imu_bridge_node.py` to convert PX4 `SensorCombined` messages into `sensor_msgs/Imu`, which is the message format OpenVINS expects. A major issue was timestamp alignment: PX4 IMU timestamps and Gazebo camera timestamps were not naturally in the same time base. The bridge now re-anchors the PX4 IMU timestamp to the latest camera time so long pre-survey waits do not create a large camera/IMU offset. This was necessary because OpenVINS is very sensitive to IMU-camera timing.

## Range-Feature OpenVINS Modifications

OpenVINS was extended with a range-feature path. The configuration is in:

```text
config/openvins_mars_estimator.yaml
```

Key range-feature parameters include:

```yaml
range_feature_enable: true
range_feature_cam_id: 0
range_feature_time_gate: 0.08
range_feature_pixel_gate: 24.0
range_feature_max_per_frame: 1
range_feature_depth_std: 0.25
range_feature_lidar_u: 400.0
range_feature_lidar_v: 300.0
topic_range: "/drone/down_rangefinder"
```

The modified OpenVINS range-feature flow is:

1. Subscribe to `/drone/down_rangefinder` as a `LaserScan`.
2. Select the finite valid range sample.
3. Store the latest range and timestamp inside the VIO manager.
4. During each monocular image update, inspect tracked camera features.
5. Find features close to the projected lidar pixel.
6. If the image timestamp and lidar timestamp are close enough, tag the nearby feature with lidar depth.
7. During SLAM feature initialization, use the lidar-provided depth to initialize the anchored feature position.
8. Skip normal monocular depth refinement for that range-seeded feature so the lidar depth remains the metric initialization.
9. Let the OpenVINS update machinery use the feature reprojection residuals and covariance to correct the vehicle state.

This is the central RVIO adaptation. We are no longer using the lidar only as altitude. We are using it to initialize the depth of visual features that lie near the lidar ray, which is closer to the paper's range-feature concept.

## Why the Lidar Is Still One-Dimensional

The lidar remains one-dimensional. It does not provide a depth map and it does not assign depth to every tracked feature. At most, it provides range for the visual feature or features that are close enough to the lidar ray in the image at the same time.

This creates several assumptions:

- The camera and lidar extrinsics must be approximately correct.
- The lidar ray must project near the configured camera pixel.
- The feature selected near that pixel must lie on the same terrain surface hit by the lidar.
- The vehicle orientation estimate must be good enough to preserve the geometry.
- The time offset between image, IMU, and range must be small.

When these assumptions hold, the range-feature update can reduce monocular scale ambiguity. When they do not hold, the range measurement can be associated with the wrong image feature and damage the estimate.

## Why Orientation Matters

A key lesson from the project was that lateral position cannot be separated from attitude. If the vehicle pitches or rolls, the downward camera optical flow and the lidar ray change their relationship to the terrain. A range value measured along a tilted lidar ray is not the same as vertical altitude. Similarly, a pixel displacement in the image cannot be converted into world x/y motion unless the camera orientation is understood.

This is why the final direction moved away from a pure optical-flow velocity shortcut and toward OpenVINS. The estimator needs a full 3D pose state:

- Position.
- Velocity.
- Orientation.
- IMU biases.
- Camera clone poses.
- Feature depth states or marginalized feature constraints.

The x/y plots are the easiest landing-risk diagnostic, but the estimator itself must estimate full pose. Otherwise, the relation between range, feature bearing, and lateral motion is geometrically inconsistent.

## RQT Plot Interface

CSV logs were disabled by default because the goal became live diagnosis rather than offline-only analysis. The evaluator publishes continuously updating `Float32` topics for `rqt_plot`. This made it possible to watch:

- Estimated x and ground-truth x on the same plot.
- Estimated y and ground-truth y on the same plot.
- Estimated vx and ground-truth vx.
- Estimated vy and ground-truth vy.
- Final estimated x/y against final ground-truth x/y.

The evaluator also publishes ground truth even when RVIO odometry has not arrived yet. This was important because an empty plot could mean either "RVIO is not publishing" or "the plotter did not receive anything." Publishing ground truth independently makes the failure mode visible.

The RQT launch helper waits until the mission reaches the plotting phase and then opens the plot windows. This keeps stale plot windows from earlier runs from misleading the operator.

## Assumptions and Limitations

This project makes several explicit assumptions:

- The downward mono camera and 1D lidar are approximately aligned.
- The configured lidar projection pixel is close to the true camera pixel hit by the lidar ray.
- The simulation terrain provides enough visual texture for OpenVINS to track sparse features.
- Gazebo ground truth is valid for evaluation, but only after converting it into a local survey-start frame.
- Lidar range is valid as a ray measurement, not as raw Gazebo world z.
- The OpenVINS camera intrinsics and IMU-camera extrinsics are sufficiently close for a proof of concept.
- The estimator is evaluated on relative x/y displacement and velocity, not global Mars coordinates.

The main limitation is that the range-feature association is still sparse and fragile. A single 1D lidar can only collapse feature depth when a tracked feature is close to the lidar ray. If the vehicle is moving quickly, if the feature density is low, if the terrain is visually repetitive, or if the projected lidar pixel is wrong, the estimator may fall back toward ordinary monocular VIO behavior.

Another limitation is initialization. OpenVINS needs a usable feature window and acceptable IMU/camera timing before it can publish odometry. Starting it exactly at the survey motion can leave it trying to initialize while the image disparity is already large. A stronger final system would include a dedicated survey-initialization phase: hover or move slowly after climb, allow OpenVINS to initialize, then begin the metrics path.

## Resources Used

The main system-design resource was:

- "Structure-Invariant Range-Visual-Inertial Odometry," provided locally as `Final_SES_RVIO.pdf`.

The simulation resources included:

- PX4 SITL for vehicle dynamics, flight control, and offboard mission execution.
- Gazebo for the Mars terrain environment and sensor simulation.
- ROS 2 Jazzy for topic transport, launch orchestration, metrics, and visualization.
- The Space demos Mars terrain/map assets included in this repository, especially the Martian terrain models under `models/martian_surface` and the Gazebo world `worlds/mars_px4.sdf`.
- The custom PX4/Gazebo model `x500_ingenuity_mars`, which includes the downward monocular camera and downward rangefinder.
- OpenVINS as the MSCKF/VIO estimation framework.
- NASA/JPL public reporting on Ingenuity Flight 72 and the relationship between feature-poor terrain, visual navigation failure, and high lateral landing velocity.

## Conclusions

The project began with a working PX4/Gazebo Mars drone simulation and developed it into an RVIO evaluation testbed. The most important outcome was not just publishing `/rvio/odometry`; it was building the full experimental loop:

- Fly a controlled mission over non-planar Mars terrain.
- Start estimation at a meaningful mission phase.
- Use the downward camera, IMU, and 1D lidar as estimator inputs.
- Compare local RVIO motion against local Gazebo ground truth.
- Watch lateral position and velocity errors live in `rqt_plot`.

The project also clarified what RVIO is and is not. Using a downward lidar as an altimeter is not enough, especially when the drone pitches and the terrain is not planar. The lidar must be treated as a one-dimensional range ray, and its value is most useful when it can be associated with a visual feature. That range-feature can then reduce monocular depth uncertainty and improve the lateral pose estimate.

The Python proof-of-concept was useful for debugging and for understanding failure modes, but it behaved too much like optical-flow VIO. The OpenVINS integration moved the project closer to the paper by using a real MSCKF-style estimator with covariance propagation, camera clones, IMU propagation, and feature reprojection updates. The range-feature extension adds the key RVIO behavior: using the downward lidar to initialize or constrain selected visual feature depths.

The final system is best understood as a research prototype. It demonstrates the pipeline and provides live metrics for lateral localization, but it still depends on accurate camera/lidar alignment, good timing, sufficient trackable features, and stable OpenVINS initialization. The next most important improvement would be to add a deliberate OpenVINS initialization phase after climb and before survey motion, then begin metrics only after the first valid RVIO odometry sample. That would make the final x/y convergence test cleaner and better aligned with the scientific question: whether range-constrained visual features can provide robust lateral position and velocity estimates during landing over challenging Martian terrain.

## References

- NASA, "NASA Performs First Aircraft Accident Investigation on Another World": https://www.nasa.gov/missions/mars-2020-perseverance/ingenuity-helicopter/nasa-performs-first-aircraft-accident-investigation-on-another-world/
- NASA/JPL, "Ingenuity's Hard Landing": https://www.jpl.nasa.gov/images/pia26482-ingenuitys-hard-landing/
- Local project paper: `Final_SES_RVIO.pdf`
- Local OpenVINS RVIO config: `config/openvins_mars_estimator.yaml`
- Local mission launch: `launch/cylinder_landing.launch.py`
- Local evaluator: `terrain_mapping_drone_control/rvio_evaluator_node.py`
- Local OpenVINS delayed launcher: `terrain_mapping_drone_control/openvins_on_state_launcher.py`
