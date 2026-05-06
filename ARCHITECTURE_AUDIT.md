# Architecture Audit

Phase 1 deliverable for converting `ses598-final` from the Assignment 3
object-centric mission into a final project about safe terrain-relative landing
over rugged terrain.

Scope of this phase: inspect and classify the existing package. No estimator
code, controller refactor, or launch behavior changes are introduced here.

## Current Package Summary

The package is still organized around the original Assignment 3 goal: find,
measure, map, or land on cylinder/rover-like targets. The most runnable current
path, however, is already closer to a terrain-relative landing scaffold:

`launch/cylinder_landing.launch.py`

This launch file starts PX4 SITL with the custom `x500_depth_mono` model, spawns
two cylinders, bridges Gazebo sensor/odometry topics into ROS 2, and launches
`rangefinder_terrain_mission`.

The active mission node:

`terrain_mapping_drone_control/rangefinder_terrain_mission.py`

uses PX4 odometry plus a downward rangefinder to take off, fly a fixed waypoint
survey, return to launch, descend, land, disarm, and log duration/battery. It
does not currently depend on cylinder detection.

## File-by-File Classification

| File | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `README.md` | User-facing setup and original assignment description | Modify | Still describes the Assignment 3 object-centric mission; now partially updated with VM-to-macOS QGroundControl link | ROS 2, PX4, QGC, package launch files | Convert into final-project README after estimator/evaluation path exists |
| `package.xml` | ROS package metadata and dependency declaration | Modify lightly | Dependencies are mostly useful, but description is stale and ArUco dependencies may become optional | `rclpy`, `px4_msgs`, `sensor_msgs`, `ros_gz_bridge`, `rtabmap`, `rviz2`, `cv_bridge`, OpenCV | Update description and add any estimator/evaluation dependencies only when needed |
| `setup.py` | Python package install data and console entry points | Modify | Contains stale/missing entry point `px4_odom_converter`; installs old cylinder files; future estimator/logger nodes need entry points | setuptools, package data, launch/config/model files | Add new estimator/logger entries in later phases; remove stale entries only after baseline is protected |
| `setup.cfg` | Installs scripts into package lib directory | Keep | Standard for ROS 2 Python console scripts | setuptools | No immediate change |
| `CMakeLists.txt` | Legacy/basic model installation with `ament_cmake` | Modify or retire later | Package uses `ament_python` in `package.xml`; this file only installs models and may confuse build ownership | `ament_cmake`, model directory | Reconcile packaging after baseline docs and launch paths are stable |
| `resource/terrain_mapping_drone_control` | ROS ament package marker | Keep | Required package resource marker | ament index | No change |
| `LICENSE` | License | Keep | Project metadata | None | No change |
| `HW3.pdf` | Original assignment artifact | Retire from main docs later | Not part of final runtime, but useful historical context | None | Move to docs/archive only after final packaging |

## Launch Files

| File | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `launch/cylinder_landing.launch.py` | Current main PX4/Gazebo/ROS launch path | Keep and modify | Best scaffold for final project: launches PX4, custom model, sensors, rangefinder, odometry bridge, and current mission | PX4-Autopilot, `ros_gz_sim`, `ros_gz_bridge`, `rangefinder_terrain_mission`, QGC host IP | Rename or parameterize later as `safe_landing_eval.launch.py`; keep old name runnable |
| `launch/visualization.launch.py` | Starts RViz and `pose_visualizer` | Keep mostly as-is | Reusable for final project visualization | `rviz2`, `config/drone_viz.rviz`, `/fmu/out/vehicle_odometry` | Add estimator visualization topics later |
| `launch/mission.launch.py` | Runs `aruco_tracker.py` and `auto_detect_land.py` directly | Retire from main path, preserve initially | Object-centric and bypasses normal ROS entry points; assumes bridge already exists | Python files, `/drone/down_mono`, `/aruco/marker_pose`, front RGB/depth topics | Mark as legacy object mission after Phase 2 |
| `launch/terrain_mapping.launch.py` | Older terrain/cylinder/gimbal launch path | Retire or archive later | Uses `gz_x500_gimbal`, older topic names, and malformed bridge type syntax in several strings | PX4 gimbal model, terrain/cylinder model, `ros_gz_bridge` | Reuse ideas only if gimbal trajectory is needed; do not use as final main path |
| `launch/rtabmap.launch.py` | RTAB-Map mapping launch | Modify or keep optional | Useful for mapping/evaluation, but topic remaps do not match current camera topics exactly | `rtabmap_ros`, static TF, camera/depth/odom topics | Make optional evaluation mapping launch after estimator interface is defined |

## Config Files

| File | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `config/drone_viz.rviz` | RViz config for drone pose markers and pose covariance | Keep and modify later | Reusable visualization baseline | `/drone/visualization_marker_array`, `/drone/pose_with_covariance` | Add terrain-relative estimator displays later |
| `config/Default.perspective` | GUI perspective for camera/debug image panels | Keep optional | Useful for inspecting front/down cameras and ArUco debug streams | `/drone/front_rgb`, `/drone/down_mono`, `/aruco/debug_image`, `/drone/front_depth` | Add estimator/debug panels if useful |
| `config/terrain_mapping_params.yaml` | Old terrain mapping and spiral parameters | Modify | Contains useful parameter names but is not wired into current launch; values reflect old assignment | mission/trajectory parameters | Replace or split into final mission/evaluation parameter files |

## Scripts

| File | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `scripts/deploy_px4_model.sh` | Copies custom PX4 airframe/model files into PX4-Autopilot | Keep | Required for custom `x500_depth_mono` SITL model | local package models, PX4-Autopilot tree | Update docs and verify copied airframe/model names |
| `scripts/setup_px4_model.sh` | Copies PX4 model files into package for development | Keep optional | Useful for refreshing model baseline from PX4 | PX4-Autopilot tree | Use carefully; avoid overwriting final custom model unintentionally |
| `scripts/generate_aruco.py` | Generates ArUco marker texture for cylinder | Retire from main path | Target-specific to cylinder landing | OpenCV ArUco, cylinder material path | Keep in legacy tools until object mission is archived |
| `scripts/generate_cylinder_texture.py` | Generates feature-rich cylinder texture | Retire from main path | Target-specific, but texture idea may inform terrain texture stress tests | OpenCV, NumPy | Replace with terrain texture/scenario generation in Phase 9 |
| `scripts/spiral_trajectory` | Standalone older spiral trajectory script | Retire or consolidate | Duplicates `terrain_mapping_drone_control/spiral_trajectory.py` with different parameters | PX4 topics, `rclpy` | Keep only package node version after baseline is protected |

## Python Nodes

| File | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `terrain_mapping_drone_control/rangefinder_terrain_mission.py` | Current active terrain-relative survey/descent/landing mission | Modify heavily, but preserve baseline | Best starting point for final safe-landing mission because it already uses rangefinder AGL and avoids target detection | `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`, `/fmu/out/battery_status`, `/drone/down_rangefinder`, PX4 command topics | In Phase 2, split into generic controlled descent mission while keeping baseline launchable |
| `terrain_mapping_drone_control/pose_visualizer.py` | Converts PX4 odometry to RViz markers and `PoseWithCovarianceStamped` | Keep mostly as-is | Generic and reusable for baseline and estimator comparison | `/fmu/out/vehicle_odometry`, `/drone/visualization_marker_array`, `/drone/pose_with_covariance` | Add estimator overlay or separate estimator visualizer later |
| `terrain_mapping_drone_control/feature_tracker.py` | ORB feature tracking from a camera image | Modify | Generic visual-feature foundation, but subscribes to older `/drone_camera` topic | `/drone_camera`, OpenCV ORB, `cv_bridge` | Retarget to `/drone/front_rgb` or make topic parameterized in estimator sandbox phase |
| `terrain_mapping_drone_control/spiral_trajectory.py` | PX4 offboard spiral descent with gimbal pointing logic | Modify | Reusable as controlled descent/evaluation trajectory, but tied to `x500_gimbal` gimbal topics | PX4 command/odom/status topics, `/model/x500_gimbal_0/command/gimbal_*` | Convert into parameterized descent-test trajectory or retire in favor of mission node |
| `terrain_mapping_drone_control/aruco_tracker.py` | Detects top-mounted ArUco markers from downward mono camera | Retire from main path, preserve legacy | Cylinder-top landing target detector; not appropriate as final landing success condition | `/drone/down_mono`, `/drone/down_mono/camera_info`, `/aruco/debug_image`, `/aruco/marker_pose`, TF | Keep as optional visual-debug/legacy object detector |
| `terrain_mapping_drone_control/auto_detect_land.py` | Object-centric cylinder mission with RGB/depth detection and ArUco landing | Retire from main path | Encodes original target-based mission: detect cylinder, measure, choose marker, land | `/drone/front_rgb`, `/drone/front_depth`, `/drone/front_depth/camera_info`, `/aruco/marker_pose`, PX4 command/odom/battery topics | Mine for state-machine patterns only; replace with estimator-centric landing logic later |
| `terrain_mapping_drone_control/cylinder_landing_node.py` | Simple takeoff/land test node with geometry tracker subscriptions | Retire or convert to smoke test | Name and subscriptions are cylinder-specific; behavior is just takeoff then land | PX4 command/odom/status topics, `/geometry/cylinder_center`, `/geometry/cylinder_info` | Keep as minimal offboard smoke test only if renamed/parameterized |
| `terrain_mapping_drone_control/geometry_tracker.py` | Depth-image cylinder-like contour/ellipse detector | Retire from main path | Target-specific geometry detector | `/drone/front_depth`, `/geometry/debug_image`, `/geometry/cylinder_center`, `/geometry/cylinder_info` | Replace with terrain-feature/roughness/debug extraction only if useful |
| `terrain_mapping_drone_control/__init__.py` | Python package marker | Keep | Required package file | Python package import system | No change |

## Models and Simulation Assets

| File / Directory | Role | Keep / Modify / Retire | Reason | Dependencies | Replacement Plan if Modified |
| --- | --- | --- | --- | --- | --- |
| `models/px4_models/gz_models/x500_depth_mono/` | Custom PX4 Gazebo drone model with front RGB/depth, downward mono, rangefinder | Keep and modify carefully | Core final-project platform | PX4 Gazebo model include `x500`, `OakD-Lite-Modify`, Gazebo sensors | Add IMU/topic exposure only if needed; preserve current sensor topics |
| `models/px4_models/airframes/4022_gz_x500_depth_mono` | PX4 airframe for custom model | Keep | Needed to launch `gz_x500_depth_mono` | PX4-Autopilot airframe registry | Verify deploy script and PX4 build path |
| `models/OakD-Lite-Modify/` | Front RGB/depth camera model | Keep | Useful for visual/range/depth inputs | Gazebo camera and depth camera sensors | Keep camera topics stable for estimator sandbox |
| `models/terrain/` | Rugged terrain mesh/model | Keep and modify | Directly relevant to rugged-terrain landing final project | Gazebo model mesh/material files | Promote from auxiliary asset to main test environment |
| `models/cylinder/` | 10 m cylinder with ArUco marker 0 | Retire from main path, preserve initially | Original object target | Gazebo cylinder, texture, ArUco material | Move to legacy object scenario after safe landing eval works |
| `models/cylinder_short/` | 7 m cylinder with ArUco marker 1 | Retire from main path, preserve initially | Original object target | Gazebo cylinder, texture, ArUco material | Move to legacy object scenario after safe landing eval works |

## Current Node and Topic Flow

### Main Runnable Flow

```text
cylinder_landing.launch.py
  -> PX4 SITL: make px4_sitl gz_x500_depth_mono
       env PX4_GZ_MODEL=x500_depth_mono
       env PX4_SIM_HOST_ADDR=<macOS QGroundControl IP>
       env PX4_SIM_UDP_PORT=14550
  -> Gazebo creates:
       cylinder_front at x=5
       cylinder_back at x=-5
  -> ros_gz_bridge parameter_bridge
       /rgb_camera                    -> /drone/front_rgb
       /rgb_camera/camera_info        -> /drone/front_rgb/camera_info
       /depth_camera                  -> /drone/front_depth
       /depth_camera/points           -> /drone/front_depth/points
       /camera_info                   -> /drone/front_depth/camera_info
       /mono_camera                   -> /drone/down_mono
       /mono_camera/camera_info       -> /drone/down_mono/camera_info
       /rangefinder                   -> /drone/down_rangefinder
       /clock                         -> /clock
       /model/x500_depth_mono_0/odometry_with_covariance
                                      -> /fmu/out/vehicle_odometry
  -> rangefinder_terrain_mission
       subscribes:
         /fmu/out/vehicle_odometry
         /fmu/out/vehicle_status
         /fmu/out/battery_status
         /drone/down_rangefinder
       publishes:
         /fmu/in/offboard_control_mode
         /fmu/in/trajectory_setpoint
         /fmu/in/vehicle_command
         /mission/agl_estimate
         /mission/ground_z_ned
```

### Current Mission State Machine

```text
WAIT_RANGE
  -> ARM_TAKEOFF
  -> SURVEY
  -> RETURN
  -> DESCEND
  -> LAND
  -> COMPLETE
  -> DONE
```

This is a useful baseline for the final project because it already has:

- terrain-relative AGL from a downward rangefinder,
- filtered ground height estimate in NED coordinates,
- PX4 offboard control commands,
- descent and landing stages,
- mission duration and battery logging.

Current limitations for the final project:

- `survey_speed` is declared but not used for velocity shaping.
- Waypoints are hard-coded unless passed as ROS parameters.
- Landing still uses simple range threshold logic rather than estimator health.
- There is no logged comparison against ground truth or an improved estimator.
- The rugged terrain model is not spawned in the current main launch.

### Legacy Object-Centric Flow

```text
mission.launch.py
  -> aruco_tracker.py
       subscribes:
         /drone/down_mono
         /drone/down_mono/camera_info
       publishes:
         /aruco/debug_image
         /aruco/marker_pose
       broadcasts TF:
         camera_frame -> aruco_marker_<id>

  -> auto_detect_land.py
       subscribes:
         /drone/front_rgb
         /drone/front_depth
         /drone/front_depth/camera_info
         /aruco/marker_pose
         /fmu/out/vehicle_odometry
         /fmu/out/battery_status
       publishes:
         /fmu/in/offboard_control_mode
         /fmu/in/trajectory_setpoint
         /fmu/in/vehicle_command
```

This flow is tightly coupled to cylinder detection and marker-based landing.
It should not be used as the final-project main mission, but it can remain as
a legacy scenario until the new mission path is verified.

### Visualization Flow

```text
visualization.launch.py
  -> pose_visualizer
       subscribes:
         /fmu/out/vehicle_odometry
       publishes:
         /drone/visualization_marker_array
         /drone/pose_with_covariance
  -> rviz2 with config/drone_viz.rviz
```

This flow is reusable and should later display both baseline and proposed
terrain-relative estimator outputs.

### Optional Mapping Flow

```text
rtabmap.launch.py
  -> static_transform_publisher base_link -> camera_link
  -> rtabmap
       expects:
         /camera/rgb/image_raw
         /camera/depth/image_raw
         /camera/rgb/camera_info
         /fmu/out/vehicle_odometry
       publishes:
         map, mapData, mapPath, cloud_map
  -> point_cloud_xyz
```

This is not aligned with the current `/drone/front_*` topic names and should be
treated as optional until remapped correctly.

## Dependency Map

```text
PX4-Autopilot
  -> custom airframe 4022_gz_x500_depth_mono
  -> custom Gazebo model x500_depth_mono
  -> MAVLink UDP to QGroundControl on macOS host
  -> PX4 uORB/ROS 2 bridge topics under /fmu/*

Gazebo / ros_gz
  -> x500_depth_mono model sensors
  -> OakD-Lite RGB/depth topics
  -> downward mono camera topic
  -> downward rangefinder topic
  -> model odometry_with_covariance
  -> ros_gz_bridge maps Gazebo topics to ROS 2 topics

Mission Nodes
  -> PX4 command inputs:
       /fmu/in/offboard_control_mode
       /fmu/in/trajectory_setpoint
       /fmu/in/vehicle_command
  -> PX4 state outputs:
       /fmu/out/vehicle_odometry
       /fmu/out/vehicle_status
       /fmu/out/battery_status
  -> Sensor inputs:
       /drone/down_rangefinder
       /drone/front_rgb
       /drone/front_depth
       /drone/down_mono

Visualization / Debug
  -> pose_visualizer consumes PX4 odometry
  -> RViz consumes marker and pose topics
  -> ArUco/geometry nodes publish debug image and target topics

Future Final-Project Estimator Insertion Point
  sensor topics + PX4 priors
    -> terrain_relative_estimator in shadow mode
    -> estimator debug/health topics
    -> logger/evaluator
    -> later, gated landing-decision logic
```

## Target-Specific vs Reusable Components

### Target-Specific Components

These should be demoted from the final-project main path:

- `terrain_mapping_drone_control/auto_detect_land.py`
- `terrain_mapping_drone_control/aruco_tracker.py`
- `terrain_mapping_drone_control/geometry_tracker.py`
- `terrain_mapping_drone_control/cylinder_landing_node.py`
- `launch/mission.launch.py`
- `models/cylinder/`
- `models/cylinder_short/`
- `scripts/generate_aruco.py`
- `scripts/generate_cylinder_texture.py`

### Reusable Components

These are directly useful for the final project:

- `launch/cylinder_landing.launch.py`
- `launch/visualization.launch.py`
- `terrain_mapping_drone_control/rangefinder_terrain_mission.py`
- `terrain_mapping_drone_control/pose_visualizer.py`
- `terrain_mapping_drone_control/feature_tracker.py`, after topic parameterization
- `terrain_mapping_drone_control/spiral_trajectory.py`, after removing gimbal-specific assumptions
- `models/px4_models/gz_models/x500_depth_mono/`
- `models/px4_models/airframes/4022_gz_x500_depth_mono`
- `models/OakD-Lite-Modify/`
- `models/terrain/`
- `scripts/deploy_px4_model.sh`
- `config/drone_viz.rviz`

### Components Needing Cleanup

- `setup.py` references `terrain_mapping_drone_control.px4_odom_converter:main`,
  but no `px4_odom_converter.py` file exists in the package.
- `setup.py` installs `models/cylinder_small`, but the repository contains
  `models/cylinder_short`.
- Source-tree `__pycache__` files are present under `launch/` and
  `terrain_mapping_drone_control/`; these should not be part of final packaging.
- `terrain_mapping.launch.py` appears stale and should not be treated as the
  canonical launch path without repair.

## Recommendations for Phase 2

1. Preserve `launch/cylinder_landing.launch.py` as the known runnable baseline.
2. Add a new launch option or new launch file for a final-project mode, such as
   `safe_landing_eval`, while keeping the old launch path runnable.
3. Use `rangefinder_terrain_mission.py` as the starting point for the simplified
   descent mission because it already avoids cylinder/rover success conditions.
4. Spawn the rugged `models/terrain` asset in the evaluation launch path so the
   mission actually exercises terrain-relative descent.
5. Keep object-centric nodes available but out of the default path.
6. Do not connect any new estimator to landing logic until a shadow-mode
   estimator is publishing and logging sensible outputs.

## Answer to Phase 1 Exit Question

The reusable parts of the assignment are the PX4 SITL launch plumbing, the custom
sensor-equipped drone model, the Gazebo-to-ROS bridge, PX4 offboard command
publishers/subscribers, downward rangefinder descent scaffold, visualization
node, and parts of the camera feature-tracking/debug infrastructure.

The parts that should be modified heavily are the mission and landing nodes,
because the final project should make landing decisions from descent state,
terrain-relative estimates, and estimator health rather than object detection.

The parts that should be retired from the main path are cylinder/rover target
detection, ArUco marker landing, cylinder dimension measurement, and cylinder
texture/marker generation.

