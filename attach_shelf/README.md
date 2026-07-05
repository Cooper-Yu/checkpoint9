# attach_shelf

ROS 2 package for the shelf pre-approach and attach-to-shelf flows in Robot Developer Masterclass.

## Overview

The `pre_approach` node:

- subscribes to `/scan`;
- extracts the closest valid distance in a front LaserScan window;
- drives forward until the robot reaches the configured `obstacle` distance;
- stops briefly, rotates by the configured `degrees`, then stops;
- publishes velocity commands on `/cmd_vel`.

The Task 2 flow adds:

- `pre_approach_v2`, which runs the same pre-approach state machine and then calls `/approach_shelf`;
- `approach_service_server`, which detects reflective shelf legs from `/scan`, publishes `cart_frame`, and optionally drives under the shelf;
- custom service `attach_shelf/srv/GoToLoading` with request field `attach_to_shelf` and response field `complete`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select attach_shelf
source install/setup.bash
```

## Launch

```bash
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.4 degrees:=-90
```

This launch file starts both `pre_approach` and RViz with the package RViz config.
For headless testing, disable RViz:

```bash
ros2 launch attach_shelf pre_approach.launch.xml use_rviz:=false obstacle:=0.4 degrees:=-90
```

Task 2 launch:

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=false
```

Run the full attach-to-shelf flow:

```bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=true
```

Use `final_approach:=true` to start `approach_service_server`; `pre_approach_v2` then calls `/approach_shelf`, drives under the shelf, and publishes `/elevator_up`.
The package defaults include the validated TF rotation and final-approach tuning.

## Parameters

- `obstacle`: target stopping distance in meters.
- `degrees`: rotation angle after stopping. Negative values rotate clockwise.
- `forward_speed`: forward velocity in meters per second. Default: `0.4`.
- `angular_speed`: rotation velocity in radians per second. Default: `0.5`.
- `rotation_scale`: multiplier applied to the open-loop rotation time when TF rotation is disabled. Default: `0.5`.
- `final_approach`: Task 2 boolean. When `false`, launch runs only the Task 1 pre-approach. When `true`, `pre_approach_v2` calls `/approach_shelf` after the pre-approach rotation.
- `use_tf_rotation`: whether pre-approach rotation uses TF yaw feedback. Default: `true`.
- `use_rviz`: whether to start RViz. Default: `true`.

## Interfaces

- Subscribed topics: `/scan`
- Published topics: `/cmd_vel`, `/elevator_up`
- Published TF: `cart_frame`
- Service: `/approach_shelf` using `attach_shelf/srv/GoToLoading`
