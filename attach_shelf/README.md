# attach_shelf

ROS 2 package for the shelf pre-approach step in Robot Developer Masterclass.

## Overview

The `pre_approach` node:

- subscribes to `/scan`;
- extracts the closest valid distance in a front LaserScan window;
- drives forward until the robot reaches the configured `obstacle` distance;
- stops briefly, rotates by the configured `degrees`, then stops;
- publishes velocity commands on `/cmd_vel`.

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

## Parameters

- `obstacle`: target stopping distance in meters.
- `degrees`: rotation angle after stopping. Negative values rotate clockwise.
- `forward_speed`: forward velocity in meters per second. Default: `0.4`.
- `angular_speed`: rotation velocity in radians per second. Default: `0.5`.
- `rotation_scale`: multiplier applied to the open-loop rotation time. Default: `0.5`.
- `use_rviz`: whether to start RViz. Default: `true`.
