# Checkpoint 10: Node Composition

This package contains the composable-node version of the Checkpoint 9 shelf
attachment project.

## Components

- `my_components::PreApproach` drives the robot in front of the shelf and turns
  it to face the shelf.
- `my_components::AttachServer` starts the `/approach_shelf` service.
- `my_components::AttachClient` calls `/approach_shelf` with
  `attach_to_shelf=true` when it is loaded into the running container.

The service interface is reused from the `attach_shelf` package:
`attach_shelf/srv/GoToLoading.srv`.

## Build on The Construct

Clone this repository into `~/ros2_ws/src`, then build both the original
Checkpoint 9 package and this component package:

```bash
cd ~/ros2_ws
colcon build --packages-select attach_shelf my_components
source install/setup.bash
```

Verify that the components are registered:

```bash
ros2 component types | grep -A3 my_components
```

Expected components:

```text
my_components
  my_components::PreApproach
  my_components::AttachServer
  my_components::AttachClient
```

## Task 1 Check

Start a component container:

```bash
ros2 run rclcpp_components component_container
```

In another terminal, load the pre-approach component:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 component load /ComponentManager my_components my_components::PreApproach
```

The robot should drive toward the shelf area, stop, and rotate to face the
shelf.

## Task 2 Check

Launch the container with `PreApproach` and `AttachServer` already loaded:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_components attach_to_shelf.launch.py
```

Verify that the service is ready:

```bash
ros2 service list | grep approach_shelf
```

Expected output:

```text
/approach_shelf
```

Load the runtime client component:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 component load /my_container my_components my_components::AttachClient
```

The client calls `/approach_shelf`, the robot performs the final approach, and
the shelf is lifted when the service completes.
