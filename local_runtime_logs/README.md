# Local Runtime Logs

This folder is committed so The Construct can write temporary launch logs here.

Generated `*.log` files are ignored by Git.

## Capture Task 2 Full Approach

From `~/ros2_ws`:

```bash
source install/setup.bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=true use_rviz:=false 2>&1 | tee ~/ros2_ws/src/checkpoint9/local_runtime_logs/attach_to_shelf_latest.log
```

## Capture Pre-Approach Only

From `~/ros2_ws`:

```bash
source install/setup.bash
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.4 degrees:=-90 final_approach:=false use_rviz:=false 2>&1 | tee ~/ros2_ws/src/checkpoint9/local_runtime_logs/pre_approach_latest.log
```

## Print Useful Debug Lines

```bash
grep "pre_approach\|approach_service_server\|SAFE_STOP\|cart_frame\|complete=\|State transition\|Valid front scan\|Invalid front scan\|Received /approach_shelf\|Starting final approach\|Driving toward\|Final shelf push" ~/ros2_ws/src/checkpoint9/local_runtime_logs/attach_to_shelf_latest.log
```
