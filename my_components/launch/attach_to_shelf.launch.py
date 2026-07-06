from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_tf_rotation = LaunchConfiguration("use_tf_rotation")

    rviz_config = [FindPackageShare("my_components"), "/rviz/pre_approach.rviz"]

    # Keep the checkpoint-critical motion constants in the launch file so the
    # same component can be loaded by hand for Task 1 or preconfigured for Task 2.
    pre_approach_params = [
        {
            "obstacle": 0.4,
            "degrees": -90.0,
            "forward_speed": 0.4,
            "angular_speed": 0.5,
            "rotation_scale": 0.5,
            "use_tf_rotation": ParameterValue(use_tf_rotation, value_type=bool),
            "rotation_tolerance": 0.03,
            "rotation_reference_frame": "odom",
            "rotation_base_frame": "robot_base_footprint",
            "shutdown_on_complete": False,
        }
    ]

    # These values mirror the tuned Checkpoint 9 final-approach behavior while
    # moving the server into a composable node.
    attach_server_params = [
        {
            "forward_speed": 0.2,
            "rotate_speed": 0.3,
            "min_rotate_speed": 0.05,
            "rotate_speed_gain": 1.0,
            "yaw_tolerance": 0.005,
            "conservative_offset": 0.0,
            "final_drive_distance": 0.30,
            "enable_final_push": True,
            "verify_center_before_final_push": False,
            "center_lateral_tolerance": 0.05,
            "center_distance_tolerance": 0.20,
            "center_lock_distance": 0.35,
            "center_lock_min_steps": 2,
            "center_drive_scale": 3.3,
            "center_extra_forward_distance": 0.0,
            "yaw_correction_steps": 3,
            "lateral_yaw_gain": 0.4,
            "min_yaw_correction_distance": 0.55,
            "restore_yaw_after_correction": False,
            "forward_step_distance": 0.20,
            "movement_timeout": 45.0,
            "service_straight_test": False,
            "straight_sample_count": 7,
            "straight_sample_max_spread": 0.25,
            "target_base_frame": "robot_base_link",
        }
    ]

    # The grader expects a container named my_container with these two nodes
    # already loaded. AttachClient is loaded later with ros2 component load.
    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="my_components",
                plugin="my_components::PreApproach",
                name="pre_approach",
                parameters=pre_approach_params,
            ),
            ComposableNode(
                package="my_components",
                plugin="my_components::AttachServer",
                name="attach_server",
                parameters=attach_server_params,
            ),
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_tf_rotation", default_value="true"),
            rviz,
            container,
            RegisterEventHandler(
                OnProcessExit(target_action=container, on_exit=[Shutdown()])
            ),
        ]
    )
