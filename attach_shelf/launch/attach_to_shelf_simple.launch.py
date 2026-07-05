from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    obstacle = LaunchConfiguration("obstacle")
    degrees = LaunchConfiguration("degrees")
    final_approach = LaunchConfiguration("final_approach")
    forward_speed = LaunchConfiguration("forward_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    rotation_scale = LaunchConfiguration("rotation_scale")
    service_forward_speed = LaunchConfiguration("service_forward_speed")
    service_rotate_speed = LaunchConfiguration("service_rotate_speed")
    conservative_offset = LaunchConfiguration("conservative_offset")
    final_drive_distance = LaunchConfiguration("final_drive_distance")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config = [FindPackageShare("attach_shelf"), "/rviz/pre_approach.rviz"]

    pre_approach_params = [
        {
            "obstacle": ParameterValue(obstacle, value_type=float),
            "degrees": ParameterValue(degrees, value_type=float),
            "forward_speed": ParameterValue(forward_speed, value_type=float),
            "angular_speed": ParameterValue(angular_speed, value_type=float),
            "rotation_scale": ParameterValue(rotation_scale, value_type=float),
        }
    ]

    pre_approach_only = Node(
        package="attach_shelf",
        executable="pre_approach_simple",
        name="pre_approach_simple",
        output="screen",
        parameters=pre_approach_params,
        condition=UnlessCondition(final_approach),
    )

    pre_approach_before_attach = Node(
        package="attach_shelf",
        executable="pre_approach_simple",
        name="pre_approach_simple",
        output="screen",
        parameters=pre_approach_params,
        condition=IfCondition(final_approach),
    )

    approach_service_call = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/approach_shelf",
            "attach_shelf/srv/GoToLoading",
            "{attach_to_shelf: true}",
        ],
        output="screen",
        condition=IfCondition(final_approach),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("obstacle", default_value="0.4"),
            DeclareLaunchArgument("degrees", default_value="-90.0"),
            DeclareLaunchArgument("final_approach", default_value="false"),
            DeclareLaunchArgument("forward_speed", default_value="0.2"),
            DeclareLaunchArgument("angular_speed", default_value="0.5"),
            DeclareLaunchArgument("rotation_scale", default_value="0.5"),
            DeclareLaunchArgument("service_forward_speed", default_value="0.2"),
            DeclareLaunchArgument("service_rotate_speed", default_value="0.3"),
            DeclareLaunchArgument("conservative_offset", default_value="0.15"),
            DeclareLaunchArgument("final_drive_distance", default_value="0.30"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="attach_shelf",
                executable="approach_service_server_simple",
                name="approach_service_server_simple",
                output="screen",
                parameters=[
                    {
                        "forward_speed": ParameterValue(service_forward_speed, value_type=float),
                        "rotate_speed": ParameterValue(service_rotate_speed, value_type=float),
                        "conservative_offset": ParameterValue(conservative_offset, value_type=float),
                        "final_drive_distance": ParameterValue(final_drive_distance, value_type=float),
                    }
                ],
                condition=IfCondition(final_approach),
            ),
            pre_approach_only,
            pre_approach_before_attach,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=pre_approach_before_attach,
                    on_exit=[approach_service_call],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=approach_service_call, on_exit=[Shutdown()])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=pre_approach_only, on_exit=[Shutdown()])
            ),
        ]
    )
