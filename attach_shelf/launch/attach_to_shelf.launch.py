from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
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
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config = [FindPackageShare("attach_shelf"), "/rviz/pre_approach.rviz"]

    pre_approach_task1 = Node(
        package="attach_shelf",
        executable="pre_approach",
        name="pre_approach",
        output="screen",
        parameters=[
            {
                "obstacle": ParameterValue(obstacle, value_type=float),
                "degrees": ParameterValue(degrees, value_type=float),
                "forward_speed": ParameterValue(forward_speed, value_type=float),
                "angular_speed": ParameterValue(angular_speed, value_type=float),
                "rotation_scale": ParameterValue(rotation_scale, value_type=float),
            }
        ],
        condition=UnlessCondition(final_approach),
    )

    pre_approach_v2 = Node(
        package="attach_shelf",
        executable="pre_approach_v2",
        name="pre_approach_v2",
        output="screen",
        parameters=[
            {
                "obstacle": ParameterValue(obstacle, value_type=float),
                "degrees": ParameterValue(degrees, value_type=float),
                "final_approach": ParameterValue(final_approach, value_type=bool),
                "forward_speed": ParameterValue(forward_speed, value_type=float),
                "angular_speed": ParameterValue(angular_speed, value_type=float),
                "rotation_scale": ParameterValue(rotation_scale, value_type=float),
            }
        ],
        condition=IfCondition(final_approach),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("obstacle", default_value="0.4"),
            DeclareLaunchArgument("degrees", default_value="-90.0"),
            DeclareLaunchArgument("final_approach", default_value="false"),
            DeclareLaunchArgument("forward_speed", default_value="0.4"),
            DeclareLaunchArgument("angular_speed", default_value="0.5"),
            DeclareLaunchArgument("rotation_scale", default_value="0.5"),
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
                executable="approach_service_server",
                name="approach_service_server",
                output="screen",
                condition=IfCondition(final_approach),
            ),
            pre_approach_task1,
            pre_approach_v2,
            RegisterEventHandler(
                OnProcessExit(target_action=pre_approach_v2, on_exit=[Shutdown()])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=pre_approach_task1, on_exit=[Shutdown()])
            ),
        ]
    )
