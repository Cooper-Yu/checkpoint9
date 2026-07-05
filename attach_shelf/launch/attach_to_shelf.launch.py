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
    use_tf_rotation = LaunchConfiguration("use_tf_rotation")
    rotation_tolerance = LaunchConfiguration("rotation_tolerance")
    rotation_reference_frame = LaunchConfiguration("rotation_reference_frame")
    rotation_base_frame = LaunchConfiguration("rotation_base_frame")
    service_forward_speed = LaunchConfiguration("service_forward_speed")
    service_rotate_speed = LaunchConfiguration("service_rotate_speed")
    service_min_rotate_speed = LaunchConfiguration("service_min_rotate_speed")
    service_rotate_speed_gain = LaunchConfiguration("service_rotate_speed_gain")
    service_yaw_tolerance = LaunchConfiguration("service_yaw_tolerance")
    conservative_offset = LaunchConfiguration("conservative_offset")
    final_drive_distance = LaunchConfiguration("final_drive_distance")
    enable_final_push = LaunchConfiguration("enable_final_push")
    verify_center_before_final_push = LaunchConfiguration("verify_center_before_final_push")
    center_lateral_tolerance = LaunchConfiguration("center_lateral_tolerance")
    center_distance_tolerance = LaunchConfiguration("center_distance_tolerance")
    center_lock_distance = LaunchConfiguration("center_lock_distance")
    center_lock_min_steps = LaunchConfiguration("center_lock_min_steps")
    center_drive_scale = LaunchConfiguration("center_drive_scale")
    center_extra_forward_distance = LaunchConfiguration("center_extra_forward_distance")
    yaw_correction_steps = LaunchConfiguration("yaw_correction_steps")
    service_lateral_yaw_gain = LaunchConfiguration("service_lateral_yaw_gain")
    min_yaw_correction_distance = LaunchConfiguration("min_yaw_correction_distance")
    restore_yaw_after_correction = LaunchConfiguration("restore_yaw_after_correction")
    forward_step_distance = LaunchConfiguration("forward_step_distance")
    movement_timeout = LaunchConfiguration("movement_timeout")
    service_straight_test = LaunchConfiguration("service_straight_test")
    straight_sample_count = LaunchConfiguration("straight_sample_count")
    straight_sample_max_spread = LaunchConfiguration("straight_sample_max_spread")
    target_base_frame = LaunchConfiguration("target_base_frame")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config = [FindPackageShare("attach_shelf"), "/rviz/pre_approach.rviz"]

    pre_approach_params = [
        {
            "obstacle": ParameterValue(obstacle, value_type=float),
            "degrees": ParameterValue(degrees, value_type=float),
            "forward_speed": ParameterValue(forward_speed, value_type=float),
            "angular_speed": ParameterValue(angular_speed, value_type=float),
            "rotation_scale": ParameterValue(rotation_scale, value_type=float),
            "use_tf_rotation": ParameterValue(use_tf_rotation, value_type=bool),
            "rotation_tolerance": ParameterValue(rotation_tolerance, value_type=float),
            "rotation_reference_frame": rotation_reference_frame,
            "rotation_base_frame": rotation_base_frame,
        }
    ]

    pre_approach_v2_params = [
        {
            "obstacle": ParameterValue(obstacle, value_type=float),
            "degrees": ParameterValue(degrees, value_type=float),
            "forward_speed": ParameterValue(forward_speed, value_type=float),
            "angular_speed": ParameterValue(angular_speed, value_type=float),
            "rotation_scale": ParameterValue(rotation_scale, value_type=float),
            "use_tf_rotation": ParameterValue(use_tf_rotation, value_type=bool),
            "rotation_tolerance": ParameterValue(rotation_tolerance, value_type=float),
            "rotation_reference_frame": rotation_reference_frame,
            "rotation_base_frame": rotation_base_frame,
            "final_approach": ParameterValue(final_approach, value_type=bool),
        }
    ]

    pre_approach_only = Node(
        package="attach_shelf",
        executable="pre_approach",
        name="pre_approach",
        output="screen",
        parameters=pre_approach_params,
        condition=UnlessCondition(final_approach),
    )

    pre_approach_before_attach = Node(
        package="attach_shelf",
        executable="pre_approach_v2",
        name="pre_approach_v2",
        output="screen",
        parameters=pre_approach_v2_params,
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
            DeclareLaunchArgument("use_tf_rotation", default_value="true"),
            DeclareLaunchArgument("rotation_tolerance", default_value="0.03"),
            DeclareLaunchArgument("rotation_reference_frame", default_value="odom"),
            DeclareLaunchArgument("rotation_base_frame", default_value="robot_base_footprint"),
            DeclareLaunchArgument("service_forward_speed", default_value="0.2"),
            DeclareLaunchArgument("service_rotate_speed", default_value="0.3"),
            DeclareLaunchArgument("service_min_rotate_speed", default_value="0.05"),
            DeclareLaunchArgument("service_rotate_speed_gain", default_value="1.0"),
            DeclareLaunchArgument("service_yaw_tolerance", default_value="0.005"),
            DeclareLaunchArgument("conservative_offset", default_value="0.0"),
            DeclareLaunchArgument("final_drive_distance", default_value="0.30"),
            DeclareLaunchArgument("enable_final_push", default_value="true"),
            DeclareLaunchArgument("verify_center_before_final_push", default_value="false"),
            DeclareLaunchArgument("center_lateral_tolerance", default_value="0.05"),
            DeclareLaunchArgument("center_distance_tolerance", default_value="0.20"),
            DeclareLaunchArgument("center_lock_distance", default_value="0.35"),
            DeclareLaunchArgument("center_lock_min_steps", default_value="2"),
            DeclareLaunchArgument("center_drive_scale", default_value="3.3"),
            DeclareLaunchArgument("center_extra_forward_distance", default_value="0.0"),
            DeclareLaunchArgument("yaw_correction_steps", default_value="3"),
            DeclareLaunchArgument("service_lateral_yaw_gain", default_value="0.4"),
            DeclareLaunchArgument("min_yaw_correction_distance", default_value="0.55"),
            DeclareLaunchArgument("restore_yaw_after_correction", default_value="false"),
            DeclareLaunchArgument("forward_step_distance", default_value="0.20"),
            DeclareLaunchArgument("movement_timeout", default_value="45.0"),
            DeclareLaunchArgument("service_straight_test", default_value="false"),
            DeclareLaunchArgument("straight_sample_count", default_value="5"),
            DeclareLaunchArgument("straight_sample_max_spread", default_value="0.25"),
            DeclareLaunchArgument("target_base_frame", default_value="robot_base_link"),
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
                parameters=[
                    {
                        "forward_speed": ParameterValue(service_forward_speed, value_type=float),
                        "rotate_speed": ParameterValue(service_rotate_speed, value_type=float),
                        "min_rotate_speed": ParameterValue(service_min_rotate_speed, value_type=float),
                        "rotate_speed_gain": ParameterValue(service_rotate_speed_gain, value_type=float),
                        "yaw_tolerance": ParameterValue(service_yaw_tolerance, value_type=float),
                        "conservative_offset": ParameterValue(conservative_offset, value_type=float),
                        "final_drive_distance": ParameterValue(final_drive_distance, value_type=float),
                        "enable_final_push": ParameterValue(enable_final_push, value_type=bool),
                        "verify_center_before_final_push": ParameterValue(
                            verify_center_before_final_push, value_type=bool
                        ),
                        "center_lateral_tolerance": ParameterValue(
                            center_lateral_tolerance, value_type=float
                        ),
                        "center_distance_tolerance": ParameterValue(center_distance_tolerance, value_type=float),
                        "center_lock_distance": ParameterValue(center_lock_distance, value_type=float),
                        "center_lock_min_steps": ParameterValue(center_lock_min_steps, value_type=int),
                        "center_drive_scale": ParameterValue(center_drive_scale, value_type=float),
                        "center_extra_forward_distance": ParameterValue(
                            center_extra_forward_distance, value_type=float
                        ),
                        "yaw_correction_steps": ParameterValue(yaw_correction_steps, value_type=int),
                        "lateral_yaw_gain": ParameterValue(service_lateral_yaw_gain, value_type=float),
                        "min_yaw_correction_distance": ParameterValue(
                            min_yaw_correction_distance, value_type=float
                        ),
                        "restore_yaw_after_correction": ParameterValue(
                            restore_yaw_after_correction, value_type=bool
                        ),
                        "forward_step_distance": ParameterValue(forward_step_distance, value_type=float),
                        "movement_timeout": ParameterValue(movement_timeout, value_type=float),
                        "service_straight_test": ParameterValue(service_straight_test, value_type=bool),
                        "straight_sample_count": ParameterValue(straight_sample_count, value_type=int),
                        "straight_sample_max_spread": ParameterValue(straight_sample_max_spread, value_type=float),
                        "target_base_frame": target_base_frame,
                    }
                ],
                condition=IfCondition(final_approach),
            ),
            pre_approach_only,
            pre_approach_before_attach,
            RegisterEventHandler(
                OnProcessExit(target_action=pre_approach_only, on_exit=[Shutdown()])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=pre_approach_before_attach, on_exit=[Shutdown()])
            ),
        ]
    )
