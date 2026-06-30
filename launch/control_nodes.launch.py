from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_scale = LaunchConfiguration("linear_scale")
    angular_scale = LaunchConfiguration("angular_scale")
    watchdog_timeout_sec = LaunchConfiguration("watchdog_timeout_sec")

    angle_rate = LaunchConfiguration("angle_rate")
    min_angle = LaunchConfiguration("min_angle")
    max_angle = LaunchConfiguration("max_angle")
    trajectory_duration = LaunchConfiguration("trajectory_duration")
    command_period = LaunchConfiguration("command_period")

    return LaunchDescription([
        DeclareLaunchArgument(
            "linear_scale",
            default_value="3.0"
        ),
        DeclareLaunchArgument(
            "angular_scale",
            default_value="8.0"
        ),
        DeclareLaunchArgument(
            "watchdog_timeout_sec",
            default_value="0.30"
        ),
        DeclareLaunchArgument(
            "angle_rate",
            default_value="1.0"
        ),
        DeclareLaunchArgument(
            "min_angle",
            default_value="-1.45"
        ),
        DeclareLaunchArgument(
            "max_angle",
            default_value="1.45"
        ),
        DeclareLaunchArgument(
            "trajectory_duration",
            default_value="0.25"
        ),
        DeclareLaunchArgument(
            "command_period",
            default_value="0.20"
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),

        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy",
            output="screen",
            parameters=[{
                "require_enable_button": False,
                "axis_linear.x": 1,
                "axis_angular.yaw": 3,
            }],
        ),

        Node(
            package="gazebo_control",
            executable="cmd_vel_track_control_node",
            name="cmd_vel_track_control_node",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "gazebo_cmd_vel_topic": "/gazebo/drok_gazebo/cmd_vel_twist",
                "watchdog_timeout_sec": watchdog_timeout_sec,
                "linear_scale": linear_scale,
                "angular_scale": angular_scale,
            }],
        ),

        Node(
            package="gazebo_control",
            executable="flipper_joy_control_node",
            name="flipper_joy_control_node",
            output="screen",
            parameters=[{
                "joy_topic": "/joy",
                "joint_state_topic": "/joint_states",
                "command_topic": "/flipper_position_controller/joint_trajectory",

                "front_axis": 7,
                "rear_axis": 6,
                "hold_button": 0,
                "reset_button": 1,

                "angle_rate": angle_rate,
                "min_angle": min_angle,
                "max_angle": max_angle,
                "deadzone": 0.2,
                "trajectory_duration": trajectory_duration,
                "command_period": command_period,

                "frf_sign": 1.0,
                "flf_sign": -1.0,
                "brf_sign": 1.0,
                "blf_sign": -1.0,
            }],
        ),
    ])
