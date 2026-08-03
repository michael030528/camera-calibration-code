from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='camera_mid360_bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                            '--rate', LaunchConfiguration('rate'), '--clock'], output='screen'),
    ])
