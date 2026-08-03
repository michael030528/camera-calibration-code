from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.152'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.41'),
        DeclareLaunchArgument('bag', default_value='camera_mid360_bag'),
        DeclareLaunchArgument('sync_tolerance_ms', default_value='60.0'),
        Node(package='camera_lidar_chessboard_calibration', executable='livox_direct', output='screen',
             parameters=[{'lidar_ip': LaunchConfiguration('lidar_ip'), 'host_ip': LaunchConfiguration('host_ip')}]),
        Node(package='camera_lidar_chessboard_calibration', executable='camera_rtsp', output='screen',
             parameters=[{'password': LaunchConfiguration('password')}]),
        Node(package='camera_lidar_chessboard_calibration', executable='sync_sensors', output='screen',
             parameters=[{'sync_tolerance_ms': LaunchConfiguration('sync_tolerance_ms')}]),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag'),
                            '/aligned/camera/image_raw', '/aligned/livox/lidar',
                            '/aligned/livox/imu', '/aligned/sync_status'], output='screen'),
    ])
