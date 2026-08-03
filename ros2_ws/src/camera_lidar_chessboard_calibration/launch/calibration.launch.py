from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('camera_lidar_chessboard_calibration'), 'config', 'calibration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.152'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.41'),
        DeclareLaunchArgument('output_dir', default_value='calibration_samples'),
        DeclareLaunchArgument('auto_save_max_samples', default_value='30'),
        Node(package='camera_lidar_chessboard_calibration', executable='livox_direct',
             parameters=[{'lidar_ip': LaunchConfiguration('lidar_ip'),
                          'host_ip': LaunchConfiguration('host_ip')}], output='screen'),
        Node(package='camera_lidar_chessboard_calibration', executable='camera_rtsp',
             parameters=[cfg, {'password': LaunchConfiguration('password')}], output='screen'),
        Node(package='camera_lidar_chessboard_calibration', executable='capture_gui',
             parameters=[cfg, {'cloud_topic': LaunchConfiguration('cloud_topic'),
                               'output_dir': LaunchConfiguration('output_dir'),
                               'auto_save_max_samples': LaunchConfiguration('auto_save_max_samples')}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='livox_rviz',
             arguments=['-d', os.path.join(get_package_share_directory('camera_lidar_chessboard_calibration'), 'config', 'livox.rviz')],
             output='screen'),
    ])
