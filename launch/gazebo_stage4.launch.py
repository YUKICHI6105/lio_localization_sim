import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('lio_localization_sim')
    lio = get_package_share_directory('lio_localization')
    gz = get_package_share_directory('ros_gz_sim')
    world = os.path.join(sim, 'worlds', 'stage4_physics.sdf')
    bridge = os.path.join(sim, 'config', 'gazebo_bridge.yaml')
    field = os.path.join(lio, 'config', 'field.yaml')
    common = [field, {'use_sim_time': True}]
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_extra_args', default_value='',
            description='Pass -s for server-only/headless execution'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': [LaunchConfiguration('gz_extra_args'), ' -r ', world],
                'on_exit_shutdown': 'true',
            }.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': bridge}], output='screen'),
        Node(package='lio_localization_sim', executable='gazebo_driver_node', parameters=[{'use_sim_time': True}]),
        Node(package='lio_localization', executable='imu_preintegration_node', parameters=common),
        Node(package='lio_localization', executable='backend_optimizer_node', parameters=common),
        Node(package='lio_localization', executable='laser_scan_matching_node', parameters=common),
        Node(package='lio_localization', executable='ball_tracking_node', parameters=[{'use_sim_time': True}]),
    ])
