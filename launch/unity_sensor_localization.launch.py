import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('lio_localization_sim')
    default_config = os.path.join(sim_share, 'config', 'robocon2026_unity.yaml')
    config = LaunchConfiguration('field_config')
    common = [config, {'use_sim_time': True}]
    initial_pose = {
        'use_sim_time': True,
        'initial_x': -2.419,
        'initial_y': 1.354,
        'initial_theta': 0.0,
    }
    backend_initial = {
        **initial_pose,
        'initial_vx': 0.0,
        'initial_vy': 0.0,
    }

    localization_nodes = [
        Node(
            package='lio_localization', executable='imu_preintegration_node',
            name='imu_preintegration_node', parameters=common, output='screen'),
        Node(
            package='lio_localization', executable='backend_optimizer_node',
            name='backend_optimizer_node', parameters=[config, backend_initial], output='screen'),
        Node(
            package='lio_localization', executable='laser_scan_matching_node',
            name='laser_scan_matching_node', parameters=[config, initial_pose], output='screen'),
        Node(
            package='lio_localization', executable='ball_tracking_node',
            name='ball_tracking_node', parameters=common, output='screen'),
        Node(
            package='lio_localization_sim', executable='evaluator_node',
            name='unity_evaluator_node',
            parameters=[config, {
                'use_sim_time': True,
                # Mission route duration (quintic minimum-jerk over start->pickup->3
                # slalom waypoints->finish, ~5.77m total) is ~6.5s at the field's
                # max_speed/max_acceleration. 30s cut ROS off mid-mission: the estimate
                # froze at its last value while Unity kept servoing off it, producing
                # a >1m true-vs-estimated gap that had nothing to do with localisation
                # accuracy. 60s leaves ample margin to see the mission actually finish
                # (or fail to, on its own terms). The competition match itself is 5min
                # (300s, rule 2.10.1); raise this further for a full-match run.
                'duration_sec': 60.0,
                'settle_sec': 3.0,
                'sim_start_time_sec': 0.0,
                'trajectory_pattern': 4,
                'field_width': 5.638,
                'field_height': 3.376,
                'output_dir': '/tmp',
            }],
            output='screen'),
    ]

    evaluator = localization_nodes[-1]
    return LaunchDescription([
        DeclareLaunchArgument('field_config', default_value=default_config),
        Node(
            package='ros_tcp_endpoint', executable='default_server_endpoint',
            name='unity_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}],
            output='screen'),
        # Give Unity time to connect and register every publisher before the strict
        # 300 ms IMU fail-safe watchdog starts.
        TimerAction(period=5.0, actions=localization_nodes),
        # An evaluation is a bounded experiment.  Do not leave a failed backend
        # running indefinitely after the evaluator has emitted its final report.
        RegisterEventHandler(
            OnProcessExit(target_action=evaluator, on_exit=[EmitEvent(event=Shutdown())])),
    ])
