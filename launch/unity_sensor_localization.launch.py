import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('lio_localization_sim')
    default_config = os.path.join(sim_share, 'config', 'robocon2026_unity.yaml')
    config = LaunchConfiguration('field_config')
    common = [config, {'use_sim_time': True}]
    # 2026-07-31追加: realtime_evaluator_nodeはduration_sec(既定60秒)で必ず終了し、
    # その終了イベントがlaunch全体をシャットダウンする(下記OnProcessExit参照)。これは
    # 「60秒の有界ベンチマーク」専用の設計で、ILC学習のように何ラップも繰り返し長時間
    # 走らせたい用途には向かない(60秒経過で自己位置推定ノードごと落ちてしまう)。
    # enable_evaluator:=falseで評価ノードとその自動シャットダウンハンドラを両方外し、
    # 自己位置推定スタックだけを無期限に動かせるようにする(既定はtrueで従来どおり
    # stage4_mcp_run.shのベンチマーク動作を変えない)。
    enable_evaluator = LaunchConfiguration('enable_evaluator')
    # 2026-08-01追加: pathplannning ⇄ Unity実機物理接続検証で、yaw推定が真値から
    # 乗離する事象を診断するため、既存のDiagRecorder(空文字なら無効、性能に影響
    # しない)への出力先を起動時に指定できるようにする。既定は空文字で従来通り無効。
    diag_odom_path = LaunchConfiguration('diag_odom_path')
    diag_wall_sigma_path = LaunchConfiguration('diag_wall_sigma_path')
    # 2026-08-02: updated from -2.419/1.354 (old start-zone spawn) to match the
    # rulebook-corrected start zone / robot spawn point (see
    # [[project_field_start_zone_fix_20260802]]). This dict is passed AFTER
    # `config` in each node's parameters=[...] list, so it silently overrode
    # robocon2026_unity.yaml's own initial_x/initial_y even after fixing that
    # file -- backend_optimizer_node kept re-initializing at the stale pose
    # and perpetually respawning ("resuming from relocalization/reset
    # handoff") because it never matched the robot's actual Unity spawn.
    initial_pose = {
        'use_sim_time': True,
        'initial_x': -1.919,
        'initial_y': 1.3715,
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
            name='imu_preintegration_node',
            parameters=[*common, {'diag_odom_path': diag_odom_path}], output='screen'),
        Node(
            package='lio_localization', executable='backend_optimizer_node',
            name='backend_optimizer_node',
            parameters=[config, backend_initial, {'diag_wall_sigma_path': diag_wall_sigma_path}],
            output='screen',
            # パーティクル再収束(relocalization_event)時にbackendは自己終了して
            # respawnに復帰を委ねる(docs/experiment_history/22番以降参照)。
            respawn=True, respawn_delay=0.2),
        Node(
            package='lio_localization', executable='laser_scan_matching_node',
            name='laser_scan_matching_node', parameters=[config, initial_pose], output='screen'),
        Node(
            package='lio_localization', executable='ball_tracking_node',
            name='ball_tracking_node', parameters=common, output='screen',
            # Ball tracking is not on the self-localization safety path. Give
            # IMU/backend callbacks precedence during a host CPU burst.
            prefix='nice -n 10'),
        Node(
            # /odom_fast is 1 kHz.  Keep its real-time comparison path in C++ so
            # Python allocation/GC cannot contend with the localization nodes.
            # The Python evaluator remains available for offline plotting only.
            package='lio_localization', executable='realtime_evaluator_node',
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
                'output_dir': '/tmp',
            }],
            output='screen',
            # The evaluator is experiment instrumentation. It must never win
            # CPU time over the 1 kHz backend IMU callback it is observing.
            prefix='nice -n 10',
            condition=IfCondition(enable_evaluator)),
    ]

    evaluator = localization_nodes[-1]
    return LaunchDescription([
        DeclareLaunchArgument('field_config', default_value=default_config),
        # 2026-07-31追加: falseにすると評価ノード(60秒で終了し、その終了がlaunch全体を
        # シャットダウンする)を起動しない。ILC学習のように自己位置推定スタックだけを
        # 長時間動かし続けたい場合に使う(例:
        # `ros2 launch lio_localization_sim unity_sensor_localization.launch.py
        # enable_evaluator:=false`)。既定はtrueでstage4_mcp_run.shの挙動を変えない。
        DeclareLaunchArgument('enable_evaluator', default_value='true'),
        DeclareLaunchArgument('diag_odom_path', default_value=''),
        DeclareLaunchArgument('diag_wall_sigma_path', default_value=''),
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
        # enable_evaluator:=falseのときは評価ノード自体が起動しないため、この
        # ハンドラも無効にする(でないとlaunch記述としては存在するが決して発火しない
        # だけで実害は無いが、条件を揃えて明示的にしておく)。
        RegisterEventHandler(
            OnProcessExit(target_action=evaluator, on_exit=[EmitEvent(event=Shutdown())]),
            condition=IfCondition(enable_evaluator)),
    ])
