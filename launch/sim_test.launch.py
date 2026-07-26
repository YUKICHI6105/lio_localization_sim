import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lio_localization_sim.trajectory import Trajectory


def launch_setup(context, *args, **kwargs):
    lio_share = get_package_share_directory('lio_localization')
    default_config = os.path.join(lio_share, 'config', 'field.yaml')

    field_config = LaunchConfiguration('field_config').perform(context)
    duration_sec = LaunchConfiguration('duration_sec').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    field_width = float(LaunchConfiguration('field_width').perform(context))
    field_height = float(LaunchConfiguration('field_height').perform(context))
    trajectory_pattern = int(LaunchConfiguration('trajectory_pattern').perform(context))

    # main.md ③の初期値PriorFactorは「事前確定の初期位置座標」を前提にしている。
    # シミュレータの軌道がt=0にちょうど原点にいるとは限らないため、真の初期姿勢を
    # 計算してbackend_optimizer_nodeへそのまま伝える（実運用で「スタート地点の
    # 既知座標を初期値として与える」ことに相当する）。
    traj = Trajectory(field_width, field_height, pattern=trajectory_pattern)
    s0 = traj.state(0.0)

    # main.md 6-A-1: imu_sim/lidar_sim/evaluatorが各々の起動時刻をt=0にすると、
    # ノード起動タイミングのズレ(数十ms)が速度×ズレの恒常誤差として現れる
    # (fableによる敵対的レビューで指摘)。ここで単一の時刻基準を計算し、
    # 全ノードに共有する。+2.0sはノード起動の猶予(起動が間に合わずt<0の
    # メッセージが出ることを避けるための単純なバッファ)。
    sim_start_time_sec = time.time() + 2.0
    # 診断解析用: 各ノードの壁時計スタンプを軌道相対時刻へ換算する基準を残す
    # (launch起動時の定数書き出し。ノードのホットパスではないので汚染しない)。
    try:
        with open('/tmp/sim_start_time.txt', 'w') as _f:
            _f.write(f'{sim_start_time_sec:.9f}\n')
    except OSError:
        pass

    imu_sim = Node(
        package='lio_localization_sim', executable='imu_sim_node',
        name='imu_sim_node',
        parameters=[field_config, {
            'field_width': field_width, 'field_height': field_height,
            'sim_start_time_sec': sim_start_time_sec,
            'trajectory_pattern': trajectory_pattern}],
        output='screen')
    lidar_sim = Node(
        package='lio_localization_sim', executable='lidar_sim_node',
        name='lidar_sim_node',
        parameters=[field_config, {
            'field_width': field_width, 'field_height': field_height,
            'sim_start_time_sec': sim_start_time_sec,
            'trajectory_pattern': trajectory_pattern}],
        output='screen')
    evaluator = Node(
        package='lio_localization_sim', executable='evaluator_node',
        name='evaluator_node',
        parameters=[field_config, {
            'field_width': field_width, 'field_height': field_height,
            'duration_sec': float(duration_sec),
            'output_dir': output_dir,
            'sim_start_time_sec': sim_start_time_sec,
            'trajectory_pattern': trajectory_pattern}],
        output='screen')

    # ライブ可視化(WSLg等の表示環境があるとき use_rviz:=true で有効化)。
    # /scan(壁・円柱の点群)・/odom_fast(推定・青)・/ground_truth_pose(真値・緑)を
    # map座標で俯瞰表示する。ヘッドレスCIでは既定offにしておく。
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in ('1', 'true', 'yes')

    imu_node = Node(
        package='lio_localization', executable='imu_preintegration_node',
        name='imu_preintegration_node',
        # 通常のシミュレーションではTFの購読者がいないため、1kHzのTF配信負荷を止める
        # (②は/odom_fastを直接購読しTFを使わない。実機launchでは有効のままにする)。
        # ただしrviz有効時は購読者ができる: rvizのFixed Frameはmapで、/scanは
        # base_linkフレームのため、map->odom(③が配信)に加えてodom->base_link(①)が
        # 無いと点群を変換できず表示が丸ごと落ちる。よってuse_rvizに追従させる。
        # field_configも渡すことで、yamlのdiag_odom_path等(診断)を①に届ける。
        parameters=[field_config, {'publish_tf': use_rviz}],
        output='screen')
    backend_node = Node(
        package='lio_localization', executable='backend_optimizer_node',
        name='backend_optimizer_node',
        parameters=[field_config, {
            'initial_x': s0.x, 'initial_y': s0.y, 'initial_theta': s0.yaw,
            'initial_vx': s0.vx, 'initial_vy': s0.vy}],
        output='screen')
    scan_matching_node = Node(
        package='lio_localization', executable='laser_scan_matching_node',
        name='laser_scan_matching_node',
        parameters=[field_config, {
            'field_width': field_width, 'field_height': field_height,
            'initial_x': s0.x, 'initial_y': s0.y, 'initial_theta': s0.yaw}],
        output='screen')
    ball_tracking_node = Node(
        package='lio_localization', executable='ball_tracking_node',
        name='ball_tracking_node', output='screen')

    extra = []
    if use_rviz:
        rviz_cfg = os.path.join(
            get_package_share_directory('lio_localization_sim'), 'rviz', 'sim_view.rviz')
        extra.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_cfg], output='screen',
            # WSLg offers both Wayland and XWayland.  RViz's OGRE GLX backend
            # needs the latter; auto-selecting Wayland produces an invalid
            # GLX parent-window handle.
            additional_env={'QT_QPA_PLATFORM': 'xcb'}))

    # evaluatorがレポートを書いて終了したらlaunch全体を終了させる
    # (これが無いとシムノード・実ノードが走り続け、次の検証実行と
    #  同一DDSドメイン上で衝突する)。
    shutdown_on_eval_exit = RegisterEventHandler(OnProcessExit(
        target_action=evaluator,
        on_exit=[EmitEvent(event=Shutdown(reason='evaluation finished'))]))

    return [
        imu_sim, lidar_sim, evaluator,
        imu_node, backend_node, scan_matching_node, ball_tracking_node,
        shutdown_on_eval_exit,
    ] + extra


def generate_launch_description():
    lio_share = get_package_share_directory('lio_localization')
    default_config = os.path.join(lio_share, 'config', 'field.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'field_config', default_value=default_config,
            description='main.mdのフィールド形状・固定円柱リスト(実ノードとシムノードで共有)'),
        DeclareLaunchArgument(
            'trajectory_pattern', default_value='0',
            description='検証軌道パターン(0=既定/1=高速並進/2=スピン多発/3=壁際周回/4=左スタートからビンゴ手前)'),
        DeclareLaunchArgument(
            'duration_sec', default_value='180.0',
            description='評価を実行する秒数(main.md想定の3分間走行)'),
        DeclareLaunchArgument(
            'output_dir', default_value='/tmp',
            description='評価レポート・CSV・グラフの出力先'),
        DeclareLaunchArgument(
            'field_width', default_value='10.0',
            description='field.yamlと一致させること(真の初期姿勢計算に使用)'),
        DeclareLaunchArgument(
            'field_height', default_value='8.0',
            description='field.yamlと一致させること(真の初期姿勢計算に使用)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='true でrviz2ライブ可視化を起動(WSLg等の表示環境が必要)'),
        OpaqueFunction(function=launch_setup),
    ])
