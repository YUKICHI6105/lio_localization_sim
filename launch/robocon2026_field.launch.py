import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    sim_share = get_package_share_directory('lio_localization_sim')
    gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(sim_share, 'worlds', 'robocon2026_field.sdf')
    minimal_gui_config = os.path.join(
        sim_share, 'config', 'robocon2026_minimal_gui.config')
    use_gui = LaunchConfiguration('use_gui').perform(context).lower() in ('1', 'true', 'yes')
    run = LaunchConfiguration('run').perform(context).lower() in ('1', 'true', 'yes')
    render_mode = LaunchConfiguration('render_mode').perform(context).lower()
    gpu_api = LaunchConfiguration('gpu_api').perform(context).lower()
    gpu_adapter = LaunchConfiguration('gpu_adapter').perform(context)

    # GUI inspection defaults to paused, avoiding continuous physics work.
    # Automated trials should run server-only so rendering consumes no CPU/GPU.
    gz_args = []
    if run:
        gz_args.append('-r')
    if not use_gui:
        gz_args.append('-s')
    else:
        # An explicit GUI config suppresses Gazebo's large default editor
        # plugin set.  This viewer only needs scene rendering and navigation.
        gz_args.extend(['--gui-config', minimal_gui_config])
        if render_mode == 'gpu':
            gz_args.extend([
                '--render-engine-gui', 'ogre2',
                '--render-engine-gui-api-backend', gpu_api,
            ])
    gz_args.append(world)

    rendering_environment = []
    if render_mode == 'gpu':
        # WSLg exposes the Windows GPU to Mesa through the D3D12 Gallium
        # driver. Keep Qt's scene graph on its render thread and explicitly
        # prohibit llvmpipe / softpipe fallback.
        rendering_environment = [
            SetEnvironmentVariable('QSG_RENDER_LOOP', 'threaded'),
            SetEnvironmentVariable('QSG_RHI_BACKEND', 'opengl'),
            SetEnvironmentVariable('QT_OPENGL', 'desktop'),
            # Ogre2's OpenGL backend creates a GLX window, so keep Qt on XCB.
            # Mesa's automatic GLX choice was llvmpipe on this machine; the
            # D3D12 overrides below select the accelerated WSLg path instead.
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),
            SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
            SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'd3d12'),
            SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', gpu_adapter),
            SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '1'),
            SetEnvironmentVariable('vblank_mode', '1'),
        ]
    else:
        rendering_environment = [
            SetEnvironmentVariable('QSG_RENDER_LOOP', 'basic'),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ]

    return rendering_environment + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ' '.join(gz_args),
                'on_exit_shutdown': 'true',
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='true: Gazebo GUI; false: server-only (recommended for trials)'),
        DeclareLaunchArgument(
            'run', default_value='false',
            description='Start physics immediately. Defaults to paused for field inspection.'),
        DeclareLaunchArgument(
            'render_mode', default_value='gpu',
            description='gpu: force WSLg D3D12 hardware rendering; software: CPU fallback'),
        DeclareLaunchArgument(
            'gpu_api', default_value='opengl',
            description='Ogre2 GPU API: opengl (recommended on WSLg) or vulkan'),
        DeclareLaunchArgument(
            'gpu_adapter', default_value='Intel',
            description='Substring of the Windows GPU selected by Mesa D3D12'),
        OpaqueFunction(function=launch_setup),
    ])
