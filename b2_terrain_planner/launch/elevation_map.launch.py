"""입력 설정에 따라 실기·시뮬 센서와 odometry를 연결하는 elevation map 실행."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    mode = value('mode')
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    share = Path(get_package_share_directory('b2_terrain_planner'))
    cupy = Path(get_package_share_directory('elevation_mapping_cupy'))
    python = os.environ.get('ELEVATION_PYTHON', '/usr/bin/python3')
    config = value('input_config')
    if not config:
        config = str(share / 'config' / f'elevation_input_{mode}.yaml')
    input_parameters = [config]
    overrides = {}
    for name in ('odom_topic', 'cloud_topic'):
        if value(name):
            overrides[name] = value(name)
    if overrides:
        input_parameters.append(overrides)
    nodes = []
    if mode == 'sim':
        if value('start_sim') == 'true':
            sim_binary = os.environ.get('PKIRO_SIM_EXECUTABLE', str(
                Path(get_package_prefix('b2_deploy')) / 'lib/b2_deploy/mujoco_node'))
            if not Path(sim_binary).is_file():
                raise RuntimeError('simulation executable missing: ' + sim_binary)
            nodes.append(Node(
                package='b2_deploy', executable=sim_binary, output='screen',
                parameters=[{'odom.enabled': True, 'odom.topic': '/odom',
                             'odom.frame_id': 'odom', 'odom.child_frame_id': 'base_link'}],
                additional_env={'PKIRO_ROBOT_BACKEND': 'mujoco'}))
    nodes.append(Node(package='b2_terrain_planner', executable='elevation_input_adapter.py',
                      prefix=[python], parameters=input_parameters, output='screen'))
    nodes.append(Node(
        package='b2_terrain_planner', executable='elevation_mapping_runner.py', prefix=[python],
        name='elevation_mapping_node', output='screen',
        parameters=[str(cupy / 'config/core/core_param.yaml'),
                    str(share / 'config/elevation_map_sim.yaml'), {
                        'map_frame': 'elevation_odom', 'base_frame': 'elevation_base',
                        'corrected_map_frame': 'elevation_odom',
                        'initialize_frame_id': ['elevation_base'],
                        'service_namespace': '/elevation_mapping_node',
                        'use_initializer_at_start': False,
                        'subscribers.lidar.topic_name': '/elevation/points',
                        'use_sim_time': False,
                    }],
        remappings=[('/tf', '/elevation/tf'), ('/tf_static', '/elevation/tf_static')]))
    if value('launch_rviz') == 'true':
        rviz = value('rviz_config') or str(share / 'config/elevation_map.rviz')
        # Humble GridMap 플러그인이 node callback group보다 먼저 unload되면 종료 시
        # SIGSEGV가 발생한다. 이 RViz 프로세스에서만 라이브러리 수명을 유지한다.
        plugin = Path(get_package_prefix('grid_map_rviz_plugin')) / 'lib'
        preload = str(plugin / 'libgrid_map_rviz_plugin.so')
        if os.environ.get('LD_PRELOAD'):
            preload += ':' + os.environ['LD_PRELOAD']
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz],
                          additional_env={'LD_PRELOAD': preload},
                          remappings=[('/tf', '/elevation/tf'),
                                      ('/tf_static', '/elevation/tf_static')], output='screen'))
    # 하위 프로세스가 죽었는데 나머지가 정상처럼 남지 않도록 함께 종료한다.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[Shutdown(reason='elevation process exited')]))
        for node in nodes]
    return handlers + nodes


def generate_launch_description():
    defaults = dict(mode='real', start_sim='false', launch_rviz='false', input_config='',
                    rviz_config='', odom_topic='', cloud_topic='')
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        OpaqueFunction(function=launch_nodes),
    ])
