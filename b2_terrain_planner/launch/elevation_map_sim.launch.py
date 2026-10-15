"""기존 launch 이름 호환: MuJoCo GT odometry로 elevation map을 실행한다."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('b2_terrain_planner'))
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/elevation_map.launch.py')),
            launch_arguments={
                'mode': 'sim', 'start_sim': 'true',
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()),
    ])
