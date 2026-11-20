from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_sim.world_config import DEFAULT_WORLD, FLEET_PARAMS, WORLDS, world_files


def _nodes(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    files = world_files(get_package_share_directory('amr_fleet_sim'), world)
    return [
        Node(package='amr_fleet_sim', executable='fleet_manager', output='screen',
             parameters=[{'use_sim_time': True,
                          'allocator': LaunchConfiguration('allocator'),
                          'stations_file': files['stations'],
                          'robots_file': files['robots'],
                          'roadmap_file': files['roadmap'],
                          **FLEET_PARAMS[world]}]),
        Node(package='amr_fleet_sim', executable='task_generator', output='screen',
             condition=IfCondition(LaunchConfiguration('generate_tasks')),
             parameters=[{'use_sim_time': True,
                          'stations_file': files['stations'],
                          'period': LaunchConfiguration('period'),
                          'max_tasks': LaunchConfiguration('max_tasks'),
                          'seed': LaunchConfiguration('seed')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD, choices=list(WORLDS)),
        DeclareLaunchArgument('allocator', default_value='hungarian',
                              description='nearest | hungarian | auction'),
        DeclareLaunchArgument('generate_tasks', default_value='false'),
        DeclareLaunchArgument('period', default_value='15.0'),
        DeclareLaunchArgument('max_tasks', default_value='20'),
        DeclareLaunchArgument('seed', default_value='42'),
        OpaqueFunction(function=_nodes),
    ])
