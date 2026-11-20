import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_sim.world_config import DEFAULT_WORLD, WORLDS, world_files


# Gazebo colours (names from gazebo.material) and the matching RGBA for RViz.
# They mirror the dashboard palette: cyan, violet, amber, green, pink.
STYLES = [('Gazebo/Turquoise', '0.13 0.83 0.93 1'), ('Gazebo/Purple', '0.55 0.36 0.96 1'),
          ('Gazebo/Orange', '0.96 0.62 0.04 1'), ('Gazebo/Green', '0.06 0.73 0.50 1'),
          ('Gazebo/Red', '0.93 0.27 0.60 1')]


def _sim(context, *args, **kwargs):
    pkg = get_package_share_directory('amr_fleet_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'amr.urdf.xacro')
    files = world_files(pkg, LaunchConfiguration('world').perform(context))
    robots_file = LaunchConfiguration('robots_file').perform(context) or files['robots']
    camera = LaunchConfiguration('camera').perform(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'world': files['world'],
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'false'}.items())

    with open(robots_file, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    # scoped: gazebo.launch.py has its own 'world' argument (a file path) that would overwrite ours
    actions = [GroupAction([gazebo], scoped=True)]
    for i, r in enumerate(robots):
        name = r['name']
        gz_mat, rgba = STYLES[i % len(STYLES)]
        urdf = xacro.process_file(xacro_file, mappings={
            'robot_name': name, 'gz_material': gz_mat, 'rgba': rgba,
            'idx': str(i + 1), 'camera': camera}).toxml()

        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'robot_description': urdf,
                'frame_prefix': name + '/',
                'use_sim_time': True,
            }]))

        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-entity', name,
                       '-topic', f'/{name}/robot_description',
                       '-x', str(r['x']), '-y', str(r['y']), '-z', '0.05',
                       '-Y', str(r.get('yaw', 0.0))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD, choices=list(WORLDS)),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='spawn robots with a front camera (needs a working GPU/OGRE)'),
        DeclareLaunchArgument('robots_file', default_value='',
                              description='override the world\'s robots.yaml'),
        OpaqueFunction(function=_sim),
    ])
