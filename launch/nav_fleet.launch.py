import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'amr_fleet_sim'


def _render_params(pkg_dir, r):
    """Fill the Nav2 template for one robot and return the temp file path."""
    with open(os.path.join(pkg_dir, 'config', 'nav2_params_template.yaml')) as f:
        txt = f.read()
    subs = {'ROBOT': r['name'], 'X': float(r['x']), 'Y': float(r['y']),
            'YAW': float(r.get('yaw', 0.0))}
    for k, v in subs.items():
        txt = txt.replace('${' + k + '}', str(v))
    out = os.path.join(tempfile.gettempdir(), f"{r['name']}_nav2_params.yaml")
    with open(out, 'w') as f:
        f.write(txt)
    return out


def _robot_stack(context, *args, **kwargs):
    pkg_dir = get_package_share_directory(PKG)
    robots_file = os.path.join(pkg_dir, 'config', 'robots.yaml')
    with open(robots_file) as f:
        robots = yaml.safe_load(f)['robots']

    actions = []
    for i, r in enumerate(robots):
        ns = r['name']
        params = _render_params(pkg_dir, r)
        common = dict(namespace=ns, output='screen', parameters=[params])

        nodes = [
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 remappings=[('map', '/map')], **common),
            Node(package='nav2_controller', executable='controller_server', **common),
            Node(package='nav2_planner', executable='planner_server', **common),
            Node(package='nav2_behaviors', executable='behavior_server', **common),
            Node(package='nav2_bt_navigator', executable='bt_navigator', **common),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': True,
                              'autostart': True,
                              'bond_timeout': 10.0,
                              'node_names': ['amcl', 'controller_server',
                                             'planner_server', 'behavior_server',
                                             'bt_navigator']}]),
        ]
        # stagger start-up so 3 robots do not hammer the CPU at once
        actions.append(TimerAction(period=8.0 + 4.0 * i, actions=nodes))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'multi_robot_sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'camera': LaunchConfiguration('camera')}.items())

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'),
                     'use_sim_time': True}])

    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'fleet.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_dir, 'maps', 'warehouse.yaml')),
        sim, map_server, lifecycle_map,
        OpaqueFunction(function=_robot_stack),
        TimerAction(period=10.0, actions=[rviz]),
    ])
