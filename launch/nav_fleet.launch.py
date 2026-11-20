import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from amr_fleet_sim.world_config import DEFAULT_WORLD, WORLDS, world_files

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
    files = world_files(pkg_dir, LaunchConfiguration('world').perform(context))
    with open(files['robots']) as f:
        robots = yaml.safe_load(f)['robots']

    actions = []
    for i, r in enumerate(robots):
        ns = r['name']
        params = _render_params(pkg_dir, r)
        common = dict(namespace=ns, output='screen', parameters=[params])

        servers = [
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 remappings=[('map', '/map')], **common),
            Node(package='nav2_controller', executable='controller_server', **common),
            Node(package='nav2_planner', executable='planner_server', **common),
            Node(package='nav2_behaviors', executable='behavior_server', **common),
            Node(package='nav2_bt_navigator', executable='bt_navigator', **common),
        ]
        # Humble's lifecycle manager aborts the whole bring-up if one server's services are not
        # discoverable yet (common on a loaded CPU), so give the servers a head start.
        manager = [
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', namespace=ns, output='screen',
                 parameters=[{'use_sim_time': True,
                              'autostart': True,
                              'bond_timeout': 30.0,     # heartbeats arrive late on a busy CPU
                              'node_names': ['amcl', 'controller_server',
                                             'planner_server', 'behavior_server',
                                             'bt_navigator']}]),
        ]
        nodes = servers + [TimerAction(period=6.0, actions=manager)]
        # Start this robot's Nav2 only once Gazebo has loaded the world, spawned the robot and
        # its odometry is flowing (a big world can take most of a minute to load), then stagger
        # the robots so their lifecycle bring-ups do not all compete for the CPU at once.
        wait = ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '--once', f'/{ns}/odom', 'nav_msgs/msg/Odometry'],
            name=f'wait_for_{ns}', output='log')
        actions += [wait, RegisterEventHandler(OnProcessExit(
            target_action=wait, on_exit=[TimerAction(period=2.0 + 6.0 * i, actions=nodes)]))]

    map_yaml = LaunchConfiguration('map').perform(context) or files['map']
    actions.append(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'multi_robot_sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'camera': LaunchConfiguration('camera')}.items())

    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'fleet.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD, choices=list(WORLDS)),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value='', description='override the world\'s map .yaml'),
        sim, lifecycle_map,
        OpaqueFunction(function=_robot_stack),
        TimerAction(period=10.0, actions=[rviz]),
    ])
