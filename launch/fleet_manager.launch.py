from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('allocator', default_value='hungarian',
                              description='nearest | hungarian | auction'),
        DeclareLaunchArgument('generate_tasks', default_value='false'),
        DeclareLaunchArgument('period', default_value='15.0'),
        DeclareLaunchArgument('max_tasks', default_value='20'),
        DeclareLaunchArgument('seed', default_value='42'),
        Node(package='amr_fleet_sim', executable='fleet_manager', output='screen',
             parameters=[{'use_sim_time': True,
                          'allocator': LaunchConfiguration('allocator')}]),
        Node(package='amr_fleet_sim', executable='task_generator', output='screen',
             condition=IfCondition(LaunchConfiguration('generate_tasks')),
             parameters=[{'use_sim_time': True,
                          'period': LaunchConfiguration('period'),
                          'max_tasks': LaunchConfiguration('max_tasks'),
                          'seed': LaunchConfiguration('seed')}]),
    ])
