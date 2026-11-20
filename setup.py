import os
from glob import glob
from setuptools import setup

package_name = 'amr_fleet_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'dashboard'), glob('dashboard/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Multi-AMR Gazebo simulation',
    license='MIT',
    entry_points={'console_scripts': ['send_goal = amr_fleet_sim.send_goal:main',
            'fleet_manager = amr_fleet_sim.fleet_manager:main',
            'task_generator = amr_fleet_sim.task_generator:main',
            'dashboard_server = amr_fleet_sim.dashboard_server:main']},
)
