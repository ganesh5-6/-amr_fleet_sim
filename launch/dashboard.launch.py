from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _video(context, *args, **kwargs):
    """MJPEG camera streams for the dashboard (optional: skipped if the package is missing)."""
    if LaunchConfiguration('video').perform(context).lower() != 'true':
        return []
    try:
        get_package_prefix('web_video_server')
    except PackageNotFoundError:
        return [LogInfo(msg='[dashboard] web_video_server not installed - camera panel disabled. '
                            'Install: sudo apt install ros-humble-web-video-server')]
    return [Node(package='web_video_server', executable='web_video_server', name='web_video_server',
                 output='screen', parameters=[{'port': int(LaunchConfiguration('video_port').perform(context))}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('video', default_value='true'),
        DeclareLaunchArgument('video_port', default_value='8181'),
        # ROS <-> browser bridge (WebSocket, JSON)
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket',
             output='screen', parameters=[{'port': LaunchConfiguration('ws_port')}]),
        # serves the page + map + roadmap
        Node(package='amr_fleet_sim', executable='dashboard_server', output='screen',
             arguments=['--port', LaunchConfiguration('web_port')]),
        OpaqueFunction(function=_video),
    ])
