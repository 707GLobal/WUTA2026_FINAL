"""Open RViz and a read-only FSD output adapter; does not start FSD itself."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fsd_visualization.configuration import render


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('fsd_visualization'))
    settings = yaml.safe_load(Path(value('topics_config')).read_text())
    template = yaml.safe_load(Path(value('rviz_config')).read_text())
    mode = value('mode')
    sim_time = value('use_sim_time') == 'true'
    velocity_type = value('velocity_type')
    temp = tempfile.TemporaryDirectory(prefix='fsd-view-')
    generated = Path(temp.name) / 'viewer.rviz'
    try:
        generated.write_text(yaml.safe_dump(render(template, settings, mode), sort_keys=False))
    except Exception:
        temp.cleanup()
        raise
    bridge = Node(
        package='fsd_visualization', executable='display_bridge',
        name='fsd_display_bridge', output='screen',
        parameters=[{
            'topics_config': value('topics_config'), 'bridge_mode': mode == 'bridge',
            'velocity_type': velocity_type, 'use_sim_time': sim_time,
        }],
    )
    def cleanup(_context):
        temp.cleanup()
        return []
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])), bridge]
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=bridge, on_exit=[EmitEvent(event=Shutdown(reason='Display bridge exited'))])))
    if value('rviz') == 'true':
        viewer = Node(package='rviz2', executable='rviz2', name='fsd_rviz',
                      arguments=['-d', str(generated)], parameters=[{'use_sim_time': sim_time}],
                      output='screen')
        actions.extend([viewer, RegisterEventHandler(OnProcessExit(
            target_action=viewer, on_exit=[EmitEvent(event=Shutdown(reason='Viewer closed'))]))])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('fsd_visualization'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='native', choices=['native', 'bridge']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('velocity_type', default_value='twist', choices=['twist', 'odometry']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('topics_config', default_value=str(share/'config/topics.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'config/fsd.rviz')),
        OpaqueFunction(function=setup),
    ])
