# safetracker_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('safetracker_auto_drive')
    cfg = os.path.join(pkg_share, 'config', 'auto_drive.yaml')

    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/camera/left/image_raw')
    port_arg        = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    baud_arg        = DeclareLaunchArgument('baud', default_value='115200')

    return LaunchDescription([
        image_topic_arg, port_arg, baud_arg,
        Node(
            package='safetracker_auto_drive',
            executable='lane_follow_node',
            name='lane_follow_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'cmd_topic': '/cmd_vel',
                'use_crosswalk_stop': True,
                'crosswalk_topic': '/autodrive/crosswalk_stop'
            }, cfg]
        ),
        Node(
            package='safetracker_auto_drive',
            executable='crosswalk_node',
            name='crosswalk_node',
            output='screen',
            parameters=[{'image_topic': LaunchConfiguration('image_topic')}, cfg]
        ),
        Node(
            package='safetracker_auto_drive',
            executable='serial_bridge_node',   # setup.py의 entry_points 이름과 동일해야 함
            name='serial_bridge_node',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port'), 'baud': LaunchConfiguration('baud')}]
        ),
    ])

