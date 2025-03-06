from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    model_path_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        f"{os.getenv('GAZEBO_MODEL_PATH', '')}:/home/kimtaemin/gazebo/ros2_ws/src/rover_line_follower/models"
    )
    
    package_path = os.path.dirname(os.path.dirname(__file__))
    urdf_file = os.path.join(package_path, 'urdf', 'simple_rover.urdf')
    world_file = os.path.join(package_path, 'worlds', 'track.world')
    
    # gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={'world': world_file}.items()

    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description", "-entity", "simple_rover"],
        output='screen'
    )

    # rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_path, 'config', 'your_config.rviz')]  # rviz 설정 파일 경로 필요
    )

    # Command viewer node
    command_viewer_node = Node(
        package='rover_line_follower',
        executable='command_viewer',
        name='command_viewer',
        output='screen'
    )

    return LaunchDescription([
        model_path_env,
        gazebo_node,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        command_viewer_node,
    ])