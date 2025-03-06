from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import SetEnvironmentVariable, TimerAction

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory("SAFETRACKER"))
    controller_pkg_path = get_package_share_directory("line_follower_controller")
    
    xacro_file = os.path.join(pkg_path, "urdf", "simple_rover.urdf.xacro")
    world_file = os.path.join(pkg_path, "worlds", "track.world")

    model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH',f"{os.getenv('GAZEBO_MODEL_PATH', '')}:{pkg_path}/models")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items()
    )

    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot',
                   '-z', '0.5',
                   '-Y', '3.1415'],
        output='screen',
    )

    line_follower_node = Node(
        package='line_follower_controller',
        executable='line_follower',
        name='line_follower',
        output='screen',
    )


    nodes_to_start = [model_path_env,
                      robot_state_publisher_node,
                      gazebo_launch,
                      urdf_spawner_node,
                      #launch with Timer 
                      #TimerAction(actions=[line_follower_node], period=10.0)
                      ]


    return LaunchDescription(nodes_to_start)
