import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Get package and file paths
    pkg_share = get_package_share_directory('gazebo_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'diver_world.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'AUV', 'base.xacro')

    # Define paths for environment variables
    model_path = os.path.join(pkg_share, 'models')
    # The plugin is installed in the 'lib' directory of your package's install space
    plugin_path = os.path.join(get_package_share_directory('gazebo_sim'), '..', '..', 'lib/gazebo_sim')


    # 2. Process XACRO to get robot_description
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    # --- KEY FIX #1: Set the Gazebo Model Path ---
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=model_path
    )

    # --- KEY FIX #2: Set the Gazebo Plugin Path ---
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', value=plugin_path
    )

    # --- KEY FIX #3: Set the QT Platform for the GUI ---
    set_qt_platform = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='xcb'
    )

    # 4. Define the Gazebo Server process
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '--pause',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen'
    )

    # 5. Define the Gazebo Client process
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 6. Define the standard ROS 2 nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'AUV'],
        output='screen'
    )

    # 7. Assemble and return the LaunchDescription
    return LaunchDescription([
        set_model_path,
        set_plugin_path, # Add the new action to the list
        set_qt_platform,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
    ])