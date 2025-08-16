import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    gazebo_sim_pkg = get_package_share_directory('gazebo_sim')
    world_path = os.path.join(gazebo_sim_pkg, 'worlds', 'diver_world.world')
    xacro_file = os.path.join(gazebo_sim_pkg, 'urdf', 'AUV', 'base.xacro')
    model_path = os.path.join(gazebo_sim_pkg, 'models')
    plugin_path = os.path.join(gazebo_sim_pkg, '..', '..', 'lib/gazebo_sim')

    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    set_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path)
    set_plugin_path = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path)
    set_qt_platform = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb')

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-u', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    start_gazebo_client_cmd = ExecuteProcess(cmd=['gzclient'], output='screen')

    robot_state_publisher_node = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description])
    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'AUV'], output='screen')

    auv_orchestrator_node = Node(
        package='auv_control',
        executable='auv_orchestrator_node',
        name='auv_orchestrator',
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        set_qt_platform,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        
        auv_orchestrator_node,
    ])