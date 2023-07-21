import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory  # get pkg path
from launch_ros.actions import Node # create ros nodes
from launch import LaunchDescription # generate the final launch description
from launch.conditions import IfCondition, UnlessCondition  # conditional launch node
from launch.substitutions import Command # execute a bash command
from launch_ros.parameter_descriptions import ParameterValue # define a param
from launch.actions import DeclareLaunchArgument # define a launch argument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration # retrieve the value of an argument
from launch.substitutions import AndSubstitution, OrSubstitution, NotSubstitution # for composite conditions
from launch.actions import IncludeLaunchDescription # include another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource # wrapper for the path to `IncludeLaunchDescription`

def generate_launch_description():

    package_name = 'articubot_one'

    use_ros2_control_arg = DeclareLaunchArgument(name='use_ros2_control', default_value="true", choices=['true', 'false'], description='Flag to use ros2 control')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    sim_mode_arg = DeclareLaunchArgument(name='sim_mode', default_value="true", choices=['true', 'false'], description='Flag to use Gazebo')
    sim_mode = LaunchConfiguration('sim_mode')

    ## ======  robot_state_publisher  =======

    xacro_file = os.path.join(get_package_share_path(package_name), 'description/robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', sim_mode])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description, 
            'use_sim_time': sim_mode
        }]
    )

    ## ======  joint_state_publisher  =======
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=UnlessCondition(sim_mode)
    )

    ## ======  rviz  =======
    default_rviz_config_path = os.path.join(get_package_share_path('package_name'),'rviz/urdf.rviz')
    rviz_cfg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    ## ======  gazebo  =======
    world_file = os.path.join(get_package_share_path(package_name), 'worlds/obstacles.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
        launch_arguments={'world': world_file}.items()
    )

    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ]
    )

    gazebo_nodes = GroupAction(
        actions=[gazebo, spawn_entity],
        condition=IfCondition(sim_mode)
    )

    ## ======  LaunchDescription  =======

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        gazebo_nodes,
    ])  