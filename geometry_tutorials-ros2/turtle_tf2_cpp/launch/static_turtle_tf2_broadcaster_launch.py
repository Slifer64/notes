from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='turtle_tf2_cpp',
             executable='static_turtle_tf2_broadcaster',
             arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
        ),
        # instead we can use the 'tf2_ros' package
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
             # or using quaternions
            #  arguments = ['--x', '0', '--y', '0', '--z', '1', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
        )
    ])