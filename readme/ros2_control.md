# Ros2 control


## `Setup`:
Include in your robot_description xacro:
```xml
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <!-- Specify the joints 
    and the exposed command/state interfaces-->
    <joint name="left_wheel_joint">
        <command_interface name="velocity">
            <param name="min">-10</param>
            <param name="max">10</param>
        </command_interface>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
    </joint>
    <joint name="right_wheel_joint">
        <command_interface name="velocity">
            <param name="min">-10</param>
            <param name="max">10</param>
        </command_interface>
        <state_interface name="velocity"/>
        <state_interface name="position"/>
    </joint>
</ros2_control>

<!-- Include the gazebo_ros2_control plugin
This will launch the node: /gazebo_ros2_control -->
<gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <!-- Specify the ros2_controllers and their params.
    This allows us to spawn (via the controller_manager) 
    the corresponding ros2_controllers in the launch file-->
        <parameters>$(find package_name)/config/controllers.yaml</parameters>
        <!-- Can specify additional params including another file -->
        <parameters>$(find package_name)/config/extra_ctrl_params.yaml</parameters>
    </plugin>
</gazebo>
```

Example for `controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 30
    # use_sim_time: true

    my_controller:
        type: <ros2_ctrl_package>/<class name>

    other_controller:
        type: <ros2_ctrl_package>/<class name>

my_controller:
  ros__parameters:
    # ...

other_controller:
  ros__parameters:
    # ...
```
The above yaml can also be used with a real controller. So we can specify additional params for Gazebo (simulation) using the `extra_ctrl_params`:
```yaml
controller_manager:
  ros__parameters:
    use_sim_time: true
```

## `controller_manager`
```python
robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')

controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[{'robot_description': robot_description},
                controller_params_file]
)
```
- **Required only in real**! In `gazebo` (simulation), the controller manager is launched automatically.

## `Spawn` controller
In the launch file:
```python
my_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_controller"], # the controller name specified in the yaml
    )
```

## `Joint_state_publisher`
Reads all state interfaces and reports them on `/joint_states` and `/dynamic_joint_states`.
```yaml
controller_manager:
  ros__parameters:
    # ...
    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_broad:
  ros__parameters:
    # joints: ['my_joint1', ...]
    # interfaces: ['my_joint1', ...]
    # ...
```

## Links

For more:

https://control.ros.org/master/doc/getting_started/getting_started.html

https://github.com/ros-controls/ros2_control_demos
