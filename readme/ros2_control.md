# Ros2 control

## CLI

```bash
ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_interfaces
```

## Introspection of ros2_control system
- What controllers are loaded in the system?
```bash
ros2 control list_controllers
```
- What is the state of the controllers?
```bash
ros2 control list_controllers
```
- What hardware interfaces are there and in which state?
```bash
ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents {}
```
- Which interfaces are available?
```bash
ros2 control list_hardware_interfaces
```
- Switch between forward_position_controller and joint_trajectory_controller?
```bash
# spawn first the controller you want as `inactive`
ros2 run controller_manager spawner forward_position_controller --inactive
# deactivate the current controller and activate the other one
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```
- What happens when you try to run all controllers in parallel?
```bash
# See output in the terminal where ros2_control_node is running:
ros2 control switch_controllers --activate forward_position_controller --activate joint_trajectory_controller
# You should see something like:
# [ERROR] [1709822963.333649137] [controller_manager]: Resource conflict for controller 'joint_trajectory_controller'. Command interface 'joint1/position' is already claimed.
```
- What interfaces are controllers using?
```bash
ros2 control list_controllers -v
```

## Hardware Description in URDF (example)

More examples at: \
https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md

```xml
<!-- multi-DoF hardware component (system: read/write each DoF) -->
<ros2_control name="RRBotSystemPositionOnly" type="system">
 <hardware>
   <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
   <param name="example_param_write_for_sec">2</param>
   <param name="example_param_read_for_sec">2</param>
 </hardware>
 <joint name="joint1">
   <command_interface name="position">
     <param name="min">-1</param>
     <param name="max">1</param>
   </command_interface>
   <state_interface name="position"/>
 </joint>
 <joint name="joint2">
   <command_interface name="position">
     <param name="min">-1</param>
     <param name="max">1</param>
   </command_interface>
   <state_interface name="position"/>
 </joint>
 <!-- optionally for integrated sensors 
  (notite: the <hardware> plugin must be designed to read the integrated sensor data) 
 -->
 <sensor name="tcp_fts_sensor">
    <state_interface name="fx"/>
    <state_interface name="tz"/>
    <param name="frame_id">rrbot_tcp</param>
    <param name="fx_range">100</param>
    <param name="tz_range">15</param>
  </sensor>
</ros2_control>

<!-- sensor hardware component (sensor: read only) -->
<ros2_control name="RRBotForceTorqueSensor1D" type="sensor">
 <hardware>
   <plugin>ros2_control_demo_hardware/ForceTorqueSensor1DHardware</plugin>
   <param name="example_param_read_for_sec">0.43</param>
 </hardware>
 <sensor name="tcp_fts_sensor">
   <state_interface name="force"/>
   <param name="frame_id">rrbot_tcp</param>
   <param name="min_force">-100</param>
   <param name="max_force">100</param>
 </sensor>
</ros2_control>

<!-- 1-DoF hardware component (actuator: [read]/write) -->
<ros2_control name="RRBotGripper" type="actuator">
 <hardware>
   <plugin>ros2_control_demo_hardware/PositionActuatorHardware</plugin>
   <param name="example_param_write_for_sec">1.23</param>
   <param name="example_param_read_for_sec">3</param>
 </hardware>
 <joint name="gripper_joint ">
   <command_interface name="position">
     <param name="min">0</param>
     <param name="max">50</param>
   </command_interface>
   <state_interface name="position"/>
   <state_interface name="velocity"/>
 </joint>
</ros2_control>

<!-- composite sensor -->
<ros2_control name="CameraWithIMU" type="sensor">
  <hardware>
    <plugin>ros2_control_demo_hardware/CameraWithIMUSensor</plugin>
    <param name="example_param_read_for_sec">2</param>
  </hardware>
  <sensor name="sensor1">
    <state_interface name="roll"/>
    <state_interface name="pitch"/>
    <state_interface name="yaw"/>
  </sensor>
  <sensor name="sensor2">
    <state_interface name="image"/>
  </sensor>
</ros2_control>
```

## Simulation with Gazebo Classic and Gazebo (Ignition)

ros2_control is integrated with simulators using simulator-specific plugins. Those plugins extend controller manager with simulator-related functionalities and enables loading hardware interfaces that are created specifically for the simulator. Simulators are using description under `<ros2_control>` to setup the interfaces. They are searched for interfaces with standard names, `position`, `velocity` and `effort`, to wire them with the internal simulator-states.

The plugins and interfaces for the simulators are the following:

### Gazebo Classic

- Package: `gazebo_ros2_control`
- Simulator plugin: `libgazebo_ros2_control.so`
- HW interface plugin: `gazebo_ros2_control/GazeboSystem`

### Gazebo

- Package: `gz_ros2_control`
- Simulator plugin: `libign_ros2_control-system.so`
- HW interface plugin: `ign_ros2_control/IgnitionSystem` (*NOTE* `ign` will be switched to `gz` very soon!)



## `Setup`:
Include in your robot_description xacro:
```xml

<!-- hardware options: [sim, real, gazebo] -->
<xacro:property name="hardware" value="sim" />

<ros2_control name="MySystem" type="system">
    
  <hardware>

    <!-- Mock hardware -->
    <xacro:if value="${hardware.lower() == 'sim'}">
      <plugin>mock_components/GenericSystem</plugin>
      <param name="mock_sensor_commands">${mock_sensor_commands}</param>
    </xacro:if>

    <!-- physical hardware or Robot simulation studio -->
    <xacro:if value="${hardware.lower() == 'real'}">
      <plugin>elite_hardware_interface/EliteSystemHardware</plugin>
      <param name="dashboard_ip">${dashboard_ip}</param>
      <param name="local_ip">${local_ip}</param>
      <param name="reverse_port">${reverse_port}</param>
      <!-- <param name="...">...</param> -->
    </xacro:if>

    <!-- Gazebo -->
    <xacro:if value="${hardware.lower() == 'gazebo'}">
      <!-- gazebo HardwareInterface plugin -->
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </xacro:if>
  </hardware>

  <!-- Specify the joints and the exposed command/state interfaces-->
  <joint name="left_wheel_joint">
      <!-- <command_interface name="position">
          <param name="min">-2.5</param>
          <param name="max">2.5</param>
      </command_interface> -->
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

- https://control.ros.org/jazzy/doc/getting_started/getting_started.html
- https://github.com/ros-controls/roadmap/blob/master/design_drafts/
- https://github.com/ros-controls/ros2_control_demos
