# Gazebo

## Version check

```bash
gazebo --version
which gzserver
which gzclient
```

---

## Install
```bash
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control
```

Check your installation:
```bash
ros2 pkg list | grep gazebo
```

---

## kill
```bash
killall -9 gzserver gzclient
```

---

## Launch gazebo
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=$(ros2 pkg prefix --share my_package)/worlds/my_world.world
```
Or from launch file:
```python
world_arg = DeclareLaunchArgument(name='world',
    default_value = os.path.join(get_package_share_path('my_package'), 'worlds/my_world.world'),
    description='Full path to the world model file to load')

gazebo = Node(
    package='gazebo_ros',
    executable='gazebo.launch.py',
)

gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
    )]),
    launch_arguments={'world': LaunchConfiguration('world')}.items()
)
```
The `world` argument is optional. If ommited, an empty world will be loaded.

---

## Spawn urdf model
```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot
```
Or from launch file:
```python
spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'my_bot'
    ]
)
```

## Gazebo tags in xacro
```xml
<!-- colors -->
<gazebo reference="my_link">
    <material>Gazebo/Green</material>
</gazebo>
```

```xml
<!-- *** joint_state_publisher ***
publish the joint_states for the selected joints (which robot_state_publisher can then use to broadcast the approprate tf). -->

<gazebo>
    <plugin name="gazebo_ros_joint_state_publisher"
        filename="libgazebo_ros_joint_state_publisher.so">
        <update_rate>20</update_rate>
        <joint_name>my_joint1</joint_name>
        <joint_name>my_joint2</joint_name>
        <joint_name>my_joint3</joint_name>
    </plugin>
</gazebo>
```

```xml
<!-- *** joint_pose_trajectory ***
    reads JointTrajectory message from the topic 
    /set_joint_trajectory 
    and moves the robot accordingly. -->

<gazebo>
    <plugin name="gazebo_ros_joint_pose_trajectory"
        filename="libgazebo_ros_joint_pose_trajectory.so">
        <update_rate>2</update_rate>
    </plugin>
</gazebo>
```

```xml
<!-- *** camera/depth sensor *** 
An extra joint/link is required to create an "optical frame" for the sensor. For more see https://www.ros.org/reps/rep-0103.html#suffix-frames -->

<!-- First, create the link and joint for the optical frame -->
<joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.571 0 -1.571" />
    <parent link="camera_link" />
    <child link="camera_link_optical" />
</joint>

<!-- camera_link must be the predifined link for the camera -->

<link name="camera_link_optical"></link>

<!-- Add a gazebo tag for the ORIGINAL camera_link (but in the plugin we reference the optical frame so that ROS can orient things correctly) -->
<!-- Although visualise is set to true, it won't actually visualise the depth camera in gazebo. To see the preview, swap "depth" to "camera"-->
<gazebo reference="camera_link">
    <sensor type="depth" name="my_camera">  <!-- For RGB only: <sensor type="camera" ...> -->
        <update_rate>20</update_rate>
        <visualize>true</visualize>
        <camera name="cam">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8B8G8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <frame_name>camera_link_optical</frame_name>
            <min_depth>0.1</min_depth>
            <max_depth>500</max_depth>
        </plugin>
    </sensor>
</gazebo>
```
Note also:
- Add damping to the joints to stop them it flopping around. e.g. `<dynamics damping="10.0" friction="10.0"/>`
- Example message for `gazebo_ros_joint_pose_trajectory`:
    ```bash
    ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory  '{header: {frame_id: world}, joint_names: [slider_joint, arm_joint], points: [  {positions: {0.8,0.6}} ]}'
    ```
    

---
