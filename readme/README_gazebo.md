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
ros2 launch gazebo_ros gazebo.launch.py
```
Or from launch file:
```python
gazebo = Node(
    package='gazebo_ros',
    executable='gazebo.launch.py',
)

gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
    )])
)
```

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

---
