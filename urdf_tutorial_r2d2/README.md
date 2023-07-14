

# Create a package

```bash
mkdir -p ~/second_ros2_ws/src
cd ~/second_ros2_ws/src
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy --license Apache-2.0
cd urdf_tutorial_r2d2
```

# Install the package
```bash
cd ~/second_ros2_ws
colcon build --symlink-install --packages-select urdf_tutorial_r2d2
source install/setup.bash
```

# Run
```bash
ros2 launch urdf_tutorial_r2d2 demo_launch.py
# in a new terminal:
rviz2 -d ~/second_ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
```