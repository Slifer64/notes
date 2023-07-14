# Calibration Package for Multiple Cameras


First implementation of the camera-robot calibration system.

## How to Run 
To .launch files have been implemented: 

Run `roslaunch camera_robot_calibration rs_camera_rviz.launch` to launch the camera and rviz 

Then, proceed running `roslaunch camera_robot_calibration calibration.launch` to launch the calibrator 

## Dependencies:
### Python dependencies: 
* Python 2
* pyrealsense
* dt_apriltags pip install dt_april_tags
* numpy
* opencv 
* _TODO_
### ROS Deps
* ros-realsense
