# Contents
- [basic](#basic)
- [remapping](#remapping)
- [pass params to node](#pass-parameters-to-a-node)
	- [command line](#command-line)
	- [yaml](#yaml)
	- [mixed (cmd + yaml)](#mixed-command-line-and-yaml)
- [topics](#ros2-topics)
- [services](#ros2-services)
- [parameters](#ros2-parameters)
- [actions](#ros2-actions)
- [logs](#logs--rqt_console)
- [launch](#launch)
- [yaml](#yaml-1)
- [ros2 bag](#ros2-bag)
- [colcon build](#colcon-build)
- [create package](#create-package)
	- [c++](#cpp_pkg)
		- [create](#cpp_pkg_create)
		- [add executables/libraries](#cpp_pkg_add_exec_lib)
		- [install](#cpp_pkg_install)
		- [build \& source](#cpp_pkg_build_source)
	- [python](#py_pkg)
		- [create](#py_pkg_create)
		- [add executables](#py_pkg_add_exec)
		- [add launch, config files etc.](#py_pkg_add_launch_config)
		- [build \& source](#py_pkg_build_source)
  - [c++ \& python](#cpp_py_pkg)
- [install workspace package dependencies](#install-workspace-package-dependencies)
- [Interfaces](#interfaces)
	- [message](#message-examples)
	- [service](#service-example)
	- [action](#action-example)
- [TF2](#tf2)
	- [view frames](#view-frames)
	- [tf2_echo](#tf2_echo)
	- [view in rviz](#view-in-rviz)
- [misc](#misc)
	- [find package path](#find-package-path)
	- [launch with rel package path](#launch-with-rel-package-path)
  - [Migration guide from ROS 1](#migration-guide-from-ros-1)


# Basic

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

ros2 pkg create

colcon_cd some_ros_package
```

Exclude a package from build `COLCON_IGNORE`

If you want to avoid configuring and building tests in CMake packages you can pass: `--cmake-args -DBUILD_TESTING=0`

```bash
printenv | grep -i ROS

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

ros2 pkg executables turtlesim

ros2 run turtlesim turtlesim_node

ros2 node list
ros2 topic list
ros2 service list
ros2 action list


ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
# return a list of subscribers, publishers, services, and actions
ros2 node info /my_turtle
```
---

# **Remapping**

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle # or -r __node:=my_turtle
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
---

# Pass parameters to a node 

- ## **command line**:
  ```bash
  ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"
  ```

- ## **yaml**:
  ```bash
  ros2 run demo_nodes_cpp parameter_blackboard --ros-args --params-file $(ros2 pkg prefix --share my_package)/config/my_params.yaml
  ```

  where `my_params.yaml` is:
  ```yaml
  parameter_blackboard:
      ros__parameters:
          some_int: 42
          a_string: "Hello world"
          some_lists:
              some_integers: [1, 2, 3, 4]
              some_doubles : [3.14, 2.718]
  ```

- ## **Mixed** (command line and yaml):
  ```bash
  ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p my_param1:=value1 -p my_param2:=value2 --params-file my_file.yaml
  ```

---

# ros2 **topics**

```bash
ros2 topic list
# return the same list of topics, with the topic type appended
ros2 topic list -t 

ros2 topic echo /turtle1/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist # see msg structure
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" # publish once
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" # publish with rate 1 Hz

ros2 topic hz /turtle1/pose # view the rate at which data is published
```
---

# ros2 **services**

```bash
# launch some nodes for practice
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 service list
ros2 service list -t # shows also the type of each service
ros2 service type <service_name> # show type for particular service
# e.g. std_srvs/srv/Empty
# The Empty type means the service call sends no data when making a request and receives no data when receiving a response.

ros2 service find <type_name> # find all the services of a specific type
# e.g.
ros2 service find std_srvs/srv/Empty

ros2 interface show <type_name> # print the structure of the service msg
# e.g.
ros2 interface show turtlesim/srv/Spawn

ros2 service call <service_name> <service_type> <arguments>
# e.g.
ros2 service call /clear std_srvs/srv/Empty
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
---

# ros2 **parameters**
```bash
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
ros2 param dump <node_name> > params.yaml
ros2 param load <node_name> <parameter_file> # load parameters from a file to a currently running node.
# Note that read-only parameters can only be modified at startup and not afterwards!

# load params on node startup
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```
---

# ros2 **actions**
```bash
ros2 action list
ros2 action list -t
ros2 action info /turtle1/rotate_absolute
ros2 interface show turtlesim/action/RotateAbsolute
# ros2 action send_goal <action_name> <action_type> <values>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
# to also view the feedback:
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```
---

# Logs (+ rqt_console)
```bash
ros2 run rqt_console rqt_console
```

ROS 2’s logger levels are ordered by severity:
```bash
Fatal # the system is going to terminate
Error # significant issues preventing the system from functioning properly
Warn # unexpected activity or non-ideal results
Info # (default) verification that the system is running as expected
Debug # step-by-step details of the system execution
```

```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN # messages of lower priority are not displayed
```
---

# Launch
```bash
ros2 launch turtlesim multisim.launch.py

ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

For writing launch files see:
- https://docs.ros.org/en/iron/How-To-Guides/Launch-files-migration-guide.html
- https://docs.ros.org/en/iron/How-To-Guides/Launch-file-different-formats.html

---

# YAML
ROS1 yaml file:
```yaml
# for node named '/lidar_ns/lidar_node_name'
lidar_name: foo
lidar_id: 10
# for node named '/imu'
ports: [11312, 11311, 21311]
# for all nodes
debug: true
```

ROS2 yaml file:
```yaml
/lidar_ns:
  lidar_node_name:
  # use “ros__parameters” to signal start of parameters
    ros__parameters:
      lidar_name: foo
      id: 10
imu:
  ros__parameters:
    ports: [2438, 2439, 2440]
/**:
  ros__parameters:
    debug: true
```
---

# ros2 **bag**
```bash
mkdir bag_files
cd bag_files
# ros2 bag record <topic_name>
ros2 bag record /turtle1/cmd_vel
# record multiple topics with the name 'subset'
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

# ros2 bag info <bag_file_name>
ros2 bag info subset

# replay
ros2 bag play subset
```
---

# colcon build
```bash
git clone https://github.com/ros/ros_tutorials.git -b $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to turtlesim
```
Can also use:
- `--packages-up-to`: builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
- `--symlink-install`: saves you from having to rebuild every time you tweak python scripts

---

# create package

<h2 id="cpp_pkg">c++</h2>

- <h3 id="cpp_pkg_create">Create</h3>

  ```bash
  # --node-name: creates a simple hello world executable
  ros2 pkg create --build-type ament_cmake --node-name hello_world_node <packge_name> --dependencies rclcpp <other packages> --license Apache-2.0
  # or for library
  ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
  ```

  Package layout:
  ```bash
  <package_name>/
     CMakeLists.txt
     include/<package_name>/
     package.xml
     src/
  ```

- <h3 id="cpp_pkg_add_exec_lib">Add executables / libraries</h3>

  In the `CMakeLists.txt`
  ```cmake

  include_directories(
    include
    # other includes...
  )

  # executable
  add_executable(<exec_name> <src_file>.cpp)
  target_include_directories(<exec_name> PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(<exec_name> PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(<exec_name> rclcpp <other pkg dependencies>)

  # library
  add_library(<lib_name> <src_files>.cpp)
  target_compile_features(<lib_name> PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(<lib_name> PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(<lib_name> rclcpp <other pkg dependencies>)
  ```

- <h3 id="cpp_pkg_install">Install</h3>

  ```cmake
  # executables
  install(TARGETS
    <executable_1>
    <executable_2>
    ...
    DESTINATION lib/${PROJECT_NAME}
  )

  # library
  install(
    # DIRECTORY include/<lib_header_folder>
    DIRECTORY include/
    DESTINATION include
  )
  
  install(
    TARGETS <lib_name>
    # EXPORT <lib_name>
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    # INCLUDES DESTINATION include
  )
  ament_export_include_directories(
    include
  )
  ament_export_libraries(
    <lib_name>
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )

  # launch, config, rviz files etc.
  install(DIRECTORY
    launch
    config
    rviz
    urdf
    worlds
    DESTINATION share/${PROJECT_NAME}
  )

  # add this to make the library available as a ros package
  ament_package()
  ```

- <h3 id="cpp_pkg_build_source">Build & source</h3>

  ```bash
  colcon build --packages-select <package_name>
  source ./install/setup.bash
  ```


<h2 id="py_pkg">python</h2>

- <h3 id="py_pkg_create">Create</h3>

  ```bash
  ros2 pkg create --build-type ament_python --node-name hello_world_node <packge_name> --dependencies rclcpp <other packages> --license Apache-2.0
  ```

  Package layout:
  ```bash
  <package_name>/
      package.xml
      resource/<package_name>
      setup.cfg # required for package executables, so ros2 run can find them
      setup.py # instructions for how to install the package
      <package_name>/ # contains the python scripts
  ```

  Notice that the `setup.cfg` should automatically contain:
  ```bash
  [develop]
  script_dir=$base/lib/<package_name>
  [install]
  install_scripts=$base/lib/<package_name>
  ```

- <h3 id="py_pkg_add_exec">Add executables</h3>

  In the `setup.py`:
  ```python
  package_name = <package_name>

  setup(
    ...
    entry_points={
          'console_scripts': [
              '<exec_name> = package_name.<script_name>:main',
              ...
          ],
      },
  )
  ```

- <h3 id="py_pkg_add_launch_config">Add launch, config files etc.</h3>

  ```python
  package_name = <package_name>

  setup(
    ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include model and simulation files
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    ...
  )
  ```

- <h3 id="py_pkg_build_source">Build & source</h3>

  ```bash
  colcon build --symlink-install --packages-select <package_name>
  source ./install/setup.bash
  ```

<h2 id="cpp_py_pkg">c++ & python</h2>

- <h3 id="cpp_py_pkg_create">create c++ package</h3>

  See [create c++ package](#cpp_pkg_create).
  The package structure should look like this:
  ```
  my_cpp_py_pkg/
  ├── CMakeLists.txt
  ├── include
  │   └── my_cpp_py_pkg
  ├── package.xml
  └── src
  ```

- <h3 id="cpp_py_pkg_add_py">Add python folders & scripts</h3>

  Add the following for python:
  ```diff
   my_cpp_py_pkg/
   ├── CMakeLists.txt
   ├── include
   │   └── my_cpp_py_pkg
  +├── my_cpp_py_pkg/    # utility classes/functions etc.
  +|   ├── __init__.py
  +|   ├── my_module1.py
  +|   └── my_module2.py
   ├── package.xml
   ├── src
  +└── scripts/          # executable scripts/nodes
  +|   ├── my_exec_node1.py
  +|   └── my_exec_node2.py
  ```
  where
  - `my_module*.py` are optional modules (classes, utility functions etc)
  - `my_node*.py` are the ''executable'' nodes/scripts (i.e. contain, among other stuff, a `main` function).

  <mark>Important</mark>: Add `#!/usr/bin/env python3` at the top of all `*.py` executable scripts/nodes. This allows us to use these executables/scripts using `ros2 run` or in launch files.

  The python utility modules can be accessed by the executables scripts in `scripts/` or any other package that includes `my_cpp_py_pkg`, e.g.:
  ```python
  #!/usr/bin/env python3

  from my_cpp_py_pkg.my_module1.py import MyClass
  from my_cpp_py_pkg.my_module2.py import MyClass2, my_fun
  # ...
  def main():
    # ...

  if __name__ == '__main__':
    main()
  ```

- <h3 id="cpp_py_pkg_package_xml">Edit <code>package.xml</code></h3>

  ```diff
  <?xml version="1.0"?>
  <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
  <package format="3">
    <name>my_cpp_py_pkg</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="your@email.com">Name</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
  + <buildtool_depend>ament_cmake_python</buildtool_depend>

    <!-- cpp dependencies -->
    <depend>rclcpp</depend>
    <!-- <depend>...</depend> -->

  + <!-- python dependencies -->
  + <exec_depend>rclpy</exec_depend>
  + <!-- <exec_depend>...</exec_depend> -->

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
  ```
  Notice that instead of `ament_python`, we use `ament_cmake_python` to setup our python stuff with cmake from the `CMakeLists.txt` file.

- <h3 id="cpp_py_pkg_cmakelists">Edit <code>CMakeLists.txt</code></h3>

  Add the following to `CMakeLists.txt`:
  ```cmake
  # Install Python modules
  ament_python_install_package(${PROJECT_NAME})

  # Install Python executables

  # make python programs executable (required when building with --symlink-install)
  execute_process(
      COMMAND bash -c "chmod +x \
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/my_exec_node1.py \
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/my_exec_node2.py"
      RESULT_VARIABLE err_code
  )
  if(${err_code})
      message(FATAL_ERROR "Error making scripts executable...")
  endif()

  install(PROGRAMS
    scripts/my_exec_node1.py
    scripts/my_exec_node2.py
    DESTINATION lib/${PROJECT_NAME}
  )
  ```
  Notice that when using `--symlink-install`, the installed script, is a symbolik link to `scripts/my_exec_node*.py`, that's why we have to make the latter executable. 

  We can now run our python scripts/executables like:
  ```bash
  ros2 run my_cpp_py_pkg my_exec_node1.py
  ```
  or from a launch file:
  ```python
  my_node = Node(
    package='my_cpp_py_pkg',
    executable='my_exec_node1.py',
    name='my_exec_node1',
  )
  ```
  Notice that the executables have the suffix `.py`. If we want to run them directly without the `.py` suffix, we can rename them during the install step in the `CMakeLists.txt`:

  ```diff
  - install(PROGRAMS
  -   scripts/my_exec_node1.py
  -   scripts/my_exec_node2.py
  -   DESTINATION lib/${PROJECT_NAME}
  - )
  + install(PROGRAMS
  +   scripts/my_exec_node1.py
  +   DESTINATION lib/${PROJECT_NAME}
  +   RENAME my_exec_node1
  + )
  + install(PROGRAMS
  +   scripts/my_exec_node2.py
  +   DESTINATION lib/${PROJECT_NAME}
  +   RENAME my_exec_node2
  + )
  ```
  In this case however, the `--symlink-install` will not work, so if we change the scripts, we will have to `colcon build` again.

  <mark>Best solution</mark>: Remove the `.py` suffix from all executables in `scripts/`. Since `#!/usr/bin/env python3` is set at the top, they are already registered as python code. So in all cases above (`CMakeLists.txt` and during `ros2 run/launch`) we have <code>my_exec_node*~~.py~~<code>.


---

# install workspace package dependencies

```bash
rosdep install --from-paths src -y --ignore-src
```
- `--from-paths src`: the path to check for package.xml files to resolve keys for
- `-y`: yes to all prompts from the package manager to install without prompts
- `--ignore-src`: ignore installing dependencies, even if a rosdep key exists, if the package itself is also in the workspace.

---


# Interfaces

Built-in-types: `bool, byte, char, float32, float64, int8, int16, uint16, int32, uint32, int64, uint64, string, wstring`

Every built-in-type can be used to define arrays.

## Message Examples:
```bash
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

```bash
uint8 x 42
int16 y -2000
string full_name "John Doe" # string values must be defined in single ' or double " quotes
int32[] samples [-200, -100, 0, 100, 200]
```

**Constants**: Constants names have to be UPPERCASE!
```bash
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```

**Composite messages**:
```bash
int32 X=123
int8 foobar
another_pkg/AnotherMessage msg
CustomMessageDefinedInThisPackage value
```

## Service Example:
```bash
# request constants
int8 FOO=1
int8 BAR=2
# request fields
int8 foobar
another_pkg/AnotherMessage msg
---
# response constants
uint32 SECRET=123456
# response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

## Action Example:
```bash
int32 order
---
int32[] sequence
---
int32[] sequence
```

For more see https://docs.ros.org/en/iron/Concepts/Basic/About-Interfaces.html

---

# TF2

Prerequisites:
```bash
sudo apt-get install ros-$ROS_DISTRO-turtle-tf2-py ros-$ROS_DISTRO-tf2-tools ros-$ROS_DISTRO-tf-transformations
```

Run demo:
```bash
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py 
```
In a second terminal:
```bash
ros2 run turtlesim turtle_teleop_key
```
One turtle continuously moves to follow the turtle you are driving around.

## view frames
```bash
ros2 run tf2_tools view_frames
```

## tf2_echo
```bash
ros2 run tf2_ros tf2_echo [source_frame] [target_frame]
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

## view in rviz
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

---

# MISC

## Find package path
```bash
$(ros2 pkg prefix --share turtle_tf2_py)
# e.g.
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

## Launch with rel package path
```bash
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf
```

## Migration guide from ROS 1

https://docs.ros.org/en/iron/The-ROS2-Project/Contributing/Migration-Guide.html
