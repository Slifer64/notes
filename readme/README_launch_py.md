# Launch

## Usesful includes

```python
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory  # get pkg path
from launch_ros.actions import Node # create ros nodes
from launch import LaunchDescription # generate the final launch description
from launch.conditions import IfCondition, UnlessCondition  # conditional launch node
from launch.substitutions import Command # execute a bash command
from launch_ros.parameter_descriptions import ParameterValue # define a param
from launch.actions import DeclareLaunchArgument # define a launch argument
from launch.substitutions import LaunchConfiguration # retrieve the value of an argument
from launch.substitutions import AndSubstitution, OrSubstitution, NotSubstitution # for composite conditions
from launch.actions import IncludeLaunchDescription # include another launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource # wrapper for the path to `IncludeLaunchDescription`

def generate_launch_description():
    return LaunchDescription([
        # ...
    ])  

```

## Declare Arguments

```python
# argument with choices
sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')

default_msg_value = 'hell@ w@rld!'.replace('@','o')
msg_arg = DeclareLaunchArgument(name='model', default_value=str(default_msg_value),
                                    description='A welcome message...')


return LaunchDescription([
        sim_arg,
        msg_arg,
        # ...
    ])                                 
```

---

## Package relative path
```python
my_pkg_path = get_package_share_path('my_package')
config_file = my_pkg_path / 'config/params.yaml'
```

---

## Load robot description

```python
robot_urdf_xacro = str(get_package_share_path('my_package') / 'urdf/robot_urdf.xacro')

robot_description = ParameterValue(Command(['xacro ', robot_urdf_xacro]), value_type=str)

# example use case
robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

return LaunchDescription([
        robot_state_publisher_node,
        # ...
    ])   
```

---

## Launch `robot/joint_state_publisher`

```python
robot_description = ParameterValue(Command(['xacro ', ...]), value_type=str)

sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description, 
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }],
    # remappings=[
    #     ('/joint_states', '/my_joint_states'),
    #     ('robot_description', '/my_robot_description')
    # ]
)

joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    # remappings=[
    #     ('/joint_states', '/my_joint_states'),
    #     ('robot_description', '/my_robot_description')
    # ]
    
)
## or to use gui sliders:
# joint_state_publisher_node = Node(
#     package='joint_state_publisher_gui',
#     executable='joint_state_publisher_gui',
# )

return LaunchDescription([
    sim_time_arg,
    robot_state_publisher_node,
    joint_state_publisher_node,
    # ...
])
```

---

## Pass arguments/parameters to node

```python
# ros2 run gazebo_ros spawn_entity -topic robot_description -entity my_bot
spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        parameters=[{
            'param1': 'value1',
            'param2': 'value2'
        }]
    )
```

---

## `rviz`

```python
default_rviz_config_path = str(get_package_share_path('package_name') / 'rviz/urdf.rviz')
rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')

rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

return LaunchDescription([
    rviz_arg,
    rviz_node,
    # ...
])
```

---

## Conditions

- ### `IfCondition` - `UnlessCondition`

```python
gui_arg = DeclareLaunchArgument(name='gui', default_value='true', 
    choices=['true', 'false'], 
    description='Flag to enable joint_state_publisher_gui')

sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false', 
    choices=['true', 'false'], 
    description='Set to true to use sim (Gazebo) clock')

# Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    condition=UnlessCondition(LaunchConfiguration('gui'))
)

joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    condition=IfCondition(LaunchConfiguration('gui'))
)

gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
    )]),
    condition=IfCondition(LaunchConfiguration('use_sim_time'))
  )


return LaunchDescription([
    joint_state_publisher_node,
    joint_state_publisher_gui_node,
    # ...
])
```

- ### Composite conditions (`And/Or/NotSubstitution`)

```python

pub_joints_arg = DeclareLaunchArgument(name='pub_joints', default_value='true', choices=['true', 'false'], description='...')
gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='...')

joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    condition=IfCondition(AndSubstitution(LaunchConfiguration("pub_joints"), 
                                          NotSubstitution(LaunchConfiguration("gui"))))
)

joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    condition=IfCondition(AndSubstitution(LaunchConfiguration("pub_joints"), 
                                          LaunchConfiguration("gui"))
)

return LaunchDescription([
    pub_joints_arg,
    gui_arg,
    joint_state_publisher_node,
    joint_state_publisher_gui_node,
    # ...
])
```

- ### Group conditions
```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
    )]),
    # condition=IfCondition(LaunchConfiguration('use_sim_time'))
  )

spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'my_bot'
    ],
    # condition=IfCondition(LaunchConfiguration('use_sim_time'))
  )

gazebo_action = GroupAction(
        actions=[gazebo, spawn_entity],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

return LaunchDescription([
    # ... ,
    gazebo_action
  ])  

```

---

## Include other launch files

```python
other_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','other_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
#...
return LaunchDescription([
        other_launch_file,
        #...
    ])
```

---

## Misc

### `LaunchConfiguration ` vs `DeclareLaunchArgument `

`DeclareLaunchArgument` exposes the argument outside of your launch file, allowing it to be listed or set from the command line (using `ros2 launch`) or when including another launch file (using `IncludeLaunchDescription`).

`LaunchConfiguration` is local to the launch file and scoped.
`LaunchConfiguration` can also be used to retrieve the value of a launch argument or a launch configuration variable.

Example:

```python
import launch

from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    msg_arg = DeclareLaunchArgument('msg', default_value='hello world')

    other_arg = DeclareLaunchArgument('other')

    return launch.LaunchDescription([
        msg_arg,
        other_arg,
        LogInfo(msg=LaunchConfiguration('msg')),
        LogInfo(msg=LaunchConfiguration('other')),
    ])
```

### Alternative way to write launch file

```python
launch_descr = LaunchDescription()
# ...
launch_descr.add_action(my_arg)
launch_descr.add_action(my_node)
launch_descr.add_action(my_launch)
launch_descr.add_action(...)

return launch_descr
```


---
