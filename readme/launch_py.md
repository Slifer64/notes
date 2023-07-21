# Launch

## Usesful includes

```python
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition 
from launch_ros.parameter_descriptions import ParameterValue 
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, AndSubstitution, OrSubstitution, NotSubstitution 
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():
    return LaunchDescription([
        # ...
    ])  

```

## Declare Arguments

```python
# argument with choices
sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

default_msg_value = 'hell@ w@rld!'.replace('@','o')
msg_arg = DeclareLaunchArgument(name='model', default_value=str(default_msg_value), description='A welcome message...')


return LaunchDescription([
        sim_arg,
        msg_arg,
        # ...
    ])                                 
```

- View arguments of a launch file with `--show-args`:
    ```bash
    ros2 launch launch_tutorial example_substitutions.launch.py --show-args
    ```

- You can also pass directly values for arguments that are declared in included launch files.

---

## Package relative path
```python
my_pkg_path = get_package_share_path(package_name)
config_file = os.path.join(my_pkg_path, 'config/params.yaml')
```

---

## Load robot description

```python
xacro_file = os.path.join(get_package_share_path(package_name), 'urdf/robot_urdf.xacro')

robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

## Creating a parameter is optional. You can instead do:
# robot_description = Command(['xacro ', xacro_file])
## to store the result, and then use directly `robot_description`, 
## which will be automatically substitured as string when used.

## or load it directly without creating a ros-param
## Downside: you cannot pass arguments with LaunchConfiguration
# import xacro
# robot_description = xacro.process_file(xacro_file).toxml()


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

We can also pass arguments to the robot_description:
```python

my_arg1 = LaunchConfiguration('ros2_control')
my_arg2 = LaunchConfiguration('use_sim_time')

robot_description = Command(['xacro ', xacro_file, ' ros2_control:=', my_arg1, ' use_sim_time:=', my_arg2])
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

extra_params_file = os.path.join(get_package_share_directory('package_name'),'config','my_params.yaml')

spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        parameters=[{
            'param1': 'value1',
            'param2': 'value2'
        },
        extra_params_file]
    )
```

where `my_params.yaml` can look something like:
```yaml
spawn_entity:
    ros__parameters:
        some_int: 42
        a_string: "Hello world"
        some_lists:
            some_integers: [1, 2, 3, 4]
            some_doubles : [3.14, 2.718]
```

Command-line equivalent:
```bash
ros2 run gazebo_ros spawn_entity -topic robot_description -entity my_bot --ros-args -p param1:=value1 -p param2:=value2 --params-file $(ros2 pkg prefix --share package_name)/config/my_params.yaml
```

---

## `rviz`

```python
default_rviz_config_path = os.path.join(get_package_share_path('package_name'),'rviz/urdf.rviz')
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

gazebo_nodes = GroupAction(
        actions=[gazebo, spawn_entity],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

return LaunchDescription([
    # ... ,
    gazebo_nodes
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

## Launch-prefix
```python
launch_ros.actions.Node(
    package='teleop_twist_keyboard',
    node_executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -fa monaco -fs 15 -e',
    node_name='teleop')
```

`xml` equivalent:
```xml
<node pkg="teleop_twist_keyboard" exec="teleop_twist_keyboard" name="teleop" output="screen" launch-prefix="xterm -fa monaco -fs 15 -e" />
```

--- 

## `gdb`
Add the launch prefix:
```xml
xterm -fa monaco -fs 13 -e gdb -args
```

---

## Execute process - Python expression

```python

turtlesim_ns = LaunchConfiguration('turtlesim_ns')
use_provided_red = LaunchConfiguration('use_provided_red')
new_background_r = LaunchConfiguration('new_background_r')

turtlesim_ns_launch_arg = DeclareLaunchArgument(
    'turtlesim_ns',
    default_value='turtlesim1'
)

use_provided_red_launch_arg = DeclareLaunchArgument(
    'use_provided_red',
    default_value='False'
)
new_background_r_launch_arg = DeclareLaunchArgument(
    'new_background_r',
    default_value='200'
)

spawn_turtle = ExecuteProcess(
    cmd=[[
        'ros2 service call ',
        turtlesim_ns,
        '/spawn ',
        'turtlesim/srv/Spawn ',
        '"{x: 2, y: 2, theta: 0.2}"'
    ]],
    shell=True
)

change_background_r_conditioned = ExecuteProcess(
    condition=IfCondition( # condition is optional
        PythonExpression([
            new_background_r,
            ' == 200',
            ' and ',
            use_provided_red
        ])
    ),
    cmd=[[ # the cmd to execute
        'ros2 param set ',
        turtlesim_ns,
        '/sim background_r ',
        new_background_r
    ]],
    shell=True
)
```

Usage:
```bash
ros2 launch <package_name> <launch_filename>.py use_provided_red:='True' new_background_r:=200
```
Notice we set `use_provided_red:='True'`, as this will be evaluated inside a python expression.

---

## Timer - delay
```python
node1 = Node(
    package='package_name',
    executable='executable_name',
)

delayed_node1 = TimerAction(
            period=2.0,
            actions=[node1],
        )

node2 = Node(
    package='package_name',
    executable='executable_name',
)

delayed_node2 = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=node1,
        on_start=[node2],
    )
)

return LaunchDescription([
        delayed_node1,
        delayed_node2,
        # ...
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
