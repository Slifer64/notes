
## Start Node with delay

```xml
<arg name="node_start_delay" default="0.5" />  
<node name="my_node_name" pkg="my_package" type="my_node"
    launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " />
```
Essentially, `bash -c` will start a bash process that executes the command(s) in `'...'`, where first we have a `sleep $(arg node_start_delay)` command, followed by the program name, stored in `$0` and all subsequent arguments, which are stored in `$@`.

## Pub jointstate message from terminal

```xml
<node name="init_state_publisher" pkg="rostopic" type="rostopic" 
    args="pub -r 10 /joint_states sensor_msgs/JointState -f '$(find welding_controller)/config/init_joint_state.yaml'" 
    output="screen" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " />
```
It is good to add a delay, so that other nodes like rviz, start first, otherwise the message might be lost.