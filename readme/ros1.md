## Realsense camera

### Filters

Available filters:
- spatial
- temporal
- hole_filling
- decimation
- disparity

Add one or more of the above filters, separated by comma, e.g.
```xml
<arg name="camera" default="rs2" />

<include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="camera" value="$(arg camera)" /> <!-- prefix of all published topics -->

    <arg name="publish_tf" value="false"/>

    <arg name="enable_depth" value="true" />
    <arg name="align_depth" value="true" />
    <arg name="enable_sync" value="true" />
    <arg name="enable_pointcloud" value="true" />
    
    <arg name="color_width" value="640"/>
    <arg name="color_height" value="480"/>
    <arg name="color_fps" value="30"/>

    <arg name="depth_width" value="640"/>
    <arg name="depth_height" value="480"/>
    <arg name="depth_fps" value="30"/>

    <arg name="filters" value="spatial,temporal"/>
</include>
```

To set the parameters filters, either:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
Or, e.g. for the `temporal` filter:
```xml
<node name="config_temporal_filter" pkg="dynamic_reconfigure" type="dynparam" args="set_from_parameters $(arg camera)/temporal">

    <param name="filter_smooth_delta" value="50" />
    <param name="filter_smooth_alpha" value="0.1"/>
    <param name="holes_fill"          value="3"  />

</node>
```
Notice, to get a list of all the reconfigurable nodes:
```bash
rosrun dynamic_reconfigure dynparam list
```
and to get a list of available parameters for the temporal filter:
```bash
rosrun dynamic_reconfigure dynparam get /camera/temporal
```
