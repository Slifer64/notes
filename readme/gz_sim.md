# Contents

---

# Install

From https://gazebosim.org/api/sim/8/install.html:
```bash
# Install tools
sudo apt install -y build-essential cmake git gnupg lsb-release wget
# Enable the Gazebo software repositories:
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
# Clone repository
git clone https://github.com/gazebosim/gz-sim -b gz-sim<#>
# Install package dependencies (including other Gazebo libraries):
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | tr '\n' ' '))
# Configure and build.
cd gz-sim
mkdir build
cd build
cmake ../
make
sudo make install
```
```bash
sudo apt-get update && \
sudo apt-get install -y \
    libgz-launch7
```


# Run
```bash
gz sim <srd_file> -r --gui-config <my_gui.config> -v 4
```
- `<srd_file>` with entities (models, links, visual as in urdf) and plugins.
- `-r` automatically starts the simulation (no need to press start manually).
- `-v` verbosity level (4 == debug).

# Customize

Edit `~/.gz/sim/<#>/gui.config` and/or `~/.gz/sim/<#>/server.config` to modify settings, enable/disable plugins or set custom plugins. \
E.g. to change the window theme and size, edit `~/.gz/sim/<#>/gui.config`:
```xml
<window>
    <width>1100</width>
    <height>845</height>
    <style material_theme="Dark" material_primary="DeepOrange" material_accent="LightBlue" toolbar_color_light="#f3f3f3" toolbar_text_color_light="#111111" toolbar_color_dark="#414141" toolbar_text_color_dark="#f3f3f3" plugin_toolbar_color_light="#bbdefb" plugin_toolbar_text_color_light="#111111" plugin_toolbar_color_dark="#607d8b" plugin_toolbar_text_color_dark="#eeeeee"/>
    <menus>
        <drawer default="false"/>
    </menus>
    <dialog_on_exit>true</dialog_on_exit>
</window>
```


# CLI
```bash
gz <tab>
gz <option> --help

gz model --list
gz model -m <model_name>
gz model -m <model_name> --link <link_name>
gz model -m <model_name> --joint <joint_name>

gz topic -l
gz topic -it "/model/elevator/cmd"  
gz topic -t "/model/elevator/cmd" -m gz.msgs.Int32 -p "data: 2"

gz topic -t "/model/vehicle_blue/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.1}"

gz service -s /world/shapes/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 --req 'name: "box", position: {z: 5.0}'
```

# c++ code

- Get a model's attribute (component):
```c++
Entity entity_id; // e.g. joint, link, model etc
auto c = _ecm.Component<components::Pose>(entity_id);
// components::<type>, e.g. Name, ChildLinkName, ParentLinkName, JointPosition, JointVelocity, Pose etc.
c->Data(); // to get the undelying data
```

- Set a model's attribute (component):
```c++
msgs::Set(jointMsg->mutable_pose(), pose->Data());
```

- Inertia component:
```cpp
// inertial
#include "gz/sim/components/Inertial.hh"

auto inertia_comp = _ecm.Component<components::Inertial>(_entity);
if (inertia_comp)
{
    math::Inertiald inertial = inertia_comp->Data();
    sdf::ElementPtr inertialElem = _elem->GetElement("inertial");
    inertialElem->GetElement("pose")->Set<math::Pose3d>(inertial.Pose());
    const math::MassMatrix3d &massMatrix = inertial.MassMatrix();
    inertialElem->GetElement("mass")->Set<double>(massMatrix.Mass());
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    inertiaElem->GetElement("ixx")->Set<double>(massMatrix.Ixx());
    inertiaElem->GetElement("ixy")->Set<double>(massMatrix.Ixy());
    inertiaElem->GetElement("ixz")->Set<double>(massMatrix.Ixz());
    inertiaElem->GetElement("iyy")->Set<double>(massMatrix.Iyy());
    inertiaElem->GetElement("iyz")->Set<double>(massMatrix.Iyz());
    inertiaElem->GetElement("izz")->Set<double>(massMatrix.Izz());
    }
```

- Get any component:
```cpp
#include "gz/sim/components/${Component}.hh"

auto comp = _ecm.Component<components::${Component}>(_entity);
if (!comp) // The component doesn't exist!
```

- Create components:
```cpp
Entity _joint;
// Create joint position component if one doesn't exist
if (!_ecm.EntityHasComponentType(_joint,
    components::JointPosition().TypeId()))
// or" 
// if (_ecm.Component<components::JointPosition>(_joint) == nullptr)
{
    double init_jpos = 0;
    _ecm.CreateComponent(_joint, components::JointPosition({init_jpos}));
}


// Create joint velocity component if one doesn't exist
if (!_ecm.EntityHasComponentType(_joint,
    components::JointVelocity().TypeId()))
{
    _ecm.CreateComponent(_joint, components::JointVelocity());
}

// Create joint force component if one doesn't exist
if (!_ecm.EntityHasComponentType(_joint, components::JointForce().TypeId()))
{
    _ecm.CreateComponent(_joint, components::JointForce());
}
```

- Create joints for a model
```cpp
std::vector<Entity> childJoints = 
    _ecm.ChildrenByComponents(this->model.Entity(), components::Joint());
for (const Entity &_joint : childJoints)
{
    // Create joint position component if one doesn't exist
    if (!_ecm.Component<components::JointPosition>(_joint))
        _ecm.CreateComponent(joint, components::JointPosition());

    // Create joint velocity component if one doesn't exist
    if (!_ecm.Component<components::JointVelocity>(_joint))
        _ecm.CreateComponent(_joint, components::JointVelocity());

    // Create joint force component if one doesn't exist
    if (!_ecm.Component<components::JointForce>(_joint))
        _ecm.CreateComponent(_joint, components::JointForce());
}
```

# Fixed models

## Robot with fixed base
Only the base link will be fixed. All other links are free to move (according to the joints).
```xml
<model name='rrbot'>
    <!-- <static>true</static> -->
    <joint name='word_rrbot_fixed_joint' type='fixed'>
        <parent>world</parent>
        <child>link1</child>
    </joint>
    <link name='link1'>
    ...
    </link>
    ...
</model>
```

## Fixed-static model
The entire model will be immovable.
```xml
<model name='rrbot'>
    <static>true</static>
    <joint name='word_rrbot_fixed_joint' type='fixed'>
        <parent>world</parent>
        <child>link1</child>
    </joint>
    <link name='link1'>
    ...
    </link>
    ...
</model>
```

# Camera - Images

```xml
<model name="box">
    <!-- <pose>...</pose> -->
    <!-- <link name="box_link"> -->
    <!-- <inertial>...</inertial> -->
    <!-- <collision name="box_collision">...</collision> -->

    <visual name="box_visual">
        <!-- <geometry>...</geometry> -->
        <!-- <material>...</material> -->

        <!-- Label the item to be able to retrieve its bounding box -->
        <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
            <label>10</label>
        </plugin>
    </visual>
    </link>
</model>
```

```xml
<gui>
    <plugin filename="ImageDisplay" name="Full 2D">
    <topic>boxes_full_2d_image</topic>
    <gz-gui>
        <title>Full 2D</title>
        <property key="state" type="string">docked</property>
        <property type="double" key="height">400</property>
        <property type="double" key="width">600</property>
    </gz-gui>
    </plugin>

    <plugin filename="ImageDisplay" name="Visible 2D">
    <topic>boxes_visible_2d_image</topic>
    <gz-gui>
        <title>Visible 2D</title>
        <property key="state" type="string">docked</property>
        <property type="double" key="height">400</property>
        <property type="double" key="width">600</property>
    </gz-gui>
    </plugin>

    <plugin filename="ImageDisplay" name="3D">
    <topic>boxes_3d_image</topic>
    <gz-gui>
        <title>3D</title>
        <property key="state" type="string">docked</property>
        <property type="double" key="height">400</property>
        <property type="double" key="width">600</property>
    </gz-gui>
    </plugin>
</gui>
```

```xml
<model name="boundingbox_camera">
    <!-- links, collision, visual etc -->

    <sensor name="full_2d" type="boundingbox_camera">
        <topic>boxes_full_2d</topic>
        <camera>
        <box_type>full_2d</box_type>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>800</width>
            <height>600</height>
        </image>
        <clip>
            <near>0.1</near>
            <far>10</far>
        </clip>
        <!-- save the image and bounding box points -->
        <save enabled="true">
            <path>bounding_box_full_2d_data</path>
        </save>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
    </sensor>

    <sensor name="visible_2d" type="boundingbox_camera">
        <topic>boxes_visible_2d</topic>
        <camera>
        <box_type>visible_2d</box_type>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>800</width>
            <height>600</height>
        </image>
        <clip>
            <near>0.1</near>
            <far>10</far>
        </clip>
        <save enabled="true">
            <path>bounding_box_visible_2d_data</path>
        </save>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
    </sensor>

    <sensor name="3d" type="boundingbox_camera">
        <topic>boxes_3d</topic>
        <camera>
        <box_type>3d</box_type>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>800</width>
            <height>600</height>
        </image>
        <clip>
            <near>0.1</near>
            <far>10</far>
        </clip>
        <save enabled="true">
            <path>bounding_box_3d_data</path>
        </save>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
    </sensor>
    </link>
</model>
```

```bash
gz topic -et /boxes_3d --json-output
```

# Reset simulation
```bash
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
```

# Plugins

There are a few places where the plugins can be defined:

- `<plugin>` elements inside an SDF file.
- File path defined by the `GZ_SIM_SERVER_CONFIG_PATH` environment variable.
- The default configuration file at `$HOME/.gz/sim/<#>/server.config`, where `<#>` is Gazebo Sim's major version.

Each of the items above takes precedence over the ones below it. For example, if the SDF file has any `<plugin>` elements, then the `GZ_SIM_SERVER_CONFIG_PATH` variable is ignored. And the default configuration file is only loaded if no plugins are passed through the SDF file or the `GZ_SIM_SERVER_CONFIG_PATH` environment variable.

## Basic plugins

```xml
<!-- simulates the entities in the world -->
<plugin entity_name="*"
            entity_type="world"
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
</plugin>

<!-- allows the user to add entities using the gui -->
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands">
</plugin>

<!-- updates the visual of the entities in the world -->
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster">
</plugin>

<!-- or rendering-based sensors to generate data -->
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
    <render_engine>ogre</render_engine>
</plugin>
```

## Plugin's search path
Gazebo will look for system plugins on the following paths, in order:
- All paths on the `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable
- `$HOME/.gz/sim/plugins`
- Systems that are installed with Gazebo


# Sensors

```xml
<!-- make sure the sensors plugin is loaded -->
<plugin entity_name="*"
        entity_type="world"
        filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
    <render_engine>ogre</render_engine>
</plugin>

<!-- add model -->
<model name="camera">
    <static>true</static>
    <pose>20 0 1.0 0 0.0 3.14</pose>
    <link name="link">
    <pose>0.05 0.05 0.05 0 0 0</pose>
    <visual name="visual">
        <geometry>
        <box>
            <size>0.1 0.1 0.1</size>
        </box>
        </geometry>
    </visual>
    <!-- add sensor -->
    <sensor name="camera" type="camera">
        <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>320</width>
            <height>240</height>
        </image>
        </camera>
        <update_rate>30</update_rate>
    </sensor>
    </link>
</model>
```

# Resources search path
Gazebo will look for URIs (path / URL) in the following, in order:
- All paths on the `GZ_SIM_RESOURCE_PATH` environment variable (if path is URI, scheme is stripped)
- Current running path / absolute path
- Gazebo Fuel
    - Cache (i.e. `$HOME/.gz/fuel`)
    - Web server

# Move Camera to model
Load the View Angle plugin, either from the drop down on the top right, or:
```xml
<gui>
    <plugin filename="ViewAngle">
    </plugin>
    <!-- ... -->
</gui>
```

E.g. move the camera to the box model looking down from 5 meters away.
```bash
gz service  -s /gui/move_to/model --reqtype gz.msgs.GUICamera  --reptype gz.msgs.Boolean -r 'name: "world", pose: {position: {z:5}, orientation: {x:0, y:0, z: -1, w:0}}' --timeout 3000

# Get the service info
gz service  -is /gui/view/collisions
# Get the request and reply messages info
gz msg -i gz.msgs.Entity
# call the service
gz service -s /gui/view/collisions --reqtype gz.msgs.StringMsg  --reptype gz.msgs.Boolean -r 'data: "<model_name>"' --timeout 3000


gz service -s /gui/view/collisions --reqtype gz.msgs.StringMsg  --reptype gz.msgs.Boolean -r 'data: "<model_name>"' --timeout 3000

## Create/Spawn
gz service -is /world/empty/create
# gz msg -i gz.msgs.EntityFactory
gz service -s /world/empty/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    -r 'name: "item1", sdf_filename: "/models/item.sdf", pose: {position: {x: 0.5, z: 1.0}, orientation: {w: 1.0}}, allow_renaming: true, relative_to: "gripper"'
# `relative_to` doesn't seem to work, but anyway...

## Delete/Remove
gz service -is /world/empty/remove
# gz msg -i gz.msgs.Entity
gz service -s /world/empty/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    -r 'id: 37'
    # -r 'name: "model_name", type: 2'
# Entity::Type {NONE = 0; LIGHT = 1; MODEL = 2; LINK = 3; VISUAL = 4; COLLISION = 5; SENSOR = 6; JOINT = 7; ACTOR = 8; WORLD = 9;

## Set Pose (move an object)
gz service -s /world/empty/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    -r 'name: "item", position: {x: 2.5, y: -2.3 z: 1.5}, orientation: {w: 1.0}'

## Set Physics
gz service -is /world/empty/set_physics

## Add System plugin
gz service -is /world/empty/entity/system/add

## List of worlds
gz service -s /gazebo/worlds --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V -r 'unused: true' --timeout 3000
```

# Drag objects with the mouse

```xml
<plugin filename="MouseDrag">
  <rotation_stiffness>100.0</rotation_stiffness>
  <position_stiffness>100.0</position_stiffness>
  <gz-gui>
    <property key="width" type="double">400</property>
    <property key="height" type="double">300</property>
    <property key="opacity" type="double">1</property>
    <property key="enabled" type="bool">true</property>
    <!-- other properties -->
  </gz-gui>
</plugin>
```

# Useful links

- http://sdformat.org/spec?elem=physics&ver=1.11 : Physics settings
- https://gazebosim.org/api/sim/8/tutorials.html : Tutorials
- https://gazebosim.org/libs : Inspect the github source code and examine examples
- https://dartsim.github.io/ : DART
- https://www.openrobotics.org/blog/2023/9/26/gazebo-harmonic-released

# Auto-inertia calculation
```xml
<model name="cylinder">
  <link name="cylinder_link">
    <!-- Overrides any present <mass>, <pose> and <inertia> tags!!! -->
    <inertial auto="true" />
    <!-- the constituent collision geometries of the link are considered for the calculations. -->
    <collision name="collision_1">
        <!-- set the density to dermine the mass based on the volume. (default = 1000 kg/m^3) -->
        <density>1240.0</density> <!-- kg/m^3 -->
        <geometry>...</geometry>
    </collision>
    <!-- multiple collisions with different densities can also be set -->
    <collision name="collision_2">
        <density>2360.0</density> <!-- kg/m^3 -->
        <geometry>...</geometry>
    </collision>
    <visual name="visual">...</visual>
  </link>
</model>
```
For more see: https://gazebosim.org/api/sim/8/auto_inertia_calculation.html 


# Convert xacro to sdf
Notice that any gazebo specific stuff that are not recognized by `xacro` should be enclosed in `<gazebo>...</gazebo>`, otherwise they are discarded!
```bash
xacro <path_to_xacro_file> <args_to_xacro_file> > robot.urdf && \
gz sdf -p robot.urdf > robot.sdf && \
rm -rf robot.urdf
```

# Spawn model
Substitute `<world_name>`

Clone existing model (set `<model_name>`):
```bash
gz service \
    -s /world/<world_name>/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req 'clone_name: "<model_name>", pose:{ position: {x: 0.1, y: 0.1, z: 0.1} }, name: "item3", allow_renaming: true'
```

Load model from `sdf file`:
```bash
gz service \
    -s /world/<world_name>/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req 'sdf_filename: "<path_to_sdf_file>", pose:{ position: {x: 0.1, y: 0.1, z: 0.1} }, name: "item3", allow_renaming: true'
```

example sdf file:
```xml
<?xml version="1.0" ?>
<sdf version="1.8">

  <!-- item -->
  <model name='item'>
    <link name="item_link">
      <inertial auto="true" />
      <collision name="item_collision">
        <density>1000.0</density>
        <geometry>...</geometry>
      </collision>
      <visual name="item_visual">
        <geometry>...</geometry>
        <material>...</material>
      </visual>
    </link>
  </model>

</sdf>
```

# Plugins

## Triggered publisher
Example:
```xml
<!-- Moving Forward: W -->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">87</match>
    </input>
    <output type="gz.msgs.Double" topic="/model/conveyor/link/base_link/track_cmd_vel">
        data: 1.0
    </output>
</plugin>
```
Note that you have to add the `Key Publisher` either by hand or specify it in:
```xml
<gui>
    <!-- other gui elements -->

    <plugin filename="KeyPublisher" name="Key publisher">
        <gz-gui>
            <anchors target="3D View">
                <line own="right" target="right"/>
                <line own="top" target="top"/>
            </anchors>
            <property key="resizable" type="bool">false</property>
            <property key="width" type="double">5</property>
            <property key="height" type="double">5</property>
            <property key="state" type="string">floating</property>
            <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
    </plugin>
</gui>
```
This captures the keystrokes and publishes them to the topic `/keyboard/keypress`.

## Parse parameters

Example sdf:
```xml
<plugin>
    <input type="gz.msgs.Empty" topic="/reset_robot"/>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0}
    </output>
</plugin>

<plugin>
    <input type="gz.msgs.Vector2d" topic="/input_topic">
        <match field="x">1.0</match>
        <match field="y">2.0</match>
    </input>
    <output type="gz.msgs.Empty" topic="/output_topic"/>
    <service
        name="/world/triggered_publisher/set_pose"
        reqType="gz.msgs.Pose"
        repType="gz.msgs.Boolean"
        timeout="3000"
        reqMsg="name: 'vehicle_blue', position: {x: -3, z: 0.325}"
    >
    </service>
</plugin>
```
Example c++ code:
```c++
// ------- Parse `input` ---------
sdf::ElementPtr sdf = _sdf->Clone();
if (sdf->HasElement("input"))
{
    auto inputElem = sdf->GetElement("input");
    this->inputMsgType = inputElem->Get<std::string>("type");
    // if (this->inputMsgType.empty()) ...

    auto inTopic = inputElem->Get<std::string>("topic");
    // if (this->inputTopic.empty()) ...

    if (inputElem->HasElement("match"))
    {
        // iterate over the `match` tags
        for (auto matchElem = inputElem->GetElement("match"); matchElem;
            matchElem = matchElem->GetNextElement("match"))
        {
            const auto logicTypeStr = matchElem->Get<std::string>("logic_type", "positive").first;
            // if (logicTypeStr != "positive" && logicTypeStr != "negative") ...

            auto inputMatchString = common::trimmed(matchElem->Get<std::string>());
            if (!inputMatchString.empty())
            {
                if (matchElem->HasAttribute("field"))
                {
                    const auto fieldName = matchElem->Get<std::string>("field");
                    // ...
                }
                else
                {
                    // process the enitre `inputMatchString` as raw string ...
                }
            }
        }
    }
    else /** ... */;
}

// ------- Parse `output` ---------
if (sdfClone->HasElement("output"))
{
    for (auto outputElem = sdfClone->GetElement("output"); outputElem;
            outputElem = outputElem->GetNextElement("output"))
    {
        auto msgType = outputElem->Get<std::string>("type");
        auto topic = outputElem->Get<std::string>("topic");
    }
}
  
// ------- Parse `service` ---------
if (sdf->HasElement("service"))
{
    // iterate over `service` elemetns
    for (auto serviceElem = sdf->GetElement("service"); serviceElem;
            serviceElem = serviceElem->GetNextElement("service"))
    {
        SrvOutputInfo serviceInfo;
        serviceInfo.srvName = serviceElem->Get<std::string>("name");
        // if (serviceInfo.srvName.empty()) ...

        serviceInfo.reqType = serviceElem->Get<std::string>("reqType");
        // if (serviceInfo.reqType.empty()) ...

        serviceInfo.repType = serviceElem->Get<std::string>("repType");
        if (serviceInfo.repType.empty())

        serviceInfo.reqMsg = serviceElem->Get<std::string>("reqMsg");
        // if (serviceInfo.reqMsg.empty()) ...

        std::string timeoutInfo = serviceElem->Get<std::string>("timeout");
        // if (timeoutInfo.empty()) ...
    }
}
```

For example see the [`gz-sim/src/systems/triggered_publisher/TriggeredPublisher.cc`](https://github.com/gazebosim/gz-sim/blob/1d5114bb0614b995220a9fa7d30d8d5d2da576a0/src/systems/triggered_publisher/TriggeredPublisher.cc#L488C6-L488C35).