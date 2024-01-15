# Contents
- [macros](#define-macro)
    - [define macro](#define-macro)
    - [`intertia` macros](#useful-inertia-macros)
    - [pass `dict` to macro param](#pass-dictionary-param-to-macro)
- [arguments](#arguments)
- [properties](#properties)
- [load params from yaml](#load-params-from-yaml)
- [include other xacro](#inlucde-other-xacro)
- [`if/else`](#if--else)
- [material](#material)
- [link](#link)
    - [simple geometries](#simple-geometries)
    - [mesh files](#mesh-files)
- [joint](#joint)
- [misc](#misc)

# Macros

## Define macro

```xml
<xacro:macro name="my_macro" 
    params="param1 
            param2:=default_value 
            joint_lim:=${default_joint_lim} 
            *origin *visual *my_block">
    <link name="${param1}">
        <xacro:insert_block name="visual"/>
    </link>
    <link name="${param2}" />
    <joint name="${param1}_${param2}_joint" type="revolute">
        <xacro:insert_block name="origin"/>
        <limit lower="${-joint_lim}" upper="${joint_lim}" effort="28.0" velocity="6.28"/>
    </joint>
    <xacro:insert_block name="my_block"/>
</xacro:macro>
```
Usage: 
```xml
<!-- param ${joint_lim} will take the default value -->
<xacro:my_macro param1="link1" param2="link2">
    <visual>
        <origin xyz="0 0 0"/>
        <geometry>
            <box size="0 0 0"/>
        </geometry>
        <material name="orange"/>
    </visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <my_block ... />
</xacro:my_macro>
```


## Useful **inertia** macros:
```xml
<!-- cylinder -->
<xacro:macro name="cylinder_inertial" params="mass radius length *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
        iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
        izz="${0.5 * mass * radius * radius}" />
    </inertial>
</xacro:macro>

<!-- box -->
<xacro:macro name="box_inertial" params="mass x y z *origin">
    <inertial>
        <xacro:insert_block name="origin"/>
        <mass value="${mass}" />
        <inertia ixx="${(1/12) * mass * (y*y+z*z)}" ixy="0.0" ixz="0.0"
                iyy="${(1/12) * mass * (x*x+z*z)}" iyz="0.0"
                izz="${(1/12) * mass * (x*x+y*y)}" />
    </inertial>
</xacro:macro>

<!-- sphere -->
<xacro:macro name="sphere_inertial" params="mass radius *origin">
    <inertial>
        <xacro:insert_block name="origin"/>
        <mass value="${mass}" />
        <inertia ixx="${(2/5) * mass * (radius*radius)}" ixy="0.0" ixz="0.0"
                iyy="${(2/5) * mass * (radius*radius)}" iyz="0.0"
                izz="${(2/5) * mass * (radius*radius)}" />
    </inertial>
</xacro:macro>  
```

## Pass dictionary param to macro
Example:
```xml
<xacro:macro name="my_robot" params="prefix parent offset:=${dict(x=0.0,y=0.0,z=0.0)}">

<joint name="${parent}_${prefix}base_link_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${prefix}base_link"/>
    <origin xyz="${offset['x']} ${offset['y']} ${offset['z']}" rpy="0 0 0" />
</joint>

<link name="${prefix}base_link">
    <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <box size="0.2 0.3 0.4" />
    </geometry>
    <material name="Cyan">
        <color rgba="0.0 1.0 1.0 0.2"/>
    </material>
    </visual>
</link>

</xacro:macro>

<link name="base_link" />
<xacro:my_robot prefix="robot" parent="base_link" offset="${dict(x=0.5,y=0.1,z=1.6)}" />
```

---

# Arguments

```xml
<xacro:arg name="use_sim" default="false"/>
```
**Get** value: `"$(arg use_sim)"` \
**Set** value: `xacro robot_urdf.xacro use_sim:=true other_arg:=hello`

---

# Properties

```xml
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_length" value="0.07"/>
<xacro:property name="wheel_offset" value="${wheel_length - 1.2*wheel_radius}"/>
```
Get value: `"${wheel_radius}"`. \
Arithmetic operations: `"${1.2*wheel_offset - wheel_length/5}"`.

---

# Load params from yaml
Lets say we have the yaml:
```yaml
links:
    shoulder:
      radius: 0.06
      length: 0.15
    upperarm:
      radius: 0.06
      length: 0.425

inertia_parameters:
  shoulder:
    CoM: [0.0, 0.0, 0.08]
    mass: 2.5
    inertia: [0.15, 0.15, 0.01] # ixx, iyy, izz
  upper_arm:
    CoM: [0.0, 0.0, 0.03]
    mass: 1.6
    inertia: [0.1, 0.1, 0.01]
``` 
We can load it using `${xacro.load(<path_to_yaml_file>)}` and store it to a `xacro:property`, from which we can then read values as in a dictionary, i.e.:
```xml
<!-- the path to the *.yaml -->
<xacro:property name="robot_params_file" value="$(find my_robot)/config/params.yaml"/>

<!-- load yaml and store it as a property -->
<xacro:property name="robot_params" value="${xacro.load_yaml(robot_params_file)}"/>

<!-- access values as in a dictionary -->

<xacro:property name="link_params" value="${robot_params['links']}"/>
<xacro:property name="inertia_params" value="${robot_params['inertia_parameters']}"/>

<link name="shoulder">
    <visual>
        <geometry>
            <cylinder radius="${link_params['shoulder']['radius']}" length="${link_params['shoulder']['length']}" />
        </geometry>
        <!-- 
        <material ...
        <origin ... 
        -->
    </visual>
    <inertial>
      <origin xyz="${inertia_params['shoulder']['CoM'][0]} ${inertia_params['shoulder']['CoM'][1]} ${inertia_params['shoulder']['CoM'][2]}" rpy="0 0 0" />
      <mass value="${inertia_params['shoulder']['mass']}" />
      <inertia 
        ixx="${inertia_params['shoulder']['inertia'][0]}" ixy="0.0" ixz="0.0"
        iyy="${inertia_params['shoulder']['inertia'][1]}" iyz="0.0"
        izz="${inertia_params['shoulder']['inertia'][2]}" 
        />
    </inertial>
</link>
```

# Inlucde other xacro

```xml
<xacro:include filename="my_other.xacro" />
<xacro:include filename="$(find <package_name>)/urdf/my_file.xacro" />
```

---

# if / else

```xml
<xacro:arg name="use_sim" default="false"/>

<xacro:if value="$(arg use_sim)"> <!-- if -->
    <!-- do stuff -->
</xacro:if>
<xacro:unless value="$(arg use_sim)"> <!-- else -->
    <!-- do other stuff -->
</xacro:unless>
```

```xml

<!-- pass $(arg gripper) to a property, to use it in expressions -->
<xacro:arg name="gripper" default="" />
<xacro:property name="gripper" value="$(arg gripper)" />

<xacro:unless value="${gripper == ''}">
    <xacro:if value="${gripper == 'rg2_ft'}">
        <!-- ... -->
    </xacro:if>
    <xacro:if value="${gripper == 'epick40'}">
        <!-- ... -->
    </xacro:if>
</xacro:unless>
```

---

# material

```xml
<material name="orange">
    <color rgba="1 0.3 0.1 1"/>
</material>
```
Usage: `<material name="orange"/>`

---

# link

- ## simple geometries

```xml
<link name="chassis">
    <!-- visual -->
    <visual>
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"/>
        <geometry>
            <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
        <material name="orange"/>
    </visual>

    <!-- additional visual can be specified
    <visual>
        ...
    </visual> -->
    
    <!-- colision (usually the same as visual, without the material) -->
    <collision>
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"/>
        <geometry>
            <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
    </collision>
    <!-- additional collision can be specified
    <collision>
        ...
    </collision> -->
    
    <!-- inertia -->
    <xacro:inertial_box mass="0.5" x="${chassis_length}" y="${chassis_width}" z="${chassis_height}">
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}" rpy="0 0 0"/>
    </xacro:inertial_box>
</link>
```

- ## `mesh` files

```xml
<property name="mesh_path" value = "package://my_package/meshes" />
<property name="mesh_path" value = "file://<path_to_meshes_folder>" />

<link name="arm_base">
    <!-- visual -->
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
                <mesh filename="${mesh_path}/arm_base.dae"/>
        </geometry>
        <material name="orange"/>
    </visual>

    <!-- colision
    Notice that for collision a convex approximation mesh model is used
    to speed up colision detection! -->
    <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
                <mesh filename="${mesh_path}/convex/arm_base_convex.stl"/>
        </geometry>
    </collision>
    <!-- inertia -->
    <xacro:inertial_box mass="0.5" x="${chassis_length}" y="${chassis_width}" z="${chassis_height}">
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}" rpy="0 0 0"/>
    </xacro:inertial_box>
</link>
```

Multiple `visual` and/or `colision` tags can be specified within the same `link`.

---

# joint

```xml
<joint name="${prefix}_arm_1_joint" type="revolute"> <!--or continuous for no limits-->
    <parent link="${prefix}_arm_1_link"/>
    <child link="${prefix}_arm_2_link"/>
	<origin xyz="0 0 0.2" rpy="0 0 0"/>

    <!-- the rest are usually optional -->

    <axis xyz="0 -1 0"/> <!-- define -y as the rot-axis (default is z) -->

    <!-- the rest are for dynamics simulation -->
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="306" velocity="${arm_vel_scale_factor * 110 * M_PI / 180}" />
    <dynamics damping="${joint_damping}" friction="${joint_friction}"/>
    <safety_controller
        soft_lower_limit="${-118 * M_PI / 180}"
        soft_upper_limit="${118 * M_PI / 180}"
        k_position="${safety_controller_k_pos}"
        k_velocity="${safety_controller_k_vel}"/>
</joint>
```
For `type=revolute`, `<limit ... />` must be specified. \
Otherwise, use `type=continuous`.


---

# Misc

- Defined arguments/properties are visible in the included xacro files, i.e.:
    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot">
        <!-- ... -->
        <xacro:arg name="sim_mode" default="false"/>
        <!-- ... -->
        <xacro:include filename="my_included.xacro" />
        <!-- ... -->
    </robot>
    ```
    and in `my_included.xacro`:
    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot">

        <xacro:if value="$(arg sim_mode)">
            <!-- do stuff -->
        </xacro:if>
        
    </robot>
    ```

---


