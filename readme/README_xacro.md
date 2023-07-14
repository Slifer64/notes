# xacro

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


Useful **inertia** macros:
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

---

## Arguments

```xml
<xacro:arg name="use_sim" default="false"/>
```
**Get** value: `"$(arg use_sim)"` \
**Set** value: `xacro robot_urdf.xacro use_sim:=true other_arg:=hello`

---

## Properties

```xml
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_length" value="0.07"/>
<xacro:property name="wheel_offset" value="${wheel_length - 1.2*wheel_radius}"/>
```
Get value: `"${wheel_radius}"`. \
Arithmetic operations: `"${1.2*wheel_offset - wheel_length/5}"`.

---

## Inlucde other xacro

```xml
<xacro:include filename="my_other.xacro" />
```

---

## if / else

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

<xacro:unless value="$(gripper == '')">
    <xacro:if value="${gripper == 'rg2_ft'}">
        <!-- ... -->
    </xacro:if>
    <xacro:if value="${gripper == 'epick40'}">
        <!-- ... -->
    </xacro:if>
</xacro:unless>
```

---

## material

```xml
<material name="orange">
    <color rgba="1 0.3 0.1 1"/>
</material>
```
Usage: `<material name="orange"/>`

---

## link

- **simple geometries**

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

    <!-- colision (usually the same as visual, without the material) -->
    <collision>
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"/>
        <geometry>
            <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
    </collision>
    <!-- inertia -->
    <xacro:inertial_box mass="0.5" x="${chassis_length}" y="${chassis_width}" z="${chassis_height}">
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}" rpy="0 0 0"/>
    </xacro:inertial_box>
</link>
```

- **`mesh` files**

```xml
<property name="mesh_path" value = "package://my_package/meshes" />

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


---

## joint

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


