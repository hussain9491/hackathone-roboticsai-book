---
sidebar_label: 'URDF & SDF'
title: 'URDF & SDF'
---

# URDF & SDF

## Introduction

URDF (Unified Robot Description Format) and SDF (Simulation Description Format) are XML-based formats used to describe robot models in robotics applications. URDF is primarily used in ROS (Robot Operating System) environments for representing robot kinematics, dynamics, and visual properties, while SDF is used in Gazebo and other simulation environments for more complex simulation-specific features. Understanding both formats is essential for creating accurate robot models for both real-world applications and simulation environments.

## URDF (Unified Robot Description Format)

### Overview

URDF is an XML format that describes robot models by defining their physical and visual properties. It specifies the kinematic structure, dynamic properties, visual appearance, and collision geometry of robots. URDF is widely used in ROS for robot visualization, kinematic analysis, and motion planning.

### URDF Structure

A basic URDF file contains:
- **Links**: Rigid bodies that make up the robot structure
- **Joints**: Connections between links that define motion constraints
- **Materials**: Visual appearance properties
- **Transmissions**: Actuator interfaces (for control)
- **Gazebo plugins**: Simulation-specific extensions

### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Link Elements

Links represent rigid bodies in the robot structure:

#### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Supported geometries: box, cylinder, sphere, mesh -->
    <box size="1.0 0.5 0.2"/>
  </geometry>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
</visual>
```

#### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1.0 0.5 0.2"/>
  </geometry>
</collision>
```

#### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

### Joint Types

URDF supports several joint types:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (rigid connection)
- **floating**: 6-DOF motion (for mobile bases)
- **planar**: Planar motion (2D translation + rotation)

```xml
<!-- Revolute joint example -->
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Xacro for Complex Models

Xacro (XML Macros) extends URDF with macros, properties, and mathematical expressions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Properties -->
  <xacro:property name="pi" value="3.14159"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Using the macro -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.2 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.2 -0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link" xyz="-0.2 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link" xyz="-0.2 -0.2 0" rpy="0 0 0"/>
</robot>
```

## SDF (Simulation Description Format)

### Overview

SDF is the native format for Gazebo and other simulation environments. It extends URDF capabilities with simulation-specific features like physics properties, sensors, plugins, and world descriptions. SDF supports more complex simulation scenarios including multi-robot environments, terrain, and advanced physics.

### SDF Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Robot model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Robot Model in SDF

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
    <!-- Links -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>

    <!-- Joint -->
    <joint name="chassis_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>10</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Sensors -->
    <link name="sensor_link">
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
  </model>
</sdf>
```

## URDF vs SDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| Primary Use | ROS robot description | Simulation environments |
| Physics | Basic inertial properties | Advanced physics simulation |
| Sensors | Through Gazebo plugins | Native sensor definitions |
| World Description | No | Yes (world files) |
| Multi-robot Support | Through separate models | Native support |
| Gazebo Integration | Requires plugins | Native support |
| Complexity | Simpler for basic models | More complex, more features |

## Best Practices

### URDF Best Practices

1. **Use consistent naming conventions**: Follow ROS naming conventions for links and joints
2. **Define proper inertial properties**: Accurate inertial properties are crucial for simulation
3. **Use collision and visual geometries appropriately**: Collision geometry can be simpler than visual geometry
4. **Organize complex models with Xacro**: Use macros and properties for maintainable models
5. **Validate your URDF**: Use `check_urdf` command to validate URDF files

### SDF Best Practices

1. **Define appropriate physics properties**: Set realistic friction, damping, and restitution coefficients
2. **Use appropriate mesh formats**: Prefer COLLADA (.dae) or STL (.stl) for complex geometries
3. **Optimize collision meshes**: Use simpler meshes for collision detection to improve performance
4. **Configure sensor parameters carefully**: Set appropriate update rates and ranges
5. **Use proper coordinate systems**: Maintain consistency in coordinate frame definitions

## Tools and Utilities

### URDF Tools

- `check_urdf <urdf_file>`: Validate URDF syntax and structure
- `urdf_to_graphiz <urdf_file>`: Generate visual graph of robot structure
- `rviz`: Visualize URDF models in ROS environment
- `gazebo`: Load and simulate URDF models

### SDF Tools

- `gz sdf -p <sdf_file>`: Parse and pretty-print SDF files
- `gz sdf -v <sdf_file>`: Validate SDF files
- `gazebo`: Load and simulate SDF models
- `gz model`: Manage models in Gazebo

## Integration with ROS 2

### URDF in ROS 2

URDF models integrate with ROS 2 through:

- **robot_state_publisher**: Publishes joint states and transforms
- **joint_state_publisher**: Publishes joint state messages
- **TF2**: Provides coordinate transformations between frames
- **rviz2**: Visualizes robot models

Example launch file for URDF:
```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
```

## Common Issues and Troubleshooting

### URDF Issues

1. **Missing inertial properties**: Can cause simulation instabilities
2. **Incorrect joint limits**: May prevent motion planning algorithms from working
3. **Mesh path problems**: Ensure meshes are in correct locations relative to package
4. **Coordinate frame issues**: Verify all transforms are correctly defined

### SDF Issues

1. **Physics parameter tuning**: Improper parameters can cause simulation instabilities
2. **Sensor configuration**: Incorrect sensor parameters can affect simulation accuracy
3. **Model loading failures**: Check file paths and permissions
4. **Performance issues**: Optimize collision meshes and sensor update rates

## Advanced Topics

### Custom Gazebo Plugins

Integrate custom behavior through Gazebo plugins in URDF:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>
  </plugin>
</gazebo>
```

### Multi-Robot Scenarios

For multi-robot simulations, ensure unique names and namespaces:

```xml
<!-- Robot 1 -->
<robot name="robot1">
  <link name="base_link"/>
  <joint name="robot1_joint" type="fixed">
    <parent link="world"/>
    <child link="robot1/base_link"/>
  </joint>
</robot>

<!-- Robot 2 -->
<robot name="robot2">
  <link name="base_link"/>
  <joint name="robot2_joint" type="fixed">
    <parent link="world"/>
    <child link="robot2/base_link"/>
  </joint>
</robot>
```

## Conclusion

URDF and SDF are fundamental formats for robot description in ROS and simulation environments. URDF provides a straightforward way to define robot kinematics and basic properties for ROS applications, while SDF offers more advanced features for simulation including physics, sensors, and world modeling. Understanding both formats and their appropriate use cases is essential for developing robust robotic systems that work seamlessly in both simulation and real-world environments. Proper use of these formats, following best practices, and utilizing available tools ensures accurate robot models that support effective development, testing, and deployment of robotic applications.