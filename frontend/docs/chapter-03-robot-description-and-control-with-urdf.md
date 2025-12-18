---
title: Chapter 3 — Robot Description and Control with URDF
description: Understanding Unified Robot Description Format and robot control mechanisms for humanoid robots
id: chapter-03-robot-description-and-control-with-urdf
sidebar_position: 3
---

import ContentFilter from '@site/src/components/ContentFilter';

# Chapter 3 — Robot Description and Control with URDF

## Introduction

In the previous chapter, we introduced ROS 2 as the nervous system for robotic systems. In this chapter, we will dive deeper into robot description using URDF (Unified Robot Description Format) and understand how to control robots using ROS 2. URDF is essential for humanoid robots as it describes their complex kinematic structure, enabling proper simulation, visualization, and control.

URDF serves as the digital blueprint of a robot, defining its physical structure, visual appearance, collision properties, and how its joints connect various components. For humanoid robots, this description is crucial for simulating their behavior, planning movements, and controlling them effectively in both simulation and real-world environments.

This chapter will cover advanced URDF concepts, Xacro macros for complex robot descriptions, and how to control robots through ROS 2 interfaces. We'll also look at how URDF integrates with control systems for humanoid robots.

## Advanced URDF Concepts

### Robot Model Structure

A complete robot model in URDF includes several key components:

- **Links**: Rigid bodies that form the physical structure of the robot
- **Joints**: Connections between links that define their relative motion
- **Visual elements**: How the robot appears in visualization tools
- **Collision elements**: How the robot interacts with its environment in simulation
- **Inertial properties**: Mass, center of mass, and moments of inertia

### Links in Detail

Each link in a URDF model can contain multiple elements:

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Define shape: box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  
  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Usually simpler than visual for faster computation -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  
  <!-- Physical properties -->
  <inertial>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
    <mass value="1.0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
</link>
```

### Joint Types and Properties

URDF supports various joint types for different kinds of motion:

- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No motion between links
- **Floating**: 6-DOF motion (for mobile bases)
- **Planar**: Motion on a plane

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Materials and Colors

Materials define the visual appearance of robot parts:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 0.5"/>
</material>

<material name="black">
  <color rgba="0 0 0 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>
```

## Xacro: XML Macros for URDF

Xacro (XML Macros) is a macro language that adds features to URDF, making complex robot descriptions more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.08"/>
  
  <!-- Define a macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>
  
  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.5 0.3 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.5 -0.3 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link" xyz="-0.5 0.3 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link" xyz="-0.5 -0.3 0"/>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have complex kinematic structures that require careful URDF modeling. Key aspects include:

### Kinematic Chains

Humanoid robots typically have multiple kinematic chains:

- **Right arm**: base_link → shoulder → elbow → wrist → hand
- **Left arm**: base_link → shoulder → elbow → wrist → hand
- **Right leg**: base_link → hip → knee → ankle → foot
- **Left leg**: base_link → hip → knee → ankle → foot
- **Head-neck**: base_link → neck → head

### Example Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base link representing the torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.8" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.4"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.8" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.85"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>
  
  <!-- Left arm example (simplified) -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="-0.1 0.2 0.6"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3"/>
  </joint>
  
  <!-- Continue for all other joints... -->
</robot>
```

## Robot Control with ROS 2

### Joint State Publisher

The joint state publisher publishes the current state of all joints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]
        
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz
        
        self.time_step = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = []
        
        # Generate oscillating joint positions
        for i in range(len(self.joint_names)):
            pos = math.sin(self.time_step + i) * 0.1
            msg.position.append(pos)
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        self.publisher.publish(msg)
        self.time_step += 0.05
```

### Joint Trajectory Controller

For more sophisticated control, we use joint trajectory controllers:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Send initial trajectory
        self.send_trajectory()

    def send_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_hip', 'left_knee', 'left_ankle']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1)  # 1 second to reach point
        
        trajectory.points = [point]
        
        self.trajectory_publisher.publish(trajectory)
```

## SDF vs URDF

While URDF is used for robot descriptions, SDF (Simulation Description Format) is used by Gazebo for physics simulation. Key differences include:

- **URDF**: Focuses on robot kinematics and visual representation
- **SDF**: Adds physics and sensor properties for simulation

SDF can include URDF models as nested elements:

```xml
<sdf version="1.7">
  <model name="my_robot">
    <include>
      <uri>model://my_robot_model</uri>
    </include>
    
    <!-- Add Gazebo-specific elements -->
    <plugin name="ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_robot</robotNamespace>
    </plugin>
  </model>
</sdf>
```

## Robot State Publisher

The robot state publisher takes joint positions and publishes the forward kinematics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import tf_transformations

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def joint_state_callback(self, msg):
        # Process joint states and publish transforms
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                # Calculate and publish transform
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = joint_name
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                
                self.tf_broadcaster.sendTransform(t)
```

## URDF Best Practices

### Organization

- Use Xacro macros to avoid repetition
- Organize models by function (arms, legs, sensors)
- Use consistent naming conventions
- Document complex joint relationships

### Performance

- Use simple collision geometries when possible
- Optimize mesh resolution for visualization
- Use appropriate inertial properties
- Consider computational complexity for real-time systems

### Simulation Considerations

- Ensure collision meshes are watertight
- Set appropriate friction coefficients
- Use realistic mass properties
- Consider joint limits and actuator capabilities

## Tools for URDF Development

### check_urdf

The `check_urdf` command verifies the syntax and structure of your URDF:

```bash
ros2 run urdf check_urdf my_robot.urdf
```

### robot_state_publisher

Publishes TF transforms based on joint states:

```bash
ros2 run robot_state_publisher robot_state_publisher my_robot.urdf
```

### RViz Visualization

Use RViz to visualize your robot model:

```bash
ros2 run rviz2 rviz2
```

## Integration with ROS 2 Ecosystem

### Controllers

URDF models integrate with ROS 2 controllers through the ros2_control framework:

```xml
<!-- In URDF, define ros2_control interface -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <joint name="left_hip">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

### Sensor Integration

Sensors can be attached to URDF models:

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.08 0.04"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Summary

This chapter has covered the essential concepts of robot description using URDF and how to control robots through ROS 2 interfaces. You've learned about:

- Advanced URDF elements including links, joints, and materials
- Xacro macros for creating maintainable robot descriptions
- How to model humanoid robots with complex kinematic structures
- Robot control mechanisms including joint state publishers and trajectory controllers
- Integration with the broader ROS 2 ecosystem
- Best practices for URDF development

URDF is fundamental to robotics as it provides the digital representation of robots that enables simulation, visualization, and control. Understanding these concepts is essential for working with humanoid robots in both simulation and real-world environments.

---

## Exercises

1. Create a URDF model for a simple humanoid robot with at least 12 joints (arms, legs, and head).
2. Implement a joint state publisher that creates realistic walking motions for a humanoid robot.
3. Design a complete URDF file using Xacro macros to avoid repetition.
4. Integrate a depth camera sensor into your humanoid robot's URDF model.
5. Create a launch file that starts robot state publisher with your URDF model.
