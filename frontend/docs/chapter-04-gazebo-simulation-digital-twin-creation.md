---
title: Chapter 4 — Gazebo Simulation - Digital Twin Creation
description: Creating and using digital twins of robots with Gazebo physics simulation
id: chapter-04-gazebo-simulation-digital-twin-creation
sidebar_position: 4
---

import ContentFilter from '@site/src/components/ContentFilter';

# Chapter 4 — Gazebo Simulation - Digital Twin Creation

## Introduction

Welcome to Module 2: The Digital Twin (Gazebo & Unity). This chapter focuses on Gazebo simulation, which serves as a crucial component in developing and testing Physical AI systems. Gazebo provides a physics-based simulation environment where you can develop, test, and validate robot behaviors before deploying them to physical hardware.

A "digital twin" is a virtual representation of a physical system that mirrors its real-world behavior. In robotics, this means creating a simulation that accurately models the physics, dynamics, sensors, and environment of the real robot. This allows you to:

- Test algorithms without risk of hardware damage
- Validate control systems in various scenarios
- Generate synthetic data for AI training
- Accelerate development cycles
- Debug systems safely

This chapter will cover the fundamentals of Gazebo simulation, physics modeling, sensor simulation, and how to create realistic digital twins of humanoid robots.

## Gazebo Simulation Environment

### Overview of Gazebo

Gazebo is a 3D simulation environment that provides:

- **Realistic physics simulation**: Accurate models of gravity, friction, collisions, and dynamics
- **Sensor simulation**: Cameras, LiDAR, IMUs, force/torque sensors, and more
- **3D visualization**: Real-time rendering of robot and environment
- **Plugin architecture**: Extensible system for custom behaviors
- **ROS integration**: Native support for ROS and ROS 2 communication

### Installing and Setting Up Gazebo

Gazebo typically comes with ROS installations. For standalone use or specific versions:

```bash
# For ROS 2 Humble
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control

# Verify installation
gz sim --version
```

### Basic Gazebo Usage

Launching Gazebo with a world file:

```bash
# Launch Gazebo with default empty world
gz sim

# Launch with a specific world
gz sim -r empty.sdf

# Launch with verbose output
gz sim -v 4 -r -s
```

## Physics Simulation Concepts

### Physics Engines

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Default, good balance of speed and accuracy
- **Bullet**: Good for articulated models and contact modeling
- **DART**: Advanced constraint handling
- **Simbody**: High-fidelity simulations

### Gravity and Environmental Forces

```xml
<!-- In world file -->
<world name="default">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### Collision Detection and Response

Gazebo handles collisions using:
- **Contact models**: Define how objects interact when they collide
- **Friction parameters**: Control surface interactions
- **Bounce coefficients**: Model elasticity
- **Contact constraints**: Handle multiple simultaneous contacts

## Setting Up Robot Models in Gazebo

### Integrating URDF with Gazebo

To use your URDF robot in Gazebo, you need to add Gazebo-specific tags:

```xml
<!-- In your URDF file -->
<robot name="my_humanoid">
  <!-- ... your existing URDF links and joints ... -->

  <!-- Gazebo plugin for ros2_control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_namespace>/my_humanoid</robot_namespace>
      <robot_param>/robot_description</robot_param>
      <parameters>$(find my_robot_description)/config/my_robot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo-specific properties for links -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <turning_inertia_kinematic>true</turning_inertia_kinematic>
  </gazebo>
</robot>
```

### Robot Control in Gazebo

Setting up controller configuration for simulation:

```yaml
# config/my_robot_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip
      - left_knee
      - left_ankle

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip
      - right_knee
      - right_ankle
```

## Sensor Simulation in Gazebo

### Camera Sensors

Simulating RGB cameras in Gazebo:

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
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
      <min_depth>0.1</min_depth>
      <max_depth>100</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensors

Simulating laser range finders:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors

Simulating Inertial Measurement Units:

```xml>
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <topic>__default_topic__</topic>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>imu/data</topicName>
      <serviceName>imu/service</serviceName>
      <gaussianNoise>0.001</gaussianNoise>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### Force/Torque Sensors

For grippers and contact detection:

```xml>
<gazebo reference="gripper_left_finger">
  <sensor name="ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <plugin name="ft_plugin" filename="libgazebo_ros_ft_sensor.so">
      <topicName>ft_sensor</topicName>
      <bodyName>gripper_left_finger</bodyName>
    </plugin>
  </sensor>
</gazebo>
```

## Creating Realistic Worlds

### Basic World Structure

```xml
<!-- worlds/my_environment.sdf -->
<sdf version="1.7">
  <world name="my_world">
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Custom objects -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="table_base">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Physics properties -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
  </world>
</sdf>
```

### Advanced World Features

#### Lighting Effects

```xml
<light name="custom_light" type="spot">
  <pose>5 0 5 0 0.5 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.5 0.5 0.5 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.5</constant>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
  <direction>-1 0 -1</direction>
  <spot>
    <inner_angle>0.1</inner_angle>
    <outer_angle>0.5</outer_angle>
    <falloff>10</falloff>
  </spot>
</light>
```

#### Terrain Modeling

```xml
<model name="terrain">
  <static>true</static>
  <link name="terrain_link">
    <!-- Heightmap for complex terrain -->
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>model://my_terrain/heightmap.png</uri>
          <size>10 10 2</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>model://my_terrain/heightmap.png</uri>
          <size>10 10 2</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Terrain</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

## Advanced Simulation Features

### Dynamic Simulation Parameters

Adjusting simulation parameters for different use cases:

```xml
<physics name="physics" type="ode">
  <!-- Time step (smaller = more accurate but slower) -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = real-time, >1 = faster, <1 = slower) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Update rate (how often physics is calculated) -->
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver settings -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Plugin Development

Example of a custom Gazebo plugin for robot behavior:

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
  class CustomRobotPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      this->world = _model->GetWorld();
      
      // Initialize ROS 2 node
      rclcpp::init(0, nullptr);
      this->ros_node = std::make_shared<rclcpp::Node>("gazebo_custom_robot");
      
      // Subscribe to velocity commands
      this->cmd_vel_sub = this->ros_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CustomRobotPlugin::OnCmdVel, this, std::placeholders::_1));
      
      // Connect to Gazebo update event
      this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CustomRobotPlugin::OnUpdate, this));
    }

  private:
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      this->target_vel_x = msg->linear.x;
      this->target_vel_yaw = msg->angular.z;
    }
    
    void OnUpdate()
    {
      // Apply control logic to robot
      auto link = this->model->GetLink();
      // Implement physics-based control
    }
    
    physics::ModelPtr model;
    physics::WorldPtr world;
    std::shared_ptr<rclcpp::Node> ros_node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    event::ConnectionPtr update_connection;
    double target_vel_x = 0;
    double target_vel_yaw = 0;
  };

  GZ_REGISTER_MODEL_PLUGIN(CustomRobotPlugin)
}
```

## Simulation Best Practices

### Performance Optimization

1. **Mesh Simplification**: Use simplified collision meshes separate from visual meshes
2. **Update Rates**: Balance physics update rate with real-time performance
3. **Sensor Configuration**: Optimize sensor settings for your use case
4. **Plugin Efficiency**: Write efficient plugins that don't hog CPU cycles

### Accuracy Considerations

1. **Physics Parameters**: Match simulation parameters to real-world values
2. **Inertial Properties**: Use realistic mass, center of mass, and inertia
3. **Friction Models**: Accurately model surface interactions
4. **Sensor Noise**: Include realistic noise models for sensors

### Validation Strategies

1. **Reality Gap Analysis**: Understand and quantify differences between sim and real
2. **Domain Randomization**: Vary simulation parameters to improve transfer
3. **System Identification**: Measure real robot parameters for simulation tuning
4. **Sim-to-Real Transfer**: Develop methods to bridge simulation and reality

## Integration with ROS 2

### Launching Simulation

Creating launch files for integrated simulation:

```python
# launch/sim_robot.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'worlds',
                    'my_world.sdf'
                ])
            }.items()
        ),
        
        # Spawn robot in simulation
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-x', '0', '-y', '0', '-z', '1.0'
            ],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': 
                PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'urdf',
                    'my_robot.urdf.xacro'
                ])
            }]
        ),
    ])
```

### Simulation Monitoring

Use ROS 2 tools to monitor simulation:

```bash
# Monitor topics
ros2 topic echo /my_robot/joint_states

# Visualize in RViz
ros2 run rviz2 rviz2

# Command robot joints
ros2 topic pub /my_robot/position_commands trajectory_msgs/msg/JointTrajectory
```

## Troubleshooting Common Issues

### Physics Instability

```bash
# Common causes and solutions:
# 1. Increase physics update rate in world file
# 2. Decrease max step size
# 3. Adjust solver parameters
# 4. Check mass properties in URDF
```

### Sensor Issues

```bash
# Check sensor topics
ros2 topic list | grep sensor

# Verify sensor plugins loaded
gz topic -l
```

### Controller Problems

```bash
# Check controller status
ros2 control list_controllers

# Verify joint names match between URDF and controller config
ros2 param list
```

## Summary

This chapter has covered the fundamentals of Gazebo simulation for creating digital twins of robots. You've learned about:

- The physics simulation capabilities of Gazebo
- How to integrate URDF robot models with Gazebo
- Sensor simulation for cameras, LiDAR, IMUs, and other sensors
- Creating realistic worlds and environments
- Advanced simulation features and plugin development
- Best practices for performance and accuracy
- Integration with ROS 2 workflow

Simulation is a critical component of Physical AI development, allowing you to test and validate systems safely before deployment to real hardware. The digital twin approach enables rapid development and testing of complex robotic behaviors.

---

## Exercises

1. Create a Gazebo world with obstacles and simulate your robot navigating through it.
2. Implement sensor simulation for a humanoid robot with cameras and IMUs.
3. Tune physics parameters to achieve stable simulation of a humanoid robot balance.
4. Develop a custom Gazebo plugin that implements a specific behavior for your robot.
5. Create a launch file that fully integrates your robot model with Gazebo simulation.
