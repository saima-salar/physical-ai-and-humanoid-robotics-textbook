---
title: Chapter 10 — Humanoid Robot Kinematics and Dynamics
description: Understanding the kinematics and dynamics of humanoid robot systems
id: chapter-10-humanoid-kinematics-and-dynamics
sidebar_position: 10
---

import ContentFilter from '@site/src/components/ContentFilter';

# Chapter 10 — Humanoid Robot Kinematics and Dynamics

## Introduction

This chapter focuses on the kinematics and dynamics of humanoid robots, which are essential for understanding and controlling their complex movements. Humanoid robots present unique challenges compared to simpler robots due to their multiple degrees of freedom, bipedal locomotion, and human-like form factor.

The key aspects covered in this chapter include:

- **Forward and inverse kinematics** for multi-joint humanoid systems
- **Dynamics modeling** for force and torque calculations
- **Center of Mass (CoM) control** for balance and stability
- **Zero Moment Point (ZMP)** for bipedal stability analysis
- **Humanoid-specific constraints** for safe and stable motion
- **Walking pattern generation** for bipedal locomotion

Understanding these concepts is crucial for developing controllers that enable humanoid robots to navigate complex environments safely and perform tasks requiring precise manipulation.

## Humanoid Robot Anatomy and Structure

### Humanoid Robot Configuration

Humanoid robots typically have a structure that mimics the human body:

- **Torso**: Central body containing main computing units and power systems
- **Head**: Contains cameras, microphones, and other sensors
- **Arms**: Each with shoulder, elbow, and wrist joints for manipulation
- **Hands**: Multi-fingered for complex grasping tasks
- **Legs**: Each with hip, knee, and ankle joints for locomotion
- **Feet**: Provide support and balance during static and dynamic phases

### Degrees of Freedom (DOF)

A typical humanoid robot has 26-30 or more degrees of freedom:

```text
Humanoid Configuration Example:
- Torso: 1 DOF (yaw rotation)
- Head: 2 DOF (pitch, yaw)
- Left Arm: 7 DOF (3 shoulder + 1 elbow + 3 wrist)
- Right Arm: 7 DOF (3 shoulder + 1 elbow + 3 wrist)
- Left Hand: 4 DOF (for manipulation)
- Right Hand: 4 DOF (for manipulation)
- Left Leg: 6 DOF (3 hip + 1 knee + 2 ankle)
- Right Leg: 6 DOF (3 hip + 1 knee + 2 ankle)
- Total: ~30+ DOF (depending on hand complexity)
```

### Joint Configuration

Humanoid robots use various joint types:

- **Revolute joints**: Rotate around a single axis (most common)
- **Prismatic joints**: Linear motion along a single axis
- **Ball joints**: Multi-axis rotation (in advanced designs)

## Forward Kinematics

### Mathematical Foundation

Forward kinematics determines the end-effector position and orientation from given joint angles. For humanoid robots, this involves complex kinematic chains.

The transformation from joint space to Cartesian space follows the Denavit-Hartenberg (DH) convention:

```python
import numpy as np
from math import sin, cos, pi

def dh_transform(a, alpha, d, theta):
    """Calculate DH transformation matrix"""
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles, dh_params):
    """Calculate forward kinematics for a kinematic chain"""
    T = np.eye(4)  # Identity transformation
    
    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        T_link = dh_transform(a, alpha, d, theta)
        T = T @ T_link
    
    return T  # Final transformation matrix
```

### Multi-Chain Kinematics

Humanoid robots have multiple kinematic chains (arms, legs). Each chain is computed separately:

```python
class HumanoidKinematics:
    def __init__(self):
        # Define DH parameters for each chain
        self.left_arm_dh = [
            # Shoulder (3 DOF): yaw, pitch, roll
            (0, -pi/2, 0, 0),
            (0, pi/2, 0, 0),
            (0, -pi/2, 0, 0),
            # Elbow (1 DOF): pitch
            (0, pi/2, 0, 0),
            # Wrist (3 DOF): pitch, roll, pitch
            (0, -pi/2, 0, 0),
            (0, pi/2, 0, 0),
            (0, -pi/2, 0, 0)
        ]
        
        self.right_arm_dh = self.left_arm_dh  # Same for right arm
        
        self.left_leg_dh = [
            # Hip (3 DOF): roll, pitch, yaw
            (0, -pi/2, 0, 0),
            (0, pi/2, 0, 0),
            (0, -pi/2, 0, 0),
            # Knee (1 DOF): pitch
            (0, pi/2, 0, 0),
            # Ankle (2 DOF): pitch, roll
            (0, -pi/2, 0, 0),
            (0, pi/2, 0, 0)
        ]
        
        self.right_leg_dh = self.left_leg_dh  # Same for right leg
    
    def left_arm_fk(self, joint_angles):
        """Forward kinematics for left arm"""
        return forward_kinematics(joint_angles, self.left_arm_dh)
    
    def right_arm_fk(self, joint_angles):
        """Forward kinematics for right arm"""
        return forward_kinematics(joint_angles, self.right_arm_dh)
    
    def left_leg_fk(self, joint_angles):
        """Forward kinematics for left leg"""
        return forward_kinematics(joint_angles, self.left_leg_dh)
    
    def right_leg_fk(self, joint_angles):
        """Forward kinematics for right leg"""
        return forward_kinematics(joint_angles, self.right_leg_dh)
    
    def get_end_effector_pose(self, joint_angles, chain_name):
        """Get end effector pose for specified chain"""
        if chain_name == "left_arm":
            return self.left_arm_fk(joint_angles[:7])
        elif chain_name == "right_arm":
            return self.right_arm_fk(joint_angles[7:14])
        elif chain_name == "left_leg":
            return self.left_leg_fk(joint_angles[14:20])
        elif chain_name == "right_leg":
            return self.right_leg_fk(joint_angles[20:26])
        else:
            raise ValueError("Invalid chain name")
```

## Inverse Kinematics

### Mathematical Approach

Inverse kinematics determines the joint angles required to achieve a desired end-effector position and orientation. This is more complex than forward kinematics and may have multiple or no solutions.

### Analytical vs Numerical Methods

Humanoid robots typically require numerical methods due to their redundancy (more DOF than required for end-effector positioning).

### Jacobian-Based Methods

The Jacobian matrix relates joint velocities to end-effector velocities:

```python
import numpy as np

class HumanoidIK:
    def __init__(self, kinematics_model):
        self.kin_model = kinematics_model
    
    def jacobian(self, joint_angles, chain_name, epsilon=1e-6):
        """Calculate Jacobian matrix for specified chain"""
        # Get current end-effector pose
        current_pose = self.kin_model.get_end_effector_pose(joint_angles, chain_name)
        current_pos = current_pose[:3, 3]
        
        # Initialize Jacobian (6xN for position and orientation)
        if chain_name in ["left_arm", "right_arm"]:
            n_dof = 7
        else:  # legs
            n_dof = 6
        
        J = np.zeros((6, n_dof))
        
        # Calculate Jacobian columns using finite differences
        for i in range(n_dof):
            # Perturb joint angle
            delta_angles = joint_angles.copy()
            delta_angles[i] += epsilon
            
            # Get perturbed pose
            perturbed_pose = self.kin_model.get_end_effector_pose(delta_angles, chain_name)
            perturbed_pos = perturbed_pose[:3, 3]
            
            # Calculate position change
            J[:3, i] = (perturbed_pos - current_pos) / epsilon
            
            # Calculate orientation change (simplified)
            # In practice, this would involve rotation matrix differences
            J[3:, i] = 0  # Simplified for this example
    
        return J

    def inverse_kinematics(self, target_pose, chain_name, current_angles, 
                          max_iterations=100, tolerance=1e-3):
        """Solve inverse kinematics using Jacobian transpose method"""
        joint_angles = current_angles.copy()
        chain_dof = 7 if chain_name in ["left_arm", "right_arm"] else 6
        
        for iteration in range(max_iterations):
            # Get current end-effector pose
            current_pose = self.kin_model.get_end_effector_pose(joint_angles, chain_name)
            current_pos = current_pose[:3, 3]
            target_pos = target_pose[:3, 3]
            
            # Calculate error
            error = target_pos - current_pos
            
            if np.linalg.norm(error) < tolerance:
                print(f"Converged after {iteration} iterations")
                break
            
            # Calculate Jacobian
            J = self.jacobian(joint_angles, chain_name)
            
            # Use pseudoinverse for redundant systems
            J_pinv = np.linalg.pinv(J[:3, :chain_dof])  # Position only
            
            # Calculate joint angle update
            delta_angles = J_pinv @ error
            
            # Apply update with damping
            joint_angles[:chain_dof] += 0.1 * delta_angles
        
        return joint_angles
```

### Cyclic Coordinate Descent (CCD)

An alternative approach that's often more efficient for humanoid robots:

```python
def ccd_ik(joint_angles, target_pos, chain_dh, max_iterations=100, tolerance=1e-4):
    """Cyclic Coordinate Descent for inverse kinematics"""
    angles = joint_angles.copy()
    
    for iteration in range(max_iterations):
        # Get current end-effector position
        current_transform = forward_kinematics(angles, chain_dh)
        current_pos = current_transform[:3, 3]
        
        # Calculate error
        error = np.linalg.norm(target_pos - current_pos)
        
        if error < tolerance:
            return angles, True  # Success
        
        # Update joints from end to base
        for i in range(len(angles) - 1, -1, -1):
            # Calculate rotation to bring end-effector closer to target
            # Simplified single-axis rotation for this example
            
            # Get position of joint i (for this rotation)
            partial_transform = forward_kinematics(angles[:i+1], chain_dh[:i+1])
            joint_pos = partial_transform[:3, 3]
            
            # Calculate rotation that moves end-effector toward target
            eff_to_target = target_pos - current_pos
            joint_to_eff = current_pos - joint_pos
            
            # Cross product for rotation axis
            rotation_axis = np.cross(joint_to_eff, eff_to_target)
            rotation_axis = rotation_axis / (np.linalg.norm(rotation_axis) + 1e-8)
            
            # Simplified rotation update
            angles[i] += 0.01  # Small step size
    
    return angles, False  # Failed to converge
```

## Dynamics Modeling

### Newton-Euler Formulation

For complex multi-body systems like humanoid robots, the Newton-Euler formulation provides a systematic approach to dynamics modeling:

```python
class HumanoidDynamics:
    def __init__(self):
        # Robot parameters (mass, inertia, etc.)
        self.links = self.define_robot_links()
    
    def define_robot_links(self):
        """Define physical properties of robot links"""
        return {
            'torso': {
                'mass': 10.0,  # kg
                'com': [0, 0, 0.5],  # Center of mass
                'inertia': np.eye(3) * 0.5  # 3x3 inertia matrix
            },
            'head': {
                'mass': 2.0,
                'com': [0, 0, 0.1],
                'inertia': np.eye(3) * 0.1
            },
            # Add other links...
        }
    
    def inverse_dynamics(self, q, q_dot, q_ddot, gravity=[0, 0, -9.81]):
        """
        Calculate required joint torques for given motion
        
        Args:
            q: joint positions
            q_dot: joint velocities  
            q_ddot: joint accelerations
            gravity: gravity vector
        """
        # Calculate torques using recursive Newton-Euler algorithm
        # This is a simplified version - full implementation is complex
        
        # Forward pass: calculate velocities and accelerations
        link_velocities = self.forward_pass(q, q_dot, q_ddot)
        
        # Backward pass: calculate forces and torques
        joint_torques = self.backward_pass(link_velocities, gravity)
        
        return joint_torques
    
    def forward_pass(self, q, q_dot, q_ddot):
        """Forward pass of Newton-Euler algorithm"""
        # Calculate link velocities and accelerations recursively
        # from base to end-effectors
        pass
    
    def backward_pass(self, link_velocities, gravity):
        """Backward pass of Newton-Euler algorithm"""
        # Calculate required joint torques recursively 
        # from end-effectors to base
        pass
```

### Lagrangian Formulation

The Lagrangian approach is often preferred for complex robotic systems:

```python
def compute_lagrangian_dynamics(q, q_dot, robot_params):
    """
    Compute Lagrangian dynamics: D(q)q_ddot + C(q,qdot)q_dot + g(q) = τ
    
    Args:
        q: joint positions
        q_dot: joint velocities
        robot_params: robot physical parameters
    """
    # Calculate mass matrix D(q)
    D = compute_mass_matrix(q, robot_params)
    
    # Calculate Coriolis and centrifugal terms C(q, q_dot)
    C = compute_coriolis_matrix(q, q_dot, robot_params)
    
    # Calculate gravitational terms g(q)
    g = compute_gravity_vector(q, robot_params)
    
    return D, C, g

def compute_mass_matrix(q, robot_params):
    """Compute the mass/inertia matrix D(q)"""
    # This would require complex calculations involving
    # the kinematics of all links in the system
    pass

def compute_coriolis_matrix(q, q_dot, robot_params):
    """Compute Coriolis and centrifugal matrix C(q, q_dot)"""
    # Calculations involving first derivatives of D(q)
    pass

def compute_gravity_vector(q, robot_params):
    """Compute gravity vector g(q)"""
    # Calculations based on link positions and gravity
    pass
```

## Center of Mass and Balance Control

### Center of Mass Calculation

For humanoid robots, maintaining the center of mass (CoM) within the support polygon is crucial for stability:

```python
import numpy as np

class CoMController:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.link_masses = self.extract_masses()
        self.link_positions = {}  # Will be updated with FK
    
    def extract_masses(self):
        """Extract masses from robot parameters"""
        masses = {}
        for link_name, params in self.robot_params.items():
            masses[link_name] = params['mass']
        return masses
    
    def calculate_com(self, joint_angles):
        """Calculate center of mass of the entire robot"""
        total_mass = 0
        weighted_pos = np.zeros(3)
        
        # For each link, calculate CoM position using FK
        links = self.get_link_positions(joint_angles)
        
        for link_name, (position, mass) in links.items():
            weighted_pos += position * mass
            total_mass += mass
        
        if total_mass > 0:
            com = weighted_pos / total_mass
        else:
            com = np.zeros(3)
        
        return com, total_mass
    
    def get_link_positions(self, joint_angles):
        """Get positions of all links using forward kinematics"""
        # This would call the FK functions for each link
        # Simplified example
        positions = {}
        
        # Calculate position of each link using FK
        # This is a simplified example - full implementation would
        # calculate FK for every link in the robot tree
        positions['torso'] = np.array([0, 0, 1.0])  # Simplified
        positions['head'] = np.array([0, 0, 1.5])
        positions['left_hand'] = np.array([0.5, 0.2, 1.2])
        positions['right_hand'] = np.array([0.5, -0.2, 1.2])
        positions['left_foot'] = np.array([0, 0.1, 0])
        positions['right_foot'] = np.array([0, -0.1, 0])
        
        # Apply masses
        for link_name in positions:
            if link_name in self.link_masses:
                positions[link_name] = (positions[link_name], self.link_masses[link_name])
        
        return positions
    
    def calculate_support_polygon(self, left_foot_pos, right_foot_pos):
        """Calculate the support polygon for bipedal stance"""
        # For a two-foot stance, the support polygon is the convex hull
        # of both feet contact points
        
        # Simplified: assume feet are rectangular contact areas
        foot_width = 0.15  # meters
        foot_length = 0.25  # meters
        
        # Calculate corner points of both feet
        left_foot_corners = self.get_foot_corners(left_foot_pos, 'left')
        right_foot_corners = self.get_foot_corners(right_foot_pos, 'right')
        
        all_corners = left_foot_corners + right_foot_corners
        
        # Find convex hull (simplified to x-y plane)
        support_polygon = self.convex_hull_2d(all_corners)
        
        return support_polygon
    
    def get_foot_corners(self, foot_pos, foot_type):
        """Get corner points of a foot"""
        # Simplified foot model: rectangular contact area
        width = 0.15 / 2  # half width
        length = 0.25 / 2  # half length
        
        # Define corner offsets
        corners = [
            [length, width, 0],
            [length, -width, 0],
            [-length, -width, 0],
            [-length, width, 0]
        ]
        
        # Apply appropriate offset for left/right foot
        if foot_type == 'right':
            corners = [[x, -y, z] for x, y, z in corners]
        
        # Translate to foot position
        corners = [np.array(foot_pos) + np.array(corner) for corner in corners]
        return corners
    
    def convex_hull_2d(self, points):
        """Calculate 2D convex hull (simplified implementation)"""
        # This would implement Graham scan or similar algorithm
        # For simplicity, we'll just return the points
        return points
    
    def is_balance_stable(self, com, support_polygon):
        """Check if CoM is within support polygon"""
        # Simplified check in 2D (x, y plane)
        com_2d = com[:2]
        
        # Check if point is inside polygon (ray casting algorithm)
        return self.point_in_polygon(com_2d, support_polygon)
    
    def point_in_polygon(self, point, polygon):
        """Check if 2D point is inside polygon using ray casting"""
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0][0], polygon[0][1]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n][0], polygon[i % n][1]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
```

## Zero Moment Point (ZMP)

### ZMP Theory

Zero Moment Point (ZMP) is a crucial concept for bipedal stability analysis. It's the point on the ground where the net moment of the ground reaction forces is zero.

```python
class ZMPController:
    def __init__(self, robot_height=1.0):
        self.robot_height = robot_height
        self.gravity = 9.81
    
    def calculate_zmp_simple(self, com_pos, com_acc):
        """
        Calculate ZMP using simplified inverted pendulum model
        
        Args:
            com_pos: Center of mass position [x, y, z]
            com_acc: Center of mass acceleration [ax, ay, az]
        """
        # Simplified ZMP calculation for x-direction
        x_com, y_com, z_com = com_pos
        ax_com, ay_com, az_com = com_acc
        
        # ZMP_x = x_com - h * ax_com / g (where h is CoM height)
        zmp_x = x_com - (z_com - 0) * ax_com / self.gravity  # Assuming z=0 is ground
        zmp_y = y_com - (z_com - 0) * ay_com / self.gravity
        
        return np.array([zmp_x, zmp_y, 0])
    
    def calculate_zmp_detailed(self, forces, moments, cop_x, cop_y):
        """
        More detailed ZMP calculation from force/moment measurements
        
        Args:
            forces: [fx, fy, fz] - Ground reaction forces
            moments: [mx, my, mz] - Moments at contact point
            cop_x, cop_y: Center of Pressure coordinates
        """
        fx, fy, fz = forces
        mx, my, mz = moments
        
        if abs(fz) < 1e-6:  # Avoid division by zero
            return np.array([cop_x, cop_y, 0])
        
        # ZMP calculation
        zmp_x = cop_x - my / fz
        zmp_y = cop_y + mx / fz
        
        return np.array([zmp_x, zmp_y, 0])
    
    def is_zmp_stable(self, zmp, support_polygon, margin=0.05):
        """Check if ZMP is within support polygon with safety margin"""
        # First expand support polygon by margin
        expanded_polygon = self.expand_polygon(support_polygon, margin)
        
        # Check if ZMP is inside expanded polygon
        zmp_2d = zmp[:2]
        return self.point_in_polygon(zmp_2d, expanded_polygon)
    
    def expand_polygon(self, polygon, margin):
        """Expand polygon by margin distance"""
        # Simplified expansion - in reality, this would use more sophisticated algorithms
        expanded = []
        for point in polygon:
            # Expand in direction away from centroid
            centroid = np.mean(polygon, axis=0)[:2]
            direction = (point[:2] - centroid)
            direction = direction / (np.linalg.norm(direction) + 1e-8)
            expanded_point = point[:2] + direction * margin
            expanded.append([expanded_point[0], expanded_point[1], point[2]])
        return expanded
    
    def point_in_polygon(self, point, polygon):
        """Check if 2D point is inside polygon using ray casting"""
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0][0], polygon[0][1]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n][0], polygon[i % n][1]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
```

## Humanoid Walking Patterns

### Inverted Pendulum Model

For bipedal walking, the Linear Inverted Pendulum Model (LIPM) is commonly used:

```python
class LIPMController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
    
    def calculate_foot_placement(self, current_com, desired_com, dt):
        """Calculate foot placement for stable walking"""
        # Inverted pendulum dynamics
        # x_desired = x_current * cosh(ω*t) + x_dot_current * sinh(ω*t) / ω
        # For foot placement, we want the ZMP to be at the foot position
        
        # Simplified approach: place foot where CoM will be projected at next step
        foot_x = current_com[0]  # Simplified
        foot_y = current_com[1]  # Simplified
        
        # More sophisticated approach would consider walking gait
        return np.array([foot_x, foot_y, 0])
    
    def calculate_com_trajectory(self, start_com, goal_com, step_time, dt):
        """Generate CoM trajectory for walking step"""
        steps = int(step_time / dt)
        trajectory = []
        
        for i in range(steps + 1):
            t = i * dt
            
            # Interpolate CoM position using various methods
            # This is simplified - real implementations use more sophisticated patterns
            progress = t / step_time
            current_com = start_com * (1 - progress) + goal_com * progress
            
            trajectory.append(current_com)
        
        return trajectory
    
    def generate_walking_pattern(self, step_length=0.3, step_height=0.05, 
                               step_time=0.8, num_steps=10):
        """Generate complete walking pattern"""
        pattern = {
            'left_foot': [],
            'right_foot': [],
            'com_trajectory': [],
            'zmp_trajectory': []
        }
        
        # Initialize positions
        com_pos = np.array([0, 0, self.com_height])
        left_foot = np.array([0, 0.1, 0])   # Left foot slightly to the side
        right_foot = np.array([0, -0.1, 0]) # Right foot slightly to the side
        
        dt = 0.01  # Integration time step
        
        for step in range(num_steps):
            # Calculate step trajectory for this step
            # This would involve double support and single support phases
            step_trajectory = self.generate_single_step(
                com_pos, step_length, step_height, step_time, dt)
            
            # Add to overall pattern
            for data_point in step_trajectory:
                pattern['com_trajectory'].append(data_point['com'])
                pattern['left_foot'].append(data_point['left_foot'])
                pattern['right_foot'].append(data_point['right_foot'])
                pattern['zmp_trajectory'].append(data_point['zmp'])
        
        return pattern
    
    def generate_single_step(self, start_com, step_length, step_height, 
                           step_time, dt):
        """Generate single step trajectory"""
        # Single support phase (SSP) and double support phase (DSP)
        # Simplified implementation
        
        trajectory = []
        t = 0
        current_com = start_com.copy()
        
        while t <= step_time:
            # Calculate current position in step
            step_progress = t / step_time
            
            # CoM follows a 3rd degree polynomial trajectory
            # This is a simplified representation
            current_com[0] = start_com[0] + step_length * step_progress
            current_com[2] = start_com[2] + step_height * np.sin(np.pi * step_progress)  # Foot lift
            
            # Calculate corresponding ZMP
            zmp = self.calculate_zmp_simple(current_com, np.zeros(3))
            
            # Foot positions are more complex in real walking
            # This is a significant simplification
            current_data = {
                'com': current_com.copy(),
                'left_foot': np.array([current_com[0], 0.1, 0]),  # Simplified
                'right_foot': np.array([current_com[0], -0.1, 0]),  # Simplified
                'zmp': zmp
            }
            
            trajectory.append(current_data)
            t += dt
        
        return trajectory
```

## Control Strategies

### Operational Space Control

Operational space control is particularly useful for humanoid robots with redundant DOF:

```python
class OperationalSpaceController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
    
    def operational_space_control(self, task_jacobian, task_error, 
                                task_desired_acceleration, joint_angles, 
                                joint_velocities, gravity_compensation=True):
        """
        Compute operational space control law
        
        τ = J^T * F_task + τ_null + τ_gravity
        """
        # Calculate task-space force
        stiffness = 100.0
        damping = 20.0
        
        task_force = (stiffness * task_error - 
                     damping * task_desired_acceleration)
        
        # Calculate joint torques
        jacobian = task_jacobian  # J_task
        
        # Use pseudoinverse for redundancy resolution
        jacobian_pinv = np.linalg.pinv(jacobian)
        tau_task = jacobian_pinv.T @ task_force
        
        # Null-space projection for secondary tasks
        mass_matrix = self.calculate_mass_matrix(joint_angles)
        lambda_task = jacobian @ np.linalg.inv(mass_matrix) @ jacobian.T
        null_projector = np.eye(len(joint_angles)) - jacobian_pinv @ jacobian
        
        # Add null-space task (e.g., keep joints away from limits)
        tau_null = self.calculate_null_space_task(
            null_projector, joint_angles, joint_velocities)
        
        # Add gravity compensation
        tau_grav = self.calculate_gravity_compensation(joint_angles) if gravity_compensation else 0
        
        # Total torque
        tau = tau_task + tau_null + tau_grav
        
        return tau
    
    def calculate_null_space_task(self, null_projector, joint_angles, joint_velocities):
        """Calculate torques for null-space tasks"""
        # Example: avoid joint limits
        joint_preferences = np.zeros(len(joint_angles))  # Preferred joint positions
        
        # Simple joint centering task
        k_null = 10.0
        tau_null = k_null * (joint_preferences - joint_angles)
        
        return null_projector @ tau_null
    
    def calculate_gravity_compensation(self, joint_angles):
        """Calculate gravity compensation torques"""
        # This would use the robot's dynamics model
        # Simplified for this example
        return np.zeros(len(joint_angles))
    
    def calculate_mass_matrix(self, joint_angles):
        """Calculate joint space mass matrix"""
        # This would be computed from the robot's DH parameters and physical properties
        # Identity matrix for this simplified example
        return np.eye(len(joint_angles))
```

## Simulation and Validation

### Humanoid Simulation Example

```python
import numpy as np
import matplotlib.pyplot as plt

class HumanoidSimulator:
    def __init__(self):
        self.kinematics = HumanoidKinematics()
        self.dynamics = HumanoidDynamics()
        self.com_controller = CoMController({})
        self.zmp_controller = ZMPController()
        
        # Simulation parameters
        self.dt = 0.01  # 100 Hz
        self.time = 0
    
    def simulate_step(self, joint_commands):
        """Simulate one time step of humanoid motion"""
        # Update robot state based on commands
        # This would integrate dynamics equations
        
        # Calculate forward kinematics
        end_effector_poses = {}
        for chain in ['left_arm', 'right_arm', 'left_leg', 'right_leg']:
            # Get appropriate joint angles for this chain
            if chain == 'left_arm':
                angles = joint_commands[:7]
            elif chain == 'right_arm':
                angles = joint_commands[7:14]
            elif chain == 'left_leg':
                angles = joint_commands[14:20]
            else:  # right leg
                angles = joint_commands[20:26]
            
            pose = self.kinematics.get_end_effector_pose(angles, chain)
            end_effector_poses[chain] = pose
        
        # Calculate CoM and ZMP
        com, total_mass = self.com_controller.calculate_com(joint_commands)
        
        # Check stability
        # Calculate support polygon based on foot positions
        left_foot_pos = end_effector_poses['left_leg'][:3, 3] if 'left_leg' in end_effector_poses else np.zeros(3)
        right_foot_pos = end_effector_poses['right_leg'][:3, 3] if 'right_leg' in end_effector_poses else np.zeros(3)
        
        support_polygon = self.com_controller.calculate_support_polygon(left_foot_pos, right_foot_pos)
        is_stable = self.com_controller.is_balance_stable(com, support_polygon)
        
        self.time += self.dt
        
        return {
            'end_effector_poses': end_effector_poses,
            'com': com,
            'is_stable': is_stable,
            'support_polygon': support_polygon
        }
    
    def simulate_walking_sequence(self, walking_pattern):
        """Simulate a complete walking sequence"""
        simulation_results = []
        
        for step in walking_pattern:
            result = self.simulate_step(step['joint_angles'])
            simulation_results.append(result)
        
        return simulation_results
    
    def visualize_simulation(self, simulation_results):
        """Visualize simulation results"""
        # Extract CoM trajectory
        com_x = [result['com'][0] for result in simulation_results]
        com_y = [result['com'][1] for result in simulation_results]
        
        # Create plot
        plt.figure(figsize=(12, 6))
        
        # Plot CoM trajectory
        plt.subplot(1, 2, 1)
        plt.plot(com_x, com_y, 'b-', linewidth=2, label='CoM Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Center of Mass Trajectory')
        plt.legend()
        plt.grid(True)
        
        # Plot stability indicator
        stability = [result['is_stable'] for result in simulation_results]
        plt.subplot(1, 2, 2)
        plt.plot(stability, 'g-', linewidth=2)
        plt.xlabel('Time Step')
        plt.ylabel('Stable (1) / Unstable (0)')
        plt.title('Balance Stability Over Time')
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    """Main function to demonstrate humanoid kinematics and dynamics"""
    
    # Initialize simulator
    simulator = HumanoidSimulator()
    
    # Simulate a simple motion - reaching with left arm
    initial_angles = np.zeros(26)  # 26 DOF for basic humanoid
    
    # Define a simple reaching motion
    reaching_motion = []
    for t in np.linspace(0, 2, int(2/0.01)):  # 2 seconds of motion
        angles = initial_angles.copy()
        
        # Simple reaching motion with left arm
        angles[0] = 0.5 * np.sin(0.5 * t)  # Shoulder joint
        angles[1] = 0.3 * np.sin(0.3 * t)  # Another shoulder joint
        
        reaching_motion.append({
            'time': t,
            'joint_angles': angles
        })
    
    # Run simulation
    results = []
    for motion in reaching_motion:
        result = simulator.simulate_step(motion['joint_angles'])
        results.append(result)
    
    # Visualize results
    simulator.visualize_simulation(results)
    
    print("Humanoid kinematics and dynamics simulation completed!")
    print(f"Simulated {len(results)} time steps")
    print(f"Final CoM position: {results[-1]['com']}")

if __name__ == "__main__":
    main()
```

## Summary

This chapter covered the critical aspects of humanoid robot kinematics and dynamics:

- **Kinematic modeling**: Forward and inverse kinematics for multi-chain systems
- **Dynamic modeling**: Newton-Euler and Lagrangian formulations for force analysis
- **Balance control**: Center of Mass (CoM) and Zero Moment Point (ZMP) concepts
- **Walking patterns**: Generation and control of bipedal locomotion
- **Control strategies**: Operational space control and redundancy resolution
- **Simulation and validation**: Techniques for testing and verifying controllers

Understanding these concepts is essential for developing stable and capable humanoid robots that can perform complex tasks while maintaining balance in dynamic environments.

---

## Exercises

1. Implement forward and inverse kinematics for a 7-DOF humanoid arm.
2. Calculate the center of mass for a simplified humanoid model.
3. Simulate a bipedal walking pattern using the inverted pendulum model.
4. Design a controller that maintains ZMP within the support polygon.
5. Implement operational space control for a humanoid manipulation task.
