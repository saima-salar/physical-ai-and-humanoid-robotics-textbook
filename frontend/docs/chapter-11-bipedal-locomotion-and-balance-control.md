---
title: Chapter 11 — Bipedal Locomotion and Balance Control
description: Advanced techniques for humanoid robot bipedal walking and balance
id: chapter-11-bipedal-locomotion-and-balance-control
sidebar_position: 11
---

import ContentFilter from '@site/src/components/ContentFilter';

# Chapter 11 — Bipedal Locomotion and Balance Control

## Introduction

This chapter focuses on the complex challenges of bipedal locomotion and balance control for humanoid robots. Unlike wheeled or tracked robots, humanoid robots must maintain balance while walking on two legs, making their movement patterns fundamentally different and more challenging to control. This chapter covers the theoretical foundations, control strategies, and practical implementations for achieving stable bipedal walking.

Bipedal locomotion in humanoid robots involves:

- **Dynamic balance**: Maintaining stability during movement
- **Walking pattern generation**: Creating natural gait patterns
- **Balance recovery**: Techniques for recovering from disturbances
- **Terrain adaptation**: Walking on various surfaces and inclines
- **Real-time control**: Fast response to maintain stability

The challenges of bipedal locomotion stem from the inherent instability of the human-like form and the need to replicate the complex neural control mechanisms that humans use for walking.

## Fundamentals of Humanoid Bipedal Locomotion

### Gait Phases

Humanoid bipedal walking consists of distinct phases:

1. **Double Support Phase (DSP)**: Both feet are in contact with the ground
2. **Single Support Phase (SSP)**: Only one foot is in contact with the ground
3. **Swing Phase**: The non-support foot moves forward
4. **Double Support Phase**: The other foot makes contact, preparing for the next step

```python
class GaitPhase:
    DOUBLE_SUPPORT = 0
    SINGLE_SUPPORT_LEFT = 1
    SINGLE_SUPPORT_RIGHT = 2
    SWING_LEFT = 3
    SWING_RIGHT = 4

class GaitAnalyzer:
    def __init__(self, step_time=0.8, double_support_ratio=0.1):
        self.step_time = step_time
        self.dsp_duration = step_time * double_support_ratio
        self.ssp_duration = step_time - 2 * self.dsp_duration  # Two DSP phases per step
        self.current_phase = GaitPhase.DOUBLE_SUPPORT
        self.phase_start_time = 0
        self.step_count = 0
    
    def update_phase(self, current_time):
        """Update gait phase based on time"""
        phase_elapsed = current_time - self.phase_start_time
        
        if self.current_phase == GaitPhase.DOUBLE_SUPPORT:
            if phase_elapsed > self.dsp_duration:
                self.current_phase = GaitPhase.SINGLE_SUPPORT_LEFT
                self.phase_start_time = current_time
        elif self.current_phase == GaitPhase.SINGLE_SUPPORT_LEFT:
            if phase_elapsed > self.ssp_duration:
                self.current_phase = GaitPhase.DOUBLE_SUPPORT
                self.phase_start_time = current_time
                self.step_count += 1
        # Add more phase transitions as needed
        
        return self.current_phase
```

### Key Biomechanical Principles

Humanoid walking follows several key biomechanical principles:

- **Passive Dynamic Walking**: Utilizing gravity and momentum for energy-efficient walking
- **Pendulum Motion**: CoM moves like an inverted pendulum during single support
- **Ground Reaction Forces**: Proper force distribution to maintain stability
- **Angular Momentum**: Controlling rotation to prevent falling

## Balance Control Strategies

### Zero Moment Point (ZMP) Control

ZMP control is fundamental to bipedal stability. The ZMP must remain within the support polygon for stable walking.

```python
import numpy as np
from math import sqrt

class ZMPController:
    def __init__(self, com_height=0.8, gravity=9.81, control_freq=200):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = sqrt(gravity / com_height)
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # ZMP tracking controller gains
        self.kp = 100.0  # Proportional gain
        self.kd = 20.0   # Derivative gain
        
        # ZMP reference trajectory
        self.zmp_reference = np.array([0.0, 0.0])
        self.zmp_error_integral = np.zeros(2)
        
    def calculate_zmp(self, com_pos, com_acc):
        """
        Calculate ZMP from CoM position and acceleration
        ZMP_x = CoM_x - h * CoM_acc_x / g
        ZMP_y = CoM_y - h * CoM_acc_y / g
        """
        zmp_x = com_pos[0] - (self.com_height * com_acc[0]) / self.gravity
        zmp_y = com_pos[1] - (self.com_height * com_acc[1]) / self.gravity
        
        return np.array([zmp_x, zmp_y])
    
    def track_zmp(self, current_zmp, reference_zmp, com_pos, com_vel):
        """Generate CoM adjustment to track desired ZMP"""
        # Calculate ZMP error
        zmp_error = reference_zmp - current_zmp
        
        # PID control for ZMP tracking
        self.zmp_error_integral += zmp_error * self.dt
        
        # Apply control limits
        max_integral = 0.1  # Limit integral windup
        self.zmp_error_integral = np.clip(self.zmp_error_integral, -max_integral, max_integral)
        
        # Calculate required CoM acceleration
        com_acc_cmd_x = (self.kp * zmp_error[0] + 
                        self.kd * (zmp_error[0] - (reference_zmp[0] - current_zmp[0])) / self.dt +
                        self.gravity / self.com_height * (reference_zmp[0] - com_pos[0]))
        
        com_acc_cmd_y = (self.kp * zmp_error[1] + 
                        self.kd * (zmp_error[1] - (reference_zmp[1] - current_zmp[1])) / self.dt +
                        self.gravity / self.com_height * (reference_zmp[1] - com_pos[1]))
        
        return np.array([com_acc_cmd_x, com_acc_cmd_y, 0.0])
    
    def generate_zmp_trajectory(self, step_length=0.3, step_time=0.8, steps=10):
        """Generate ZMP reference trajectory for walking"""
        zmp_trajectory = []
        
        # Simplified walking pattern: ZMP moves from one foot to the other
        for step in range(steps):
            # DSP phase - ZMP moves to next foot position
            dsp_steps = int(self.dt * 0.2 / self.dt)  # 200ms double support
            for i in range(dsp_steps):
                progress = i / dsp_steps
                zmp_x = progress * step_length / 2  # Move toward next foot
                zmp_y = 0.0
                zmp_trajectory.append(np.array([zmp_x, zmp_y]))
            
            # SSP phase - ZMP stays at supporting foot
            ssp_steps = int((step_time - 0.2) / self.dt)  # Remaining time
            for i in range(ssp_steps):
                zmp_trajectory.append(np.array([step_length / 2, 0.0]))
        
        return zmp_trajectory
```

### Capture Point Theory

The Capture Point indicates where the CoM must step to come to a stop.

```python
class CapturePointController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = sqrt(gravity / com_height)
    
    def calculate_capture_point(self, com_pos, com_vel):
        """
        Calculate the capture point where the robot should step to stop
        Capture Point = CoM_position + CoM_velocity/omega
        """
        cp_x = com_pos[0] + com_vel[0] / self.omega
        cp_y = com_pos[1] + com_vel[1] / self.omega
        
        return np.array([cp_x, cp_y])
    
    def is_capturable(self, com_pos, com_vel, foot_pos, margin=0.1):
        """Check if the current state is capturable"""
        capture_point = self.calculate_capture_point(com_pos, com_vel)
        
        # Check if capture point is within reach of foot (with margin)
        distance = np.linalg.norm(capture_point[:2] - foot_pos[:2])
        max_reach = 0.3 + margin  # Typical step reach
        
        return distance <= max_reach
    
    def calculate_next_foot_position(self, com_pos, com_vel, walking_state):
        """Calculate where to place the next foot based on capture point"""
        capture_point = self.calculate_capture_point(com_pos, com_vel)
        
        # Adjust foot position to be capturable
        desired_foot_pos = capture_point.copy()
        
        # Apply constraints based on walking state
        if walking_state == 'forward':
            # Move foot forward but consider capture point
            desired_foot_pos[0] += 0.1  # Add step length bias
        elif walking_state == 'turning':
            # Adjust for turning motion
            pass  # Implementation would depend on turning direction
        
        return desired_foot_pos
```

## Walking Pattern Generation

### Preview Control

Preview control uses future reference trajectory information to improve stability.

```python
class PreviewController:
    def __init__(self, com_height=0.8, gravity=9.81, dt=0.005, preview_steps=200):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = sqrt(gravity / com_height)
        self.dt = dt
        self.preview_steps = preview_steps
        self.preview_horizon = preview_steps * dt
        
        # Initialize preview controller matrices
        self.initialize_preview_matrices()
    
    def initialize_preview_matrices(self):
        """Initialize matrices for preview control"""
        # System matrices for CoM dynamics (linear inverted pendulum model)
        # dx/dt = A*x + B*u + W*w
        # y = C*x
        self.A = np.array([[0, 1], [self.omega**2, 0]])
        self.B = np.array([[0], [-self.omega**2]])
        self.C = np.array([[1, 0]])  # Output is ZMP (CoM position)
        
        # Solve Riccati equation for LQR
        Q = np.array([[100, 0], [0, 1]])  # State weighting matrix
        R = np.array([[1]])               # Control weighting matrix
        
        # Discrete-time system
        dt = self.dt
        A_d = np.eye(2) + self.A * dt + 0.5 * np.dot(self.A, self.A) * dt**2
        B_d = self.B * dt  # Simplified discrete conversion
        
        # For preview control, we also need the preview gain matrix
        # This is a simplified implementation - full preview control is complex
        self.K = np.array([[10.0, 1.0]])  # Simple control gain for demonstration
    
    def generate_reference_trajectory(self, start_pos, goal_pos, total_time):
        """Generate smooth reference trajectory"""
        steps = int(total_time / self.dt)
        trajectory = []
        
        for i in range(steps):
            t = i * self.dt / total_time  # Normalized time [0, 1]
            
            # Cubic interpolation for smooth motion
            blend = 3*t**2 - 2*t**3
            pos_ref = start_pos + (goal_pos - start_pos) * blend
            vel_ref = (goal_pos - start_pos) * (6*t - 6*t**2) / total_time
            
            trajectory.append({'pos': pos_ref, 'vel': vel_ref})
        
        return trajectory
    
    def preview_control_step(self, current_state, reference_trajectory, current_time):
        """Execute one step of preview control"""
        # Get current CoM state [position, velocity]
        com_pos = current_state[0]
        com_vel = current_state[1]
        
        # Calculate tracking error
        ref_idx = min(int(current_time / self.dt), len(reference_trajectory) - 1)
        ref_pos = reference_trajectory[ref_idx]['pos']
        ref_vel = reference_trajectory[ref_idx]['vel']
        
        pos_error = ref_pos - com_pos
        vel_error = ref_vel - com_vel
        
        # Apply preview control law
        # This is a simplified version - full preview control requires more complex mathematics
        control_effort = self.K[0, 0] * pos_error + self.K[0, 1] * vel_error
        
        return control_effort
```

### Walking Pattern Generator

```python
class WalkingPatternGenerator:
    def __init__(self, com_height=0.8, step_height=0.05, step_length=0.3, 
                 step_time=0.8, control_freq=200):
        self.com_height = com_height
        self.step_height = step_height
        self.step_length = step_length
        self.step_time = step_time
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # Initialize ZMP and CoM trajectory generators
        self.zmp_controller = ZMPController(com_height=com_height, control_freq=control_freq)
        self.capture_point_controller = CapturePointController(com_height=com_height)
        self.preview_controller = PreviewController(com_height=com_height, dt=self.dt)
        
        # Walking parameters
        self.walking_speed = 0.0
        self.turning_rate = 0.0
        self.foot_separation = 0.2  # Distance between feet in double support
    
    def generate_walking_trajectory(self, steps=10, walking_speed=0.2, turning_rate=0.0):
        """Generate complete walking trajectory for multiple steps"""
        trajectory = {
            'com_x': [], 'com_y': [], 'com_z': [],
            'left_foot_x': [], 'left_foot_y': [], 'left_foot_z': [],
            'right_foot_x': [], 'right_foot_y': [], 'right_foot_z': [],
            'zmp_x': [], 'zmp_y': [],
            'time': []
        }
        
        # Initialize robot state
        com_pos = np.array([0.0, 0.0, self.com_height])
        com_vel = np.zeros(3)
        com_acc = np.zeros(3)
        
        left_foot_pos = np.array([0.0, self.foot_separation/2, 0.0])
        right_foot_pos = np.array([0.0, -self.foot_separation/2, 0.0])
        
        current_time = 0.0
        step_count = 0
        
        for step in range(int(steps * self.step_time * self.control_freq)):
            # Determine gait phase and update foot positions
            phase = self.get_gait_phase(current_time)
            
            if phase == GaitPhase.DOUBLE_SUPPORT:
                # Both feet on ground, prepare for step
                pass
            elif phase == GaitPhase.SINGLE_SUPPORT_LEFT:
                # Right foot swings forward
                right_foot_pos = self.calculate_swing_trajectory(
                    right_foot_pos, left_foot_pos, current_time, 'right')
            elif phase == GaitPhase.SINGLE_SUPPORT_RIGHT:
                # Left foot swings forward
                left_foot_pos = self.calculate_swing_trajectory(
                    left_foot_pos, right_foot_pos, current_time, 'left')
            
            # Update CoM trajectory based on walking pattern
            com_pos, com_vel, com_acc = self.update_com_dynamics(
                com_pos, com_vel, left_foot_pos, right_foot_pos, current_time)
            
            # Calculate ZMP
            zmp = self.zmp_controller.calculate_zmp(com_pos, com_acc)
            
            # Store trajectory points
            trajectory['com_x'].append(com_pos[0])
            trajectory['com_y'].append(com_pos[1])
            trajectory['com_z'].append(com_pos[2])
            trajectory['left_foot_x'].append(left_foot_pos[0])
            trajectory['left_foot_y'].append(left_foot_pos[1])
            trajectory['left_foot_z'].append(left_foot_pos[2])
            trajectory['right_foot_x'].append(right_foot_pos[0])
            trajectory['right_foot_y'].append(right_foot_pos[1])
            trajectory['right_foot_z'].append(right_foot_pos[2])
            trajectory['zmp_x'].append(zmp[0])
            trajectory['zmp_y'].append(zmp[1])
            trajectory['time'].append(current_time)
            
            current_time += self.dt
        
        return trajectory
    
    def get_gait_phase(self, time):
        """Determine current gait phase based on time"""
        step_phase = (time % self.step_time) / self.step_time
        
        dsp_duration = 0.1  # 10% of step time in double support
        ssp_duration = self.step_time - dsp_duration
        
        if step_phase < dsp_duration / self.step_time:
            # First DSP phase
            return GaitPhase.DOUBLE_SUPPORT
        elif step_phase < (dsp_duration + ssp_duration) / self.step_time:
            # SSP phase - swing foot determined by step count
            if int(time / self.step_time) % 2 == 0:
                return GaitPhase.SINGLE_SUPPORT_LEFT
            else:
                return GaitPhase.SINGLE_SUPPORT_RIGHT
        else:
            # Second DSP phase
            return GaitPhase.DOUBLE_SUPPORT
    
    def calculate_swing_trajectory(self, swing_foot, stance_foot, time, foot_type):
        """Calculate swing foot trajectory"""
        # Determine when this swing phase started
        step_start = (time // self.step_time) * self.step_time
        phase_in_step = time - step_start
        
        # Swing phase is part of the step time
        dsp_duration = 0.1
        ssp_start = step_start + dsp_duration
        swing_duration = (self.step_time - 2*dsp_duration) / 2  # Half for each foot
        
        if foot_type == 'right' and int(time / self.step_time) % 2 == 0:
            # Right foot swinging
            swing_start = ssp_start
        elif foot_type == 'left' and int(time / self.step_time) % 2 == 1:
            # Left foot swinging
            swing_start = ssp_start + (self.step_time - 2*dsp_duration)/2
        else:
            # This foot is not swinging, return current position
            return swing_foot
        
        # Calculate progress in swing phase
        swing_time = time - swing_start
        if swing_time < 0 or swing_time > swing_duration:
            # Not currently swinging this foot
            return swing_foot
        
        progress = min(swing_time / swing_duration, 1.0)
        
        # Calculate swing trajectory (simplified)
        target_x = stance_foot[0] + self.step_length
        target_y = -stance_foot[1]  # Switch side
        target_z = 0.0  # On ground
        
        # Cubic interpolation for smooth motion
        blend = 3*progress**2 - 2*progress**3
        
        # Lateral movement
        if foot_type == 'right':
            new_y = self.foot_separation/2
        else:
            new_y = -self.foot_separation/2
        
        # Forward movement with arc
        new_x = swing_foot[0] + (target_x - swing_foot[0]) * blend
        new_z = self.step_height * np.sin(np.pi * progress)  # Arc motion
        
        return np.array([new_x, new_y, new_z])
    
    def update_com_dynamics(self, com_pos, com_vel, left_foot, right_foot, time):
        """Update CoM position based on walking dynamics"""
        # This is a simplified model - real implementation would be more complex
        # Calculate support polygon and ZMP reference
        support_polygon = self.calculate_support_polygon(left_foot, right_foot)
        
        # Determine desired ZMP based on walking pattern
        desired_zmp = self.calculate_desired_zmp(time, support_polygon)
        
        # Apply ZMP control to update CoM acceleration
        current_zmp = com_pos[:2]  # Simplified assumption
        zmp_error = desired_zmp - current_zmp
        
        # Simple PD control for CoM tracking
        kp = 1.0
        kd = 0.5
        
        com_acc_x = kp * zmp_error[0] - kd * com_vel[0]
        com_acc_y = kp * zmp_error[1] - kd * com_vel[1]
        com_acc_z = 0  # Maintain constant height
        
        com_acc = np.array([com_acc_x, com_acc_y, com_acc_z])
        
        # Integrate to get velocity and position
        com_vel += com_acc * self.dt
        com_pos += com_vel * self.dt + 0.5 * com_acc * self.dt**2
        
        return com_pos, com_vel, com_acc
    
    def calculate_support_polygon(self, left_foot, right_foot):
        """Calculate support polygon from foot positions"""
        # For two feet, the support polygon is the convex hull of both feet
        # Simplified to a rectangle between feet
        center_x = (left_foot[0] + right_foot[0]) / 2
        center_y = (left_foot[1] + right_foot[1]) / 2
        width = abs(left_foot[1] - right_foot[1])  # Distance between feet
        length = self.step_length  # Simplified
        
        # Return corner points of support polygon
        corners = [
            [center_x - length/2, center_y - width/2],
            [center_x - length/2, center_y + width/2],
            [center_x + length/2, center_y + width/2],
            [center_x + length/2, center_y - width/2]
        ]
        
        return corners
    
    def calculate_desired_zmp(self, time, support_polygon):
        """Calculate desired ZMP for stable walking"""
        # For steady walking, ZMP typically follows a pattern
        # Move toward the stance foot during single support
        
        step_number = int(time / self.step_time)
        phase_in_step = (time % self.step_time) / self.step_time
        
        # Simplified ZMP pattern - moves between feet
        if step_number % 2 == 0:
            # Left foot support phase
            return np.array([0, 0.1])  # Near left foot
        else:
            # Right foot support phase
            return np.array([0, -0.1])  # Near right foot
```

## Balance Recovery Strategies

### Disturbance Response

Humanoid robots must respond to unexpected disturbances to maintain balance:

```python
class BalanceRecoveryController:
    def __init__(self, com_height=0.8, max_step_size=0.3, max_angular_velocity=1.0):
        self.com_height = com_height
        self.max_step_size = max_step_size
        self.max_angular_velocity = max_angular_velocity
        
        self.capture_point_controller = CapturePointController(com_height=com_height)
        self.zmp_controller = ZMPController(com_height=com_height)
        
        # Recovery state
        self.in_recovery = False
        self.recovery_start_time = 0
        self.recovery_foot_position = None
        
    def detect_balance_loss(self, com_pos, com_vel, foot_positions, threshold=0.1):
        """Detect when robot is losing balance"""
        # Calculate capture point
        capture_point = self.capture_point_controller.calculate_capture_point(com_pos, com_vel)
        
        # Check if capture point is outside safe region
        # Define safe region around feet
        support_polygon = self.calculate_support_polygon(foot_positions)
        
        # For simplicity, check distance from center of support
        support_center = np.mean(foot_positions, axis=0)[:2]
        distance_to_support = np.linalg.norm(capture_point - support_center)
        
        # If too far, we need to take a recovery step
        is_unstable = distance_to_support > self.max_step_size * 0.7  # 70% of max step
        
        return is_unstable
    
    def calculate_recovery_action(self, com_pos, com_vel, current_foot_positions):
        """Calculate appropriate recovery action"""
        if self.in_recovery:
            # Already in recovery, continue current action
            return self.continue_recovery(com_pos, com_vel, current_foot_positions)
        
        # Check if balance is lost
        is_unstable = self.detect_balance_loss(com_pos, com_vel, current_foot_positions)
        
        if is_unstable:
            self.start_recovery(com_pos, com_vel, current_foot_positions)
            return self.recovery_foot_position
        else:
            return None  # No recovery needed
    
    def start_recovery(self, com_pos, com_vel, current_foot_positions):
        """Start balance recovery sequence"""
        self.in_recovery = True
        self.recovery_start_time = self.get_current_time()
        
        # Calculate where to place recovery foot
        capture_point = self.capture_point_controller.calculate_capture_point(
            com_pos[:2], com_vel[:2])
        
        # Place foot near capture point but within reach
        # Also consider current momentum
        current_support = self.identify_support_foot(current_foot_positions)
        
        # Calculate optimal recovery foot position
        recovery_foot = self.plan_recovery_step(capture_point, current_support)
        self.recovery_foot_position = recovery_foot
        
        print(f"Balance recovery initiated. Moving foot to: {recovery_foot}")
    
    def identify_support_foot(self, foot_positions):
        """Identify which foot is currently supporting"""
        # Simplified - in real implementation, this would use force sensors
        # For now, return the foot that's farthest from the CoM projection
        com_projection = np.array([0, 0])  # Simplified
        distances = [np.linalg.norm(com_projection - foot[:2]) for foot in foot_positions]
        support_foot_idx = np.argmin(distances)
        return foot_positions[support_foot_idx]
    
    def plan_recovery_step(self, capture_point, support_foot):
        """Plan the best recovery step"""
        # Calculate desired step location
        # Move toward capture point but don't overstep
        direction_to_cp = capture_point - support_foot[:2]
        distance_to_cp = np.linalg.norm(direction_to_cp)
        
        if distance_to_cp > self.max_step_size:
            # Scale to maximum step size
            step_direction = direction_to_cp / distance_to_cp
            recovery_foot = support_foot[:2] + step_direction * self.max_step_size
        else:
            # Step directly toward capture point
            recovery_foot = capture_point
        
        # Ensure foot placement is feasible
        recovery_foot = self.constrain_foot_placement(recovery_foot, support_foot)
        
        # Add height for lifting foot during step
        recovery_foot = np.append(recovery_foot, [0.0])  # Ground level
        
        return recovery_foot
    
    def constrain_foot_placement(self, desired_pos, support_foot):
        """Constrain foot placement to feasible locations"""
        # Limit step length
        step_vector = desired_pos - support_foot[:2]
        step_length = np.linalg.norm(step_vector)
        
        if step_length > self.max_step_size:
            step_direction = step_vector / step_length
            desired_pos = support_foot[:2] + step_direction * self.max_step_size
        
        # Add other constraints (joint limits, obstacle avoidance, etc.)
        
        return desired_pos
    
    def continue_recovery(self, com_pos, com_vel, current_foot_positions):
        """Continue ongoing recovery action"""
        current_time = self.get_current_time()
        time_in_recovery = current_time - self.recovery_start_time
        
        # Recovery actions typically take 1-2 seconds
        if time_in_recovery > 2.0:
            # Recovery timeout
            self.end_recovery()
            return None
        
        # Check if balance is restored
        capture_point = self.capture_point_controller.calculate_capture_point(
            com_pos[:2], com_vel[:2])
        
        # Check if capture point is now within safe region
        if np.linalg.norm(capture_point - self.recovery_foot_position[:2]) < 0.1:
            # Balance restored
            self.end_recovery()
            print("Balance recovery successful")
        
        return self.recovery_foot_position
    
    def end_recovery(self):
        """End recovery sequence"""
        self.in_recovery = False
        self.recovery_foot_position = None
    
    def get_current_time(self):
        """Get current simulation time"""
        import time
        return time.time()  # In real implementation, use robot's time source
    
    def calculate_support_polygon(self, foot_positions):
        """Calculate support polygon from foot positions"""
        # Simplified support polygon calculation
        if len(foot_positions) == 2:
            # Two feet - rectangular support area
            center = np.mean(foot_positions, axis=0)[:2]
            width = abs(foot_positions[0][1] - foot_positions[1][1])
            length = 0.3  # Typical foot length
            
            return [
                center + [-length/2, -width/2],
                center + [-length/2, width/2],
                center + [length/2, width/2],
                center + [length/2, -width/2]
            ]
        else:
            return [foot_positions[0][:2]]
```

## Advanced Control Techniques

### Model Predictive Control (MPC)

MPC is highly effective for humanoid balance control due to its ability to handle constraints:

```python
class MPCBalanceController:
    def __init__(self, com_height=0.8, prediction_horizon=20, dt=0.01):
        self.com_height = com_height
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.gravity = 9.81
        
        # MPC problem parameters
        self.state_dim = 4  # [x, y, x_dot, y_dot] for CoM
        self.control_dim = 2  # [zmp_x, zmp_y] or CoM acceleration
        self.Q = np.eye(self.state_dim) * 10  # State cost matrix
        self.R = np.eye(self.control_dim) * 1  # Control cost matrix
        self.P = np.eye(self.state_dim) * 5   # Terminal cost matrix
        
        # Linear inverted pendulum model matrices
        self.A_continuous = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [self.gravity/self.com_height, 0, 0, 0],
            [0, self.gravity/self.com_height, 0, 0]
        ])
        
        self.B_continuous = np.array([
            [0, 0],
            [0, 0],
            [-self.gravity/self.com_height, 0],
            [0, -self.gravity/self.com_height]
        ])
        
        # Discretize system matrices
        self.A, self.B = self.discretize_system()
    
    def discretize_system(self):
        """Discretize continuous system matrices"""
        # Use zero-order hold discretization
        # A_d = I + A*dt + (A*dt)^2/2! + ...
        # For small dt, A_d ≈ I + A*dt
        A_d = np.eye(self.state_dim) + self.A_continuous * self.dt
        B_d = self.B_continuous * self.dt  # Simplified ZOH
        
        return A_d, B_d
    
    def solve_mpc(self, current_state, reference_trajectory):
        """
        Solve MPC optimization problem
        This is a simplified version - real implementation would use 
        quadratic programming
        """
        # For this example, we'll use a simplified approach
        # In practice, this would solve a QP problem
        
        # Predict states over horizon
        predicted_states = []
        current_x = current_state.copy()
        
        for k in range(self.prediction_horizon):
            # Get reference for this step
            ref_idx = min(k, len(reference_trajectory) - 1)
            reference_state = reference_trajectory[ref_idx]
            
            # State feedback control law (simplified)
            # u = -K(x - x_ref)
            state_error = current_x - reference_state
            
            # Simple gain matrix for demonstration
            K = np.array([[1.0, 0, 2.0, 0],    # For x control
                         [0, 1.0, 0, 2.0]])   # For y control
            
            control_input = -K @ state_error
            
            # Apply constraints
            control_input = np.clip(control_input, -0.3, 0.3)  # Limit ZMP deviation
            
            # Update state
            current_x = self.A @ current_x + self.B @ control_input
            
            predicted_states.append(current_x.copy())
        
        # Return first control input
        return control_input
    
    def linear_inverted_pendulum_model(self, com_state, zmp_ref):
        """
        Linear inverted pendulum model for humanoid balance
        dx/dt = A*x + B*u
        where x = [px, py, px_dot, py_dot] (CoM position and velocity)
              u = [px_ref, py_ref] (ZMP reference)
        """
        px, py, px_dot, py_dot = com_state
        zmp_x_ref, zmp_y_ref = zmp_ref
        
        # LIPM dynamics
        px_ddot = self.gravity / self.com_height * (px - zmp_x_ref)
        py_ddot = self.gravity / self.com_height * (py - zmp_y_ref)
        
        return np.array([px_dot, py_dot, px_ddot, py_ddot])
    
    def integrate_lipm(self, state, zmp_input, dt):
        """Integrate LIPM dynamics"""
        derivatives = self.linear_inverted_pendulum_model(state, zmp_input)
        return state + derivatives * dt
    
    def generate_balance_trajectory(self, start_state, target_state, duration=2.0):
        """Generate smooth trajectory for balance control"""
        steps = int(duration / self.dt)
        trajectory = []
        
        for i in range(steps):
            t = i * self.dt / duration  # Normalized time [0, 1]
            
            # Interpolate between start and target states
            blend = 3*t**2 - 2*t**3  # Smooth interpolation
            state = start_state * (1 - blend) + target_state * blend
            
            trajectory.append(state)
        
        return trajectory
```

## Hardware Implementation Considerations

### Joint Control for Balance

Balance control requires precise joint control:

```python
import time

class JointBalanceController:
    def __init__(self, joint_names, control_freq=200):
        self.joint_names = joint_names
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # PID controllers for each joint
        self.pid_controllers = {}
        for joint in joint_names:
            self.pid_controllers[joint] = PIDController(
                kp=100.0, ki=10.0, kd=10.0, dt=self.dt
            )
        
        # Joint limits and safety constraints
        self.joint_limits = self.initialize_joint_limits()
    
    def initialize_joint_limits(self):
        """Initialize joint limits for safety"""
        # Example for a humanoid robot
        limits = {}
        for joint in self.joint_names:
            if 'hip' in joint:
                limits[joint] = (-1.57, 1.57)  # ±90 degrees
            elif 'knee' in joint:
                limits[joint] = (0, 2.35)      # 0 to 135 degrees
            elif 'ankle' in joint:
                limits[joint] = (-0.5, 0.5)    # ±28 degrees
            elif 'shoulder' in joint:
                limits[joint] = (-2.0, 2.0)    # ±114 degrees
            else:
                limits[joint] = (-3.14, 3.14)  # ±180 degrees (safest default)
        
        return limits
    
    def compute_balance_torques(self, desired_joint_positions, current_joint_positions,
                               current_joint_velocities, com_feedback_torques):
        """Compute joint torques for balance control"""
        joint_torques = {}
        
        # Apply position control for each joint
        for joint in self.joint_names:
            desired_pos = desired_joint_positions.get(joint, 0.0)
            current_pos = current_joint_positions.get(joint, 0.0)
            current_vel = current_joint_velocities.get(joint, 0.0)
            
            # Compute position error
            pos_error = desired_pos - current_pos
            
            # Apply PID control
            torque = self.pid_controllers[joint].update(pos_error, current_vel)
            
            # Add balance feedback (simplified)
            if joint in com_feedback_torques:
                torque += com_feedback_torques[joint]
            
            # Apply joint limits
            torque = self.apply_joint_constraints(joint, torque, current_pos)
            
            joint_torques[joint] = torque
        
        return joint_torques
    
    def apply_joint_constraints(self, joint, torque, position):
        """Apply joint constraints to torques"""
        # Apply joint position limits
        min_pos, max_pos = self.joint_limits[joint]
        
        # Reduce torque if near limits
        if position < min_pos + 0.1:  # 0.1 rad from limit
            torque = min(torque, 0)  # Only allow movement away from limit
        elif position > max_pos - 0.1:
            torque = max(torque, 0)  # Only allow movement away from limit
        
        # Limit maximum torque (simplified)
        max_torque = 50.0  # N*m (example value)
        torque = max(min(torque, max_torque), -max_torque)
        
        return torque

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, derivative=None):
        """Update PID controller"""
        # Calculate derivative if not provided
        if derivative is None:
            derivative = (error - self.prev_error) / self.dt
        
        # Update integral with anti-windup
        self.integral += error * self.dt
        max_integral = 10.0
        self.integral = max(min(self.integral, max_integral), -max_integral)
        
        # Calculate output
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Store values for next iteration
        self.prev_error = error
        
        return output
```

## Simulation and Testing

### Complete Walking Controller

```python
class CompleteWalkingController:
    def __init__(self, com_height=0.8):
        self.com_height = com_height
        self.zmp_controller = ZMPController(com_height=com_height)
        self.capture_point_controller = CapturePointController(com_height=com_height)
        self.walking_pattern_gen = WalkingPatternGenerator(com_height=com_height)
        self.balance_recovery = BalanceRecoveryController(com_height=com_height)
        self.mpc_controller = MPCBalanceController(com_height=com_height)
        self.joint_controller = JointBalanceController(
            joint_names=['left_hip', 'left_knee', 'left_ankle', 
                        'right_hip', 'right_knee', 'right_ankle']
        )
        
        # Robot state
        self.com_pos = np.array([0.0, 0.0, com_height])
        self.com_vel = np.zeros(3)
        self.com_acc = np.zeros(3)
        self.left_foot_pos = np.array([0.0, 0.1, 0.0])
        self.right_foot_pos = np.array([0.0, -0.1, 0.0])
        
        self.current_time = 0.0
        self.control_freq = 200  # Hz
        self.dt = 1.0 / self.control_freq
    
    def update_walking_control(self, walking_command, sensor_data):
        """
        Main control function for walking
        """
        # Update robot state from sensors
        self.update_robot_state(sensor_data)
        
        # Check for balance recovery needs
        recovery_action = self.balance_recovery.calculate_recovery_action(
            self.com_pos, self.com_vel, [self.left_foot_pos, self.right_foot_pos]
        )
        
        if recovery_action is not None:
            # Execute balance recovery
            return self.execute_balance_recovery(recovery_action)
        
        # Normal walking control
        if walking_command['mode'] == 'walk':
            # Generate walking pattern based on command
            desired_zmp = self.calculate_walking_zmp(walking_command)
            
            # Control CoM to track desired ZMP
            com_control = self.zmp_controller.track_zmp(
                self.current_zmp(), desired_zmp, self.com_pos[:2], self.com_vel[:2]
            )
            
            # Convert to joint commands
            joint_commands = self.compute_joint_commands(com_control, walking_command)
            
            return joint_commands
        
        return self.stand_still()
    
    def update_robot_state(self, sensor_data):
        """Update robot state from sensor data"""
        # This would process IMU, joint encoders, force sensors, etc.
        # For simulation:
        self.com_pos = sensor_data.get('com_position', self.com_pos)
        self.com_vel = sensor_data.get('com_velocity', self.com_vel)
        self.left_foot_pos = sensor_data.get('left_foot_position', self.left_foot_pos)
        self.right_foot_pos = sensor_data.get('right_foot_position', self.right_foot_pos)
    
    def current_zmp(self):
        """Calculate current ZMP"""
        return self.zmp_controller.calculate_zmp(self.com_pos, self.com_acc)
    
    def calculate_walking_zmp(self, walking_command):
        """Calculate desired ZMP for walking"""
        # Based on walking speed and direction
        speed = walking_command.get('speed', 0.0)
        direction = walking_command.get('direction', [1, 0])  # Forward by default
        turning = walking_command.get('turning', 0.0)  # Turning rate
        
        # Generate ZMP trajectory based on command
        # This is simplified - real implementation would be more complex
        desired_zmp = np.array([0.1, 0.0])  # Slightly forward for forward walking
        
        return desired_zmp
    
    def execute_balance_recovery(self, recovery_foot_pos):
        """Execute balance recovery action"""
        print(f"Executing balance recovery to position: {recovery_foot_pos}")
        
        # This would implement the full recovery sequence:
        # 1. Plan step to recovery location
        # 2. Execute step with appropriate joint trajectories
        # 3. Stabilize on new foot placement
        
        # For now, return appropriate joint commands
        return {'left_hip': 0.1, 'left_knee': -0.1, 'left_ankle': 0.0,
                'right_hip': 0.1, 'right_knee': -0.1, 'right_ankle': 0.0}
    
    def compute_joint_commands(self, com_control, walking_command):
        """Convert CoM control to joint commands"""
        # This would implement inverse kinematics and whole-body control
        # For now, return example joint commands
        
        # Simple example based on CoM control
        hip_roll = com_control[1] * 0.1  # Lateral CoM control -> hip roll
        hip_pitch = com_control[0] * 0.05  # Forward CoM control -> hip pitch
        
        joint_commands = {
            'left_hip': hip_pitch + hip_roll,
            'left_knee': 0.3 - com_control[0] * 0.1,  # Adjust knee based on forward control
            'left_ankle': -hip_roll,  # Ankle opposes hip roll
            'right_hip': hip_pitch - hip_roll,
            'right_knee': 0.3 - com_control[0] * 0.1,
            'right_ankle': hip_roll
        }
        
        return joint_commands
    
    def stand_still(self):
        """Return joint commands for standing still"""
        return {'left_hip': 0.0, 'left_knee': 0.0, 'left_ankle': 0.0,
                'right_hip': 0.0, 'right_knee': 0.0, 'right_ankle': 0.0}

def main():
    """Main function to demonstrate bipedal control"""
    controller = CompleteWalkingController()
    
    # Simulate walking for a few seconds
    print("Starting bipedal walking simulation...")
    
    # Walking command
    walking_cmd = {
        'mode': 'walk',
        'speed': 0.2,  # 0.2 m/s
        'direction': [1, 0],  # Forward
        'turning': 0.0
    }
    
    # Simulate sensor data (in real robot, this comes from actual sensors)
    sensor_data = {
        'com_position': np.array([0.0, 0.0, 0.8]),
        'com_velocity': np.array([0.0, 0.0, 0.0]),
        'left_foot_position': np.array([0.0, 0.1, 0.0]),
        'right_foot_position': np.array([0.0, -0.1, 0.0])
    }
    
    # Run control loop
    for t in range(1000):  # 5 seconds at 200 Hz
        joint_commands = controller.update_walking_control(walking_cmd, sensor_data)
        
        if t % 100 == 0:  # Print every 0.5 seconds
            print(f"Time: {t*controller.dt:.2f}s, Joint Commands: {joint_commands}")
    
    print("Walking simulation completed!")

if __name__ == "__main__":
    main()
