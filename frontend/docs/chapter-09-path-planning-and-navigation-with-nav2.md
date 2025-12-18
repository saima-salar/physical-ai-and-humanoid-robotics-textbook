---
title: Chapter 9 — Path Planning and Navigation with Nav2
description: Advanced navigation for bipedal humanoid movement using Nav2
id: chapter-09-path-planning-and-navigation-with-nav2
sidebar_position: 9
---

import ContentFilter from '@site/src/components/ContentFilter';

# Chapter 9 — Path Planning and Navigation with Nav2

## Introduction

This chapter focuses on navigation and path planning for humanoid robots using the Navigation2 (Nav2) framework. Nav2 is the next-generation navigation system for ROS 2, specifically designed for mobile robots operating in dynamic environments. For humanoid robots, navigation presents unique challenges due to their bipedal locomotion and human-scale dimensions.

Nav2 provides:

- **Advanced path planning algorithms**: A*, Dijkstra, and other sophisticated planners
- **Dynamic obstacle avoidance**: Real-time navigation around moving obstacles
- **Global and local planners**: Long-term route planning and short-term obstacle avoidance
- **Recovery behaviors**: Automated strategies for unblocking stuck robots
- **Behavior trees**: Composable navigation behaviors
- **Simulation tools**: Extensive tools for testing and validation

This chapter will cover Nav2's architecture, implementation for humanoid robots, and integration with Isaac ROS for enhanced navigation capabilities.

## Navigation2 Architecture

### Nav2 System Overview

Navigation2 follows a modular architecture that separates different navigation concerns:

```
+---------------------+
|   Navigation API    |  <- High-level interface
+---------------------+
|   Behavior Trees    |  <- Composable behaviors
+---------------------+
|  Task Dispatcher   |  <- Coordinates navigation tasks
+---------------------+
|     Planners       |  <- Global and local planners
+---------------------+
|   Controllers      |  <- Robot motion controllers
+---------------------+
|    Recovery        |  <- Behavior recovery
+---------------------+
|    Sensors         |  <- Perception inputs
+---------------------+
```

### Key Components

The Nav2 system consists of several key components:

1. **Navigation Server**: Main orchestrator of navigation tasks
2. **Global Planner**: Computes optimal paths from start to goal
3. **Local Planner**: Tracks global path while avoiding obstacles
4. **Controller**: Translates navigation commands to robot actuators
5. **Recovery Manager**: Handles navigation failures
6. **Map Server**: Provides static and costmap information
7. **Lifecycle Manager**: Manages component lifecycles

### Behavior Trees in Nav2

Nav2 uses behavior trees to compose navigation behaviors:

```xml
<!-- Example Nav2 Behavior Tree -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RateController>
            <FollowPath path="{path}" controller_id="FollowPath" />
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Global Path Planning

### Planning Algorithms

Nav2 includes several global planning algorithms optimized for different scenarios:

- **A* (A-star)**: Best first search with heuristic, optimal for static environments
- **Dijkstra**: Uninformed search, optimal but slower than A*
- **Lazy Theta***: Any-angle path planning, smoother paths
- **NavFn**: Gradient-based planner, fast for large maps
- **Global Planner**: Alternative implementation with more parameters

### Global Planner for Humanoid Robots

Humanoid robots require specialized path planning considerations:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer
import numpy as np

class HumanoidGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('humanoid_global_planner')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_plan', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)
        
        # Navigation parameters specific to humanoid robots
        self.humanoid_width = 0.6  # Width of humanoid base (bipedal stance)
        self.humanoid_length = 0.4  # Length of humanoid base
        self.min_turning_radius = 0.3  # Minimum turning radius for stability
        
        # Costmap and map data
        self.costmap = None
        self.map_resolution = 0.05  # 5cm per cell
        self.map_origin = [0.0, 0.0, 0.0]  # x, y, theta
        
        # A* path planning components
        self.open_set = []
        self.closed_set = set()
        
        self.get_logger().info('Humanoid Global Planner initialized')
    
    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.orientation.z  # Simplified for demo
        ]
        self.map_data = np.array(msg.data).reshape(self.map_height, self.map_width)
    
    def costmap_callback(self, msg):
        """Process costmap with additional humanoid-specific costs"""
        costmap_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
        # Apply humanoid-specific cost adjustments
        # Consider robot dimensions and stability factors
        costmap_data = self.apply_humanoid_cost_adjustments(costmap_data, msg.info)
        
        self.costmap = costmap_data
        self.costmap_info = msg.info
    
    def apply_humanoid_cost_adjustments(self, costmap, info):
        """Apply costs specific to humanoid robot navigation"""
        # Increase costs for areas that are too narrow for humanoid passage
        # Humanoid robots need wider passages than wheeled robots
        adjusted_costmap = costmap.copy()
        
        # Calculate required passage width based on humanoid dimensions
        min_passage_width = max(self.humanoid_width, self.humanoid_length)
        min_passage_cells = int(np.ceil(min_passage_width / info.resolution))
        
        # Identify narrow passages and increase their cost
        for i in range(info.height - min_passage_cells):
            for j in range(info.width - min_passage_cells):
                region = costmap[i:i+min_passage_cells, j:j+min_passage_cells]
                if np.any(region > 80):  # High cost obstacle
                    # Check if region is traversable for humanoid
                    free_cells = np.sum(region < 50)  # Free space threshold
                    if free_cells < min_passage_cells * min_passage_cells * 0.3:  # 30% free
                        # Mark this area as high cost for humanoid
                        adjusted_costmap[i:i+min_passage_cells, j:j+min_passage_cells] = 100
        
        return adjusted_costmap
    
    def plan_path(self, start_pose, goal_pose):
        """Plan path from start to goal for humanoid robot"""
        # Convert poses to map coordinates
        start_map = self.world_to_map(start_pose.pose.position.x, start_pose.pose.position.y)
        goal_map = self.world_to_map(goal_pose.pose.position.x, goal_pose.pose.position.y)
        
        if self.costmap is None:
            self.get_logger().error('Costmap not available for path planning')
            return None
        
        # Run A* algorithm with humanoid-specific constraints
        path = self.a_star_search(start_map, goal_map)
        
        if path is not None:
            # Convert path back to world coordinates
            world_path = self.map_path_to_world_path(path)
            return world_path
        else:
            self.get_logger().warn('No valid path found')
            return None
    
    def a_star_search(self, start, goal):
        """A* path finding algorithm adapted for humanoid robots"""
        # Implementation of A* algorithm with humanoid constraints
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        import heapq
        open_heap = open_set
        
        while open_heap:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_heap)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            # Explore neighbors
            for neighbor in self.get_valid_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    heapq.heappush(open_heap, (f_score[neighbor], neighbor))
        
        return None  # No path found
    
    def get_valid_neighbors(self, pos):
        """Get valid neighbors for humanoid robot movement"""
        # Consider humanoid stability constraints
        x, y = pos
        neighbors = []
        
        # 8-connectivity neighbors (allowing diagonal movement)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current cell
                    
                nx, ny = x + dx, y + dy
                
                # Check bounds
                if 0 <= nx < self.costmap.shape[1] and 0 <= ny < self.costmap.shape[0]:
                    # Check if cell is traversable for humanoid
                    if self.is_traversable_humanoid(nx, ny):
                        neighbors.append((nx, ny))
        
        return neighbors
    
    def is_traversable_humanoid(self, x, y):
        """Check if cell is traversable for humanoid robot"""
        # Check main cell
        if self.costmap[y, x] > 80:  # High cost / obstacle
            return False
        
        # For humanoid robots, check area around the cell for stability
        # considering the robot's footprint
        robot_cells = int(np.ceil(self.humanoid_width / self.map_resolution))
        half_size = robot_cells // 2
        
        for dx in range(-half_size, half_size + 1):
            for dy in range(-half_size, half_size + 1):
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.costmap.shape[1] and 
                    0 <= ny < self.costmap.shape[0]):
                    if self.costmap[ny, nx] > 80:
                        return False
        
        return True
    
    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def distance(self, a, b):
        """Distance between two points"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def world_to_map(self, x_world, y_world):
        """Convert world coordinates to map coordinates"""
        if not hasattr(self, 'map_origin'):
            return (0, 0)
            
        x_map = int((x_world - self.map_origin[0]) / self.map_resolution)
        y_map = int((y_world - self.map_origin[1]) / self.map_resolution)
        
        return (x_map, y_map)
    
    def map_path_to_world_path(self, map_path):
        """Convert map path to world path"""
        world_path = Path()
        world_path.header.frame_id = 'map'
        world_path.header.stamp = self.get_clock().now().to_msg()
        
        for x_map, y_map in map_path:
            x_world = x_map * self.map_resolution + self.map_origin[0]
            y_world = y_map * self.map_resolution + self.map_origin[1]
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x_world
            pose.pose.position.y = y_world
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation for path points
            
            world_path.poses.append(pose)
        
        return world_path

def main(args=None):
    rclpy.init(args=args)
    
    planner_node = HumanoidGlobalPlannerNode()
    
    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        planner_node.get_logger().info('Shutting down Humanoid Global Planner...')
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Local Path Planning and Control

### Local Planner for Humanoid Locomotion

Bipedal locomotion requires specialized local planning that considers balance and gait:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
import numpy as np

class HumanoidLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('humanoid_local_planner')
        
        # Subscribers
        self.global_path_sub = self.create_subscription(
            Path, '/humanoid_plan', self.global_path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/pointcloud', self.pointcloud_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.velocity_markers_pub = self.create_publisher(MarkerArray, '/velocity_vectors', 10)
        
        # Robot parameters for humanoid
        self.robot_radius = 0.3  # Effective radius for collision checking
        self.max_linear_vel = 0.5  # m/s - conservative for stability
        self.max_angular_vel = 0.6  # rad/s
        self.min_linear_vel = 0.1  # Minimum stable walking speed
        self.min_turn_radius = 0.2  # Minimum turning radius for bipedal stability
        
        # Navigation state
        self.current_pose = None
        self.current_twist = None
        self.global_path = None
        self.path_index = 0
        self.local_path = []  # Path points for local planning
        
        # Local planner parameters
        self.lookahead_dist = 1.0  # Lookahead distance in meters
        self.controller_frequency = 10.0  # Hz
        self.time_step = 1.0 / self.controller_frequency
        
        # Create navigation timer
        self.nav_timer = self.create_timer(
            self.time_step, self.navigation_control_loop)
        
        # Initialize DWA (Dynamic Window Approach) parameters
        self.initialize_dwa_parameters()
        
        self.get_logger().info('Humanoid Local Planner initialized')
    
    def initialize_dwa_parameters(self):
        """Initialize Dynamic Window Approach parameters for humanoid"""
        # DWA parameters adapted for humanoid stability
        self.dwa_params = {
            'max_vel_x': self.max_linear_vel,
            'min_vel_x': self.min_linear_vel,
            'max_vel_theta': self.max_angular_vel,
            'min_vel_theta': -self.max_angular_vel,
            'max_inflation': 0.5,  # Max inflation for safety
            'xy_goal_tolerance': 0.3,  # Goal tolerance
            'yaw_goal_tolerance': 0.1,  # Angle tolerance
            'sim_time': 2.0,  # Simulation time for trajectory evaluation
            'vx_samples': 10,  # Number of velocity samples in x
            'vtheta_samples': 20,  # Number of angular velocity samples
            'path_distance_bias': 0.6,  # Weight for path following
            'goal_distance_bias': 0.8,  # Weight for goal approach
            'obstacle_cost_mult': 2.0,  # Weight for obstacle avoidance
            'humanoid_stability_factor': 1.2  # Additional factor for humanoid stability
        }
    
    def global_path_callback(self, msg):
        """Process global path from global planner"""
        self.global_path = msg
        self.path_index = 0  # Reset to beginning of path
        self.get_logger().info(f'Received global path with {len(msg.poses)} waypoints')
    
    def odom_callback(self, msg):
        """Process odometry information"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        # Convert laser scan to local obstacle representation
        # For humanoid robots, pay attention to lower-body obstacles
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # Filter out invalid range readings
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        self.scan_angles = angles[valid_mask]
        self.scan_ranges = ranges[valid_mask]
    
    def pointcloud_callback(self, msg):
        """Process 3D point cloud data"""
        # Extract obstacle information from point cloud
        # Important for humanoid robots that need to consider obstacles at different heights
        import sensor_msgs.point_cloud2 as pc2
        
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.pointcloud_obstacles = np.array(points) if points else np.array([])
    
    def navigation_control_loop(self):
        """Main navigation control loop"""
        if (self.current_pose is None or 
            self.global_path is None or 
            len(self.global_path.poses) == 0):
            return
        
        # Compute local path based on global path and current position
        local_path = self.compute_local_path()
        
        # Run local planner to avoid obstacles and follow path
        cmd_vel = self.local_plan(local_path)
        
        # Publish command velocity
        if cmd_vel is not None:
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Publish visualization
            self.publish_local_path(local_path)
            self.publish_velocity_vector(cmd_vel)
    
    def compute_local_path(self):
        """Compute local path segment from global path"""
        if self.global_path is None:
            return []
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Find closest point on global path
        closest_idx = self.find_closest_waypoint(current_x, current_y)
        
        # Create local path with points ahead of current position
        local_path = []
        for i in range(closest_idx, min(closest_idx + 10, len(self.global_path.poses))):
            wp = self.global_path.poses[i]
            local_path.append((wp.pose.position.x, wp.pose.position.y))
        
        return local_path
    
    def find_closest_waypoint(self, x, y):
        """Find the closest waypoint on the global path"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return 0
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(self.global_path.poses):
            dist = np.sqrt((pose.pose.position.x - x)**2 + (pose.pose.position.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
    
    def local_plan(self, local_path):
        """Local planning using DWA adapted for humanoid robots"""
        if not local_path:
            return self.stop_robot()
        
        # Get current robot state
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Current velocities
        v = self.current_twist.linear.x if self.current_twist else 0.0
        omega = self.current_twist.angular.z if self.current_twist else 0.0
        
        # Define dynamic window (feasible velocities)
        vs = self.calc_dynamic_window(v, omega)
        
        # Evaluate trajectories in dynamic window
        best_traj = None
        best_score = float('-inf')
        
        for v_sample in np.linspace(vs[0], vs[1], self.dwa_params['vx_samples']):
            for omega_sample in np.linspace(vs[2], vs[3], self.dwa_params['vtheta_samples']):
                # Predict trajectory
                traj = self.predict_trajectory(x, y, theta, v_sample, omega_sample)
                
                # Evaluate trajectory
                score = self.evaluate_trajectory(traj, local_path)
                
                if score > best_score:
                    best_score = score
                    best_traj = (v_sample, omega_sample)
        
        if best_traj is not None:
            v_cmd, omega_cmd = best_traj
            cmd_vel = Twist()
            cmd_vel.linear.x = v_cmd
            cmd_vel.angular.z = omega_cmd
            
            # Add humanoid-specific constraints
            cmd_vel = self.apply_humanoid_constraints(cmd_vel)
            
            return cmd_vel
        else:
            return self.stop_robot()
    
    def calc_dynamic_window(self, v, omega):
        """Calculate dynamic window of feasible velocities"""
        # Dynamic window based on current velocity and acceleration limits
        dt = self.time_step
        
        vs = [
            max(0, v - 0.5 * dt),  # Min velocity (deceleration limit)
            min(self.max_linear_vel, v + 0.5 * dt),  # Max velocity (acceleration limit)
            max(-self.max_angular_vel, omega - 1.0 * dt),  # Min omega (angular decel)
            min(self.max_angular_vel, omega + 1.0 * dt)   # Max omega (angular accel)
        ]
        
        return vs
    
    def predict_trajectory(self, x, y, theta, v, omega):
        """Predict trajectory given velocity commands"""
        dt = self.time_step
        traj = []
        
        # Simple kinematic model for prediction
        current_x, current_y, current_theta = x, y, theta
        
        for t in np.arange(0, self.dwa_params['sim_time'], dt):
            # Update position based on velocity command
            current_x += v * np.cos(current_theta) * dt
            current_y += v * np.sin(current_theta) * dt
            current_theta += omega * dt
            
            traj.append((current_x, current_y))
        
        return traj
    
    def evaluate_trajectory(self, traj, local_path):
        """Evaluate trajectory based on multiple criteria"""
        if not traj:
            return float('-inf')
        
        # Calculate obstacle cost
        obstacle_cost = self.calc_obstacle_cost(traj)
        
        # Calculate path following cost
        path_cost = self.calc_path_cost(traj, local_path)
        
        # Calculate goal approach cost
        goal_cost = self.calc_goal_cost(traj[-1]) if traj else 0
        
        # Weighted sum of costs
        total_cost = (
            self.dwa_params['path_distance_bias'] * path_cost +
            self.dwa_params['goal_distance_bias'] * goal_cost +
            self.dwa_params['obstacle_cost_mult'] * obstacle_cost
        )
        
        return -total_cost  # Return negative cost as score
    
    def calc_obstacle_cost(self, traj):
        """Calculate cost based on proximity to obstacles"""
        if not hasattr(self, 'scan_ranges') or len(self.scan_ranges) == 0:
            return 0  # No obstacles detected
        
        min_dist = float('inf')
        
        for point in traj:
            # Check distance to obstacles from pointcloud
            if hasattr(self, 'pointcloud_obstacles') and len(self.pointcloud_obstacles) > 0:
                dists = np.sqrt(
                    np.sum((self.pointcloud_obstacles[:, :2] - np.array([point[0], point[1]]))**2, axis=1)
                )
                if len(dists) > 0:
                    min_dist = min(min_dist, np.min(dists))
            else:
                # Use laser scan approximation
                # Convert trajectory point to polar coordinates relative to robot
                rel_x = point[0] - self.current_pose.position.x
                rel_y = point[1] - self.current_pose.position.y
                
                angle_to_point = np.arctan2(rel_y, rel_x)
                
                # Find corresponding laser scan range
                angle_diffs = np.abs(self.scan_angles - angle_to_point)
                closest_idx = np.argmin(angle_diffs)
                
                if angle_diffs[closest_idx] < 0.2:  # 0.2 rad = ~11 degrees tolerance
                    dist_to_obstacle = self.scan_ranges[closest_idx]
                    min_dist = min(min_dist, dist_to_obstacle)
        
        # Return high cost if too close to obstacles
        if min_dist < self.robot_radius * 1.5:  # Safety margin
            return 1000
        elif min_dist < self.robot_radius * 2.5:
            return 100 / (min_dist + 0.1)  # Inverse relationship
        else:
            return 0  # No significant obstacle cost
    
    def calc_path_cost(self, traj, local_path):
        """Calculate cost based on deviation from local path"""
        if not local_path:
            return float('inf')
        
        total_error = 0
        num_points = 0
        
        for point in traj:
            # Find closest point on local path
            min_dist = float('inf')
            for path_point in local_path:
                dist = np.sqrt((point[0] - path_point[0])**2 + (point[1] - path_point[1])**2)
                min_dist = min(min_dist, dist)
            
            total_error += min_dist
            num_points += 1
        
        if num_points > 0:
            return total_error / num_points
        else:
            return float('inf')
    
    def calc_goal_cost(self, last_point):
        """Calculate cost based on distance to goal"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return float('inf')
        
        goal = self.global_path.poses[-1]
        dist_to_goal = np.sqrt(
            (last_point[0] - goal.pose.position.x)**2 + 
            (last_point[1] - goal.pose.position.y)**2
        )
        
        return dist_to_goal
    
    def apply_humanoid_constraints(self, cmd_vel):
        """Apply humanoid-specific constraints to velocity commands"""
        # For bipedal humanoid, ensure stability by limiting velocities
        cmd_vel.linear.x = max(
            self.min_linear_vel, 
            min(cmd_vel.linear.x, self.max_linear_vel)
        )
        
        cmd_vel.angular.z = max(
            -self.max_angular_vel, 
            min(cmd_vel.angular.z, self.max_angular_vel)
        )
        
        # Humanoid-specific constraints for smoother transitions
        # Reduce maximum velocities for complex maneuvers
        if abs(cmd_vel.angular.z) > 0.4:  # Turning significantly
            cmd_vel.linear.x = min(cmd_vel.linear.x, 0.3)  # Slow down when turning
        
        return cmd_vel
    
    def stop_robot(self):
        """Return stop command"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel
    
    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        import math
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_local_path(self, local_path):
        """Publish local path for visualization"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in local_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.local_plan_pub.publish(path_msg)
    
    def publish_velocity_vector(self, cmd_vel):
        """Publish velocity vector for visualization"""
        # Implementation for publishing visualization markers
        pass

def main(args=None):
    rclpy.init(args=args)
    
    local_planner = HumanoidLocalPlannerNode()
    
    try:
        rclpy.spin(local_planner)
    except KeyboardInterrupt:
        local_planner.get_logger().info('Shutting down Humanoid Local Planner...')
    finally:
        local_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Isaac ROS Navigation

### Isaac ROS Navigation Components

Isaac ROS enhances Nav2 with GPU-accelerated capabilities:

- **GPU-accelerated perception**: Enhanced obstacle detection
- **Visual-inertial odometry**: More accurate localization
- **Point cloud processing**: Accelerated 3D mapping
- **Deep learning navigation**: Learning-based navigation behaviors

### Isaac ROS Navigation Stack Configuration

```yaml
# config/isaac_nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: 
      "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: 
      "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_horizon: 1.0
      dt: 0.1
      vx_samples: 20
      vy_samples: 5
      theta_samples: 20
      lambda: 0.05
      mu: 0.05
      collision_cost: 1.0
      goal_cost: 1.0
      path_cost: 1.0
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.15
      max_linear_speed: 0.5
      max_angular_speed: 0.6
      min_linear_speed: 0.1
      critical_obstacle_distance: 0.5
      critical_obstacle_angle: 1.57
      transform_tolerance: 0.1
      # Humanoid-specific parameters
      stability_factor: 1.2
      step_size_limit: 0.1

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      origin_x: -3.0
      origin_y: -3.0
      # Humanoid-specific inflation
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan pointcloud
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Humanoid consideration
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
        pointcloud:
          topic: /pointcloud
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.5  # Humanoid buffer zone
        cost_scaling_factor: 3.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.35  # Humanoid effective radius
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: True
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        inflation_radius: 0.7  # Humanoid consideration
        cost_scaling_factor: 3.0
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      humanoid_width: 0.6
      robot_model_radius: 0.35
