#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations

class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')
        
        # MPPI Parameters
        self.declare_parameter('num_samples', 1000)
        self.declare_parameter('horizon', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('lambda_', 1.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'world')
        
        self.num_samples = self.get_parameter('num_samples').value
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.lambda_ = self.get_parameter('lambda_').value
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        
        # Cost weights
        self.w_goal = 10.0
        self.w_obstacle = 100.0
        self.w_control = 0.1
        self.w_smoothness = 0.5
        
        # State
        self.current_pose = None
        self.goal = None
        self.costmap = None
        self.costmap_info = None
        
        # Control sequence (warmstart)
        self.control_sequence = np.zeros((self.horizon, 2))
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap',
            self.costmap_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rollouts_pub = self.create_publisher(MarkerArray, '/mppi_rollouts', 10)
        self.best_traj_pub = self.create_publisher(Marker, '/mppi_best_trajectory', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('MPPI Controller initialized')
        self.get_logger().info(f'Samples: {self.num_samples}, Horizon: {self.horizon}, dt: {self.dt}')
        
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.pose.orientation),
            'v': msg.twist.twist.linear.x,
            'w': msg.twist.twist.angular.z
        }
    
    def goal_callback(self, msg):
        """Update goal from published goal pose"""
        self.goal = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': self.quat_to_yaw(msg.pose.orientation)
        }
        self.get_logger().info(f'New goal: x={self.goal["x"]:.2f}, y={self.goal["y"]:.2f}')
    
    def costmap_callback(self, msg):
        """Update local costmap"""
        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.costmap_info = msg.info
    
    def control_loop(self):
        """Main MPPI control loop"""
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return
        
        if self.goal is None:
            self.get_logger().warn('Waiting for goal...', throttle_duration_sec=2.0)
            return
        
        # Check if goal reached
        dist_to_goal = np.sqrt(
            (self.goal['x'] - self.current_pose['x'])**2 + 
            (self.goal['y'] - self.current_pose['y'])**2
        )
        
        if dist_to_goal < 0.3:
            self.get_logger().info('Goal reached!', throttle_duration_sec=1.0)
            self.publish_zero_velocity()
            return
        
        # Run MPPI
        best_control = self.mppi_step()
        
        # Publish control
        cmd = Twist()
        cmd.linear.x = float(best_control[0])
        cmd.angular.z = float(best_control[1])
        self.cmd_pub.publish(cmd)
    
    def mppi_step(self):
        """Single MPPI optimization step"""
        # Sample control sequences
        control_samples = self.sample_controls()
        
        # Rollout trajectories and compute costs
        trajectories = []
        costs = np.zeros(self.num_samples)
        
        for i in range(self.num_samples):
            traj, cost = self.rollout_trajectory(control_samples[i])
            trajectories.append(traj)
            costs[i] = cost
        
        # Compute weights using softmax
        min_cost = np.min(costs)
        exp_costs = np.exp(-self.lambda_ * (costs - min_cost))
        weights = exp_costs / np.sum(exp_costs)
        
        # Weighted average of control sequences
        optimal_controls = np.sum(
            weights[:, np.newaxis, np.newaxis] * control_samples, 
            axis=0
        )
        
        # Update control sequence (warmstart for next iteration)
        self.control_sequence = optimal_controls
        
        # Visualize
        self.visualize_rollouts(trajectories, costs, weights)
        
        # Return first control
        return optimal_controls[0]
    
    def sample_controls(self):
        """Sample random control perturbations"""
        # Gaussian noise around current control sequence
        noise = np.random.randn(self.num_samples, self.horizon, 2)
        
        # Scale noise
        noise[:, :, 0] *= 0.3  # Linear velocity noise
        noise[:, :, 1] *= 0.5  # Angular velocity noise
        
        # Add to warmstart sequence
        samples = self.control_sequence[np.newaxis, :, :] + noise
        
        # Clip to limits
        samples[:, :, 0] = np.clip(samples[:, :, 0], -self.max_v, self.max_v)
        samples[:, :, 1] = np.clip(samples[:, :, 1], -self.max_w, self.max_w)
        
        return samples
    
    def rollout_trajectory(self, controls):
        """Simulate trajectory and compute cost"""
        traj = np.zeros((self.horizon + 1, 3))  # [x, y, theta]
        
        # Initial state
        traj[0] = [self.current_pose['x'], self.current_pose['y'], self.current_pose['theta']]
        
        # Rollout using kinematic model
        for t in range(self.horizon):
            x, y, theta = traj[t]
            v, w = controls[t]
            
            # Differential drive kinematics
            traj[t + 1, 0] = x + v * np.cos(theta) * self.dt
            traj[t + 1, 1] = y + v * np.sin(theta) * self.dt
            traj[t + 1, 2] = theta + w * self.dt
        
        # Compute cost
        cost = self.compute_cost(traj, controls)
        
        return traj, cost
    
    def compute_cost(self, traj, controls):
        """Compute trajectory cost"""
        cost = 0.0
        
        # Goal cost
        final_x, final_y = traj[-1, 0], traj[-1, 1]
        dist_to_goal = np.sqrt(
            (self.goal['x'] - final_x)**2 + 
            (self.goal['y'] - final_y)**2
        )
        cost += self.w_goal * dist_to_goal
        
        # Obstacle cost
        if self.costmap is not None:
            for t in range(len(traj)):
                x, y = traj[t, 0], traj[t, 1]
                obstacle_cost = self.get_costmap_value(x, y)
                if obstacle_cost > 50:  # Collision threshold
                    cost += self.w_obstacle * 10  # Heavy penalty
                cost += self.w_obstacle * (obstacle_cost / 100.0)
        
        # Control effort cost
        cost += self.w_control * np.sum(controls**2)
        
        # Smoothness cost (penalize control changes)
        if self.horizon > 1:
            control_diff = np.diff(controls, axis=0)
            cost += self.w_smoothness * np.sum(control_diff**2)
        
        return cost
    
    def get_costmap_value(self, x, y):
        """Get costmap value at world coordinates"""
        if self.costmap is None or self.costmap_info is None:
            return 0
        
        # Transform to robot frame (costmap is in base_link frame)
        # For simplicity, assume costmap origin is known
        robot_x = x - self.current_pose['x']
        robot_y = y - self.current_pose['y']
        
        # Convert to grid coordinates
        grid_x = int((robot_x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        grid_y = int((robot_y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        
        # Check bounds
        if 0 <= grid_x < self.costmap_info.width and 0 <= grid_y < self.costmap_info.height:
            return self.costmap[grid_y, grid_x]
        
        return 0
    
    def visualize_rollouts(self, trajectories, costs, weights):
        """Publish visualization of sampled trajectories"""
        marker_array = MarkerArray()
        
        # Normalize costs for coloring
        cost_min = np.min(costs)
        cost_max = np.max(costs)
        cost_range = cost_max - cost_min if cost_max > cost_min else 1.0
        
        # Sample subset for visualization (too many is cluttered)
        vis_indices = np.random.choice(len(trajectories), min(50, len(trajectories)), replace=False)
        
        for idx, i in enumerate(vis_indices):
            traj = trajectories[i]
            cost_normalized = (costs[i] - cost_min) / cost_range
            
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'mppi_rollouts'
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Color: green (low cost) to red (high cost)
            marker.color.r = cost_normalized
            marker.color.g = 1.0 - cost_normalized
            marker.color.b = 0.0
            marker.color.a = 0.3
            
            marker.scale.x = 0.02
            
            for point in traj:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.1
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # Best trajectory
        best_idx = np.argmin(costs)
        best_traj = trajectories[best_idx]
        
        best_marker = Marker()
        best_marker.header.frame_id = self.world_frame
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = 'best_trajectory'
        best_marker.id = 0
        best_marker.type = Marker.LINE_STRIP
        best_marker.action = Marker.ADD
        
        best_marker.color.r = 0.0
        best_marker.color.g = 0.0
        best_marker.color.b = 1.0
        best_marker.color.a = 1.0
        
        best_marker.scale.x = 0.05
        
        for point in best_traj:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.2
            best_marker.points.append(p)
        
        self.rollouts_pub.publish(marker_array)
        self.best_traj_pub.publish(best_marker)
    
    def publish_zero_velocity(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    @staticmethod
    def quat_to_yaw(quat):
        """Convert quaternion to yaw angle"""
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = MPPIController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()