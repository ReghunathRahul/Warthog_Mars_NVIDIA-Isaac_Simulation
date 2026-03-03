#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.get_logger().info('Goal Publisher node started')
    
    def publish_goal(self, x, y, theta=0.0):
        """Publish a goal pose"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Convert theta to quaternion (simplified - only yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0  # sin(theta/2)
        msg.pose.orientation.w = 1.0  # cos(theta/2)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Usage: ros2 run mppi_navigation goal_publisher <x> <y> [theta]")
        print("Example: ros2 run mppi_navigation goal_publisher 5.0 3.0 0.0")
        return
    
    node = GoalPublisher()
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        # Publish goal
        node.publish_goal(x, y, theta)
        
        # Keep node alive briefly
        rclpy.spin_once(node, timeout_sec=1.0)
        
    except ValueError:
        node.get_logger().error('Invalid coordinates. Please provide numbers.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()