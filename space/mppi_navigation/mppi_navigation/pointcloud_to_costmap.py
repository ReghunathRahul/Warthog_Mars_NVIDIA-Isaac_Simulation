#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import numpy as np
import struct
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PointCloudToCostmap(Node):
    def __init__(self):
        super().__init__('pointcloud_to_costmap')
        
        # Parameters
        self.declare_parameter('grid_resolution', 0.1)  # 10cm per cell
        self.declare_parameter('grid_width', 10.0)      # 10m x 10m grid
        self.declare_parameter('grid_height', 10.0)
        self.declare_parameter('min_obstacle_height', 0.15)  # Filter ground
        self.declare_parameter('max_obstacle_height', 2.0)
        self.declare_parameter('base_frame', 'base_link')
        
        self.resolution = self.get_parameter('grid_resolution').value
        self.width = self.get_parameter('grid_width').value
        self.height = self.get_parameter('grid_height').value
        self.min_height = self.get_parameter('min_obstacle_height').value
        self.max_height = self.get_parameter('max_obstacle_height').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Grid dimensions in cells
        self.grid_width_cells = int(self.width / self.resolution)
        self.grid_height_cells = int(self.height / self.resolution)
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers and Publishers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar/scan',  # Change this to your actual topic
            self.pointcloud_callback,
            10
        )
        
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap',
            10
        )
        
        self.get_logger().info(f'PointCloud to Costmap node started')
        self.get_logger().info(f'Grid size: {self.grid_width_cells}x{self.grid_height_cells} cells')
        self.get_logger().info(f'Resolution: {self.resolution}m/cell')
        
    def pointcloud_callback(self, msg):
        """Process point cloud and create occupancy grid"""
        try:
            # Parse point cloud
            points = self.parse_pointcloud(msg)
            
            if points is None or len(points) == 0:
                self.get_logger().warn('No points in cloud')
                return
            
            # Transform points to base_link frame if needed
            # if msg.header.frame_id != self.base_frame:
            #     points = self.transform_points(points, msg.header.frame_id, msg.header.stamp)
            #     if points is None:
            #         return
            
            # Filter by height (remove ground, keep obstacles)
            mask = (points[:, 2] > self.min_height) & (points[:, 2] < self.max_height)
            obstacle_points = points[mask]
            
            if len(obstacle_points) == 0:
                self.get_logger().debug('No obstacle points after filtering')
            
            # Create 2D occupancy grid
            costmap = self.create_occupancy_grid(obstacle_points)
            
            # Publish
            grid_msg = self.create_grid_msg(costmap, msg.header.stamp)
            self.costmap_pub.publish(grid_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud_callback: {str(e)}')
    
    def parse_pointcloud(self, msg):
        """Parse PointCloud2 message to numpy array"""
        # Get point step and fields
        point_step = msg.point_step
        
        # Find x, y, z field offsets
        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if None in [x_offset, y_offset, z_offset]:
            self.get_logger().error('Could not find x, y, z fields in point cloud')
            return None
        
        # Extract points
        num_points = len(msg.data) // point_step
        points = np.zeros((num_points, 3))
        
        for i in range(num_points):
            base = i * point_step
            x = struct.unpack('f', msg.data[base + x_offset:base + x_offset + 4])[0]
            y = struct.unpack('f', msg.data[base + y_offset:base + y_offset + 4])[0]
            z = struct.unpack('f', msg.data[base + z_offset:base + z_offset + 4])[0]
            
            # Filter out invalid points
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points[i] = [x, y, z]
        
        return points
    
    def transform_points(self, points, source_frame, stamp):
        """Transform points to base_link frame"""
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix (simplified, only if needed)
            # For now, just apply translation
            points_transformed = points.copy()
            points_transformed[:, 0] += trans.x
            points_transformed[:, 1] += trans.y
            points_transformed[:, 2] += trans.z
            
            return points_transformed
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform points: {str(e)}')
            return None
    
    def create_occupancy_grid(self, points):
        """Create 2D occupancy grid from 3D points"""
        # Initialize grid (0 = free, 100 = occupied, -1 = unknown)
        grid = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.int8)
        
        # Grid is centered on robot
        origin_x = -self.width / 2.0
        origin_y = -self.height / 2.0
        
        # Project points to 2D and mark cells
        for point in points:
            x, y = point[0], point[1]
            
            # Convert to grid coordinates
            grid_x = int((x - origin_x) / self.resolution)
            grid_y = int((y - origin_y) / self.resolution)
            
            # Check bounds
            if 0 <= grid_x < self.grid_width_cells and 0 <= grid_y < self.grid_height_cells:
                grid[grid_y, grid_x] = 100  # Mark as occupied
        
        return grid
    
    def create_grid_msg(self, grid, stamp):
        """Create OccupancyGrid message"""
        msg = OccupancyGrid()
        
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width_cells
        msg.info.height = self.grid_height_cells
        
        # Grid origin (centered on robot)
        msg.info.origin.position.x = -self.width / 2.0
        msg.info.origin.position.y = -self.height / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten grid (row-major order)
        msg.data = grid.flatten().tolist()
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()