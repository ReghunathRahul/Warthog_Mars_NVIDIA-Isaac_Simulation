from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Point cloud to costmap converter
        Node(
            package='mppi_navigation',
            executable='pointcloud_to_costmap',
            name='pointcloud_to_costmap',
            output='screen',
            parameters=[{
                'grid_resolution': 0.1,
                'grid_width': 10.0,
                'grid_height': 10.0,
                'min_obstacle_height': 0.15,
                'max_obstacle_height': 2.0,
                'base_frame': 'lidar_mount_link'
            }],
            remappings=[
                ('/lidar/scan', '/point_cloud')  # Change to your actual topic
            ]
        ),
        
        # MPPI Controller
        Node(
            package='mppi_navigation',
            executable='mppi_controller',
            name='mppi_controller',
            output='screen',
            parameters=[{
                'num_samples': 2000,
                'horizon': 50,
                'dt': 0.1,
                'lambda_': 1.0,
                'max_linear_vel': 1.0,
                'max_angular_vel': 1.0,
                'base_frame': 'base_link',
                'world_frame': 'world'
            }],
            remappings=[
                ('/odom', '/odom'),
                ('/cmd_vel', '/cmd_vel')
            ]
        ),
    ])