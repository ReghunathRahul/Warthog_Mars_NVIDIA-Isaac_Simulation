from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="local_costmap",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            "/home/jotheesh/warthog_nav2_ws/src/warthog_nav2/config/local_costmap.yaml",
        ],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            "/home/jotheesh/warthog_nav2_ws/src/warthog_nav2/config/controller.yaml",
        ],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "controller_server",
                "costmap"
            ]
        }]
    )

    return LaunchDescription([costmap, controller, lifecycle])

