import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to be ready
    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()

    # --- DEFINE THE MISSION (edit coordinates x, y) ---
    goal_poses = []

    # Waypoint 1: The "Safe" Approach (2 meters forward)
    wp1 = PoseStamped()
    wp1.header.frame_id = 'world'
    wp1.header.stamp = navigator.get_clock().now().to_msg()
    wp1.pose.position.x = 2.0
    wp1.pose.position.y = 0.5
    wp1.pose.orientation.w = 1.0
    goal_poses.append(wp1)

    # Waypoint 2: The "Corner" (Turn around the rock)
    wp2 = PoseStamped()
    wp2.header.frame_id = 'world'
    wp2.header.stamp = navigator.get_clock().now().to_msg()
    wp2.pose.position.x = 4.0
    wp2.pose.position.y = 2.0
    wp2.pose.orientation.w = 1.0
    goal_poses.append(wp2)

    # Waypoint 3: The Final Goal
    wp3 = PoseStamped()
    wp3.header.frame_id = 'world'
    wp3.header.stamp = navigator.get_clock().now().to_msg()
    wp3.pose.position.x = 6.0
    wp3.pose.position.y = 0.0
    wp3.pose.orientation.w = 1.0
    goal_poses.append(wp3)

    # --- EXECUTE MISSION ---
    print("Starting Waypoint Mission...")
    # 'goThroughPoses' is better than 'followWaypoints' because it doesn't stop at each point
    navigator.goThroughPoses(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
            
            # Anti-Stuck Logic: If robot freezes for too long, cancel
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60):
                navigator.cancelTask()

    # --- RESULT ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Mission Complete! Robot reached the goal.')
    elif result == TaskResult.CANCELED:
        print('Mission canceled.')
    elif result == TaskResult.FAILED:
        print('Mission failed!')

    exit(0)

if __name__ == '__main__':
    main()
