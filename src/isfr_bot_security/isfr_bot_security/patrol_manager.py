import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import time

class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # 1. Action Client for Navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 2. Subscriber: DepthSentry alarm
        self.alarm_subscription = self.create_subscription(
            Bool, '/isfr/security/alarm_triggered', self.alarm_callback, 10
        )

        # --- WAITING FOR NAV2 ---
        # We assume Nav2 is launching. We wait here before starting the main timer.
        self.get_logger().info("Waiting for 'navigate_to_pose' action server...")
        self.nav_to_pose_client.wait_for_server() # This blocks until Nav2 is ready
        self.get_logger().info("Nav2 is ready! Starting patrol.")
        
        # --- CONFIGURATION & WAYPOINTS ---
        # Calculated from your World File:
        # Robot starts at World X=6.36. 
        # We subtract 6.36 from World X to get these numbers.
        
        self.waypoints = [
            (1.1, 0.0),    # Center of the room (World X=7.5)
            (1.1, 2.0),    # Move Left (World Y=2.0)
            (2.5, 0.0),    # Near the far wall (World X=8.8)
            (0.0, 0.0)     # Return Home
        ]
        self.current_waypoint_index = 0
        
        self.settle_duration = 2.0
        self.scan_duration = 5.0

        # --- STATE MANAGEMENT ---
        # States: 'SEND_GOAL', 'NAVIGATING', 'SETTLING', 'SCANNING'
        self.current_state = 'SEND_GOAL'
        self.state_start_time = time.time()
        self.latest_alarm_status = False
        self.navigation_finished = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Patrol Manager with Nav2 Initialized.")

    def alarm_callback(self, msg):
        self.latest_alarm_status = msg.data

    def switch_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"Switching state to: {new_state}")

    def send_nav_goal(self):
        """Sends the next waypoint to the Nav2 stack."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 Action Server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        x, y = self.waypoints[self.current_waypoint_index]
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Simple orientation

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index}: x={x}, y={y}")
        
        self.navigation_finished = False
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.switch_state('SEND_GOAL')
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.navigation_finished = True
        # Move to next waypoint index for the next loop
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def control_loop(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time

        if self.current_state == 'SEND_GOAL':
            self.send_nav_goal()
            self.switch_state('NAVIGATING')

        elif self.current_state == 'NAVIGATING':
            # Transition when the Nav2 result callback sets this to True
            if self.navigation_finished:
                self.switch_state('SETTLING')

        elif self.current_state == 'SETTLING':
            if elapsed_time > self.settle_duration:
                self.switch_state('SCANNING')

        elif self.current_state == 'SCANNING':
            if self.latest_alarm_status:
                self.get_logger().warn("!!! INTRUDER DETECTED !!!")

            if elapsed_time > self.scan_duration:
                self.get_logger().info("Scan clear. Moving to next waypoint.")
                self.switch_state('SEND_GOAL')

def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()