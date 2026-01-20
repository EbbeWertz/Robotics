import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # 1. Publisher: Control robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. Subscriber: Listen to the DepthSentry output
        self.alarm_subscription = self.create_subscription(
            Bool,
            '/isfr/security/alarm_triggered',
            self.alarm_callback,
            10
        )

        # --- CONFIGURATION ---
        self.timer_period = 0.1  # 10Hz control loop
        self.move_duration = 5.0 # Seconds to move
        self.settle_duration = 2.0 # Seconds to wait after stopping (to let camera stabilize)
        self.scan_duration = 5.0 # Seconds to actively scan for intruders

        # --- STATE MANAGEMENT ---
        # States: 'MOVING', 'SETTLING', 'SCANNING'
        self.current_state = 'MOVING'
        self.state_start_time = time.time()
        self.latest_alarm_status = False

        # Create the main control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Patrol Manager Initialized. Starting Patrol...")

    def alarm_callback(self, msg):
        """Continually update local variable with latest sensor status"""
        self.latest_alarm_status = msg.data

    def switch_state(self, new_state):
        self.current_state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"Switching state to: {new_state}")

    def control_loop(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        
        twist = Twist()

        # --- STATE MACHINE ---

        if self.current_state == 'MOVING':
            # ACTION: Drive forward
            twist.linear.x = 0.3 # Adjust speed as needed
            twist.angular.z = 0.0
            
            # TRANSITION: After X seconds, stop
            if elapsed_time > self.move_duration:
                self.switch_state('SETTLING')

        elif self.current_state == 'SETTLING':
            # ACTION: Stop completely
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            # TRANSITION: After camera stabilizes, start scanning
            # We ignore alarms here because the "deceleration" might still look like motion
            if elapsed_time > self.settle_duration:
                self.switch_state('SCANNING')

        elif self.current_state == 'SCANNING':
            # ACTION: Stay stopped
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            # LOGIC: Check for intruders
            # We only trust the DepthSentry when we are in this state
            if self.latest_alarm_status:
                self.get_logger().warn("!!! INTRUDER DETECTED DURING SCAN !!!")
                # Optional: Extend scan time if movement is found?
                # self.state_start_time = current_time 

            # TRANSITION: Patrol area clear, move to next spot
            if elapsed_time > self.scan_duration:
                self.get_logger().info("Scan clear. Resuming patrol.")
                self.switch_state('MOVING')

        # Publish the command
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        stop_msg = Twist()
        node.cmd_vel_publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()