import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        # Luister naar de Nav2 output
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        # Stuur door naar de robot (die op stamped wacht)
        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10)

    def listener_callback(self, msg):
        new_msg = TwistStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'base_link'
        new_msg.twist = msg
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()