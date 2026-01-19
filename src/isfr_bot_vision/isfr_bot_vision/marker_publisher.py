import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('yolo_marker_publisher')

        # Dictionary to hold subscribers for each object type
        self.subscribers = {}

        # Publisher for RViz Markers
        self.marker_pub = self.create_publisher(Marker, '/vision/object_markers', 10)

        # Unique marker ID
        self.marker_id = 0

        # Optional: object types you expect; otherwise dynamically subscribe
        self.object_types = ['bottle', 'wine_glass', 'cup']

        # Subscribe to each object topic
        for obj in self.object_types:
            topic_name = f'/vision/objects/{obj}'
            self.subscribers[obj] = self.create_subscription(
                Point,
                topic_name,
                self.point_callback_factory(obj),
                10
            )

    def point_callback_factory(self, obj_name):
        """Creates a callback for each object type to capture its name in closure."""
        def callback(msg: Point):
            marker = Marker()
            marker.header.frame_id = "camera_sensor"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = obj_name
            marker.id = self.marker_id
            self.marker_id += 1

            # Choose marker type (sphere)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set 3D position
            marker.pose.position.x = msg.x
            marker.pose.position.y = msg.y
            marker.pose.position.z = msg.z
            marker.pose.orientation.w = 1.0  # no rotation

            # Size of marker in meters
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color based on object type
            if obj_name == 'bottle':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif obj_name == 'wine_glass':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:  # cup or other
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.8  # transparency

            # Publish marker
            self.marker_pub.publish(marker)

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
