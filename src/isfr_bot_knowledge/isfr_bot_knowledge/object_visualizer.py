import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from isfr_bot_msgs.msg import YoloVisionObjectArray

class ObjectVisualizer(Node):
    def __init__(self):
        super().__init__('object_visualizer')

        # Subscriber luistert naar jouw Yolo topic
        self.subscription = self.create_subscription(
            YoloVisionObjectArray,
            '/vision/absolute_position',  # Check of dit topic klopt
            self.listener_callback,
            10)

        # Publisher stuurt een Array van markers naar RViz
        self.publisher_ = self.create_publisher(MarkerArray, '/vision/knowledge', 10)
        
        self.get_logger().info('Knowledge gestart: Yolo Array -> Markers')

    def listener_callback(self, msg):
        marker_array = MarkerArray()
        target_frame = "odom"

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        # marker_array.markers.append(delete_marker)

        # Loop door alle gevonden objecten in de lijst
        for i, obj in enumerate(msg.objects):
            self.get_logger().info(f'Visualizing object: {obj.label} at ({obj.x}, {obj.y}, {obj.z})')
                        
            # --- 1. De Visuele Bol (Positie) ---
            sphere_marker = Marker()
            sphere_marker.header.frame_id = target_frame
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            
            sphere_marker.ns = "object_shapes"
            sphere_marker.id = i  # Uniek ID per object
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            # Positie uit jouw bericht halen
            sphere_marker.pose.position.x = float(obj.x)
            sphere_marker.pose.position.y = float(obj.y)
            sphere_marker.pose.position.z = float(obj.z)
            
            sphere_marker.pose.orientation.w = 1.0
            
            # Grootte (bijv. 10cm)
            sphere_marker.scale.x = 0.1
            sphere_marker.scale.y = 0.1
            sphere_marker.scale.z = 0.1
            
            # Kleur: Cyaan
            sphere_marker.color.r = 0.0
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 1.0
            sphere_marker.color.a = 1.0 # Alpha

            marker_array.markers.append(sphere_marker)

            # --- 2. Het Label (Tekst boven object) ---
            text_marker = Marker()
            text_marker.header.frame_id = target_frame
            text_marker.header.stamp = self.get_clock().now().to_msg()
            
            text_marker.ns = "object_labels"
            text_marker.id = i + 1000 # Zorg voor een unieke ID die niet botst met de bollen
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Tekst iets boven de bol zetten
            text_marker.pose.position.x = float(obj.x)
            text_marker.pose.position.y = float(obj.y)
            text_marker.pose.position.z = float(obj.z) + 0.2 
            
            text_marker.text = obj.label # Het label uit jouw bericht
            text_marker.scale.z = 0.1 # Tekstgrootte
            
            # Kleur: Wit
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        # Publiceer de hele lijst
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()