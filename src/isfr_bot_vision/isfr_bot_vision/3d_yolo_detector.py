import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from isfr_bot_msgs.msg import YoloVisionObject, YoloVisionObjectArray
from builtin_interfaces.msg import Time

# TF2 Imports voor coördinaat transformaties
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLO model loaded.")

        # --- TF2 Setup (Voor wereldposities) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Image,
            '/isfr/camera_sensor/image_raw/image_color',
            self.image_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/isfr/camera_sensor/depth/image',
            self.depth_callback,
            10
        )
        self.latest_depth = None

        self.bridge = CvBridge()

        # Topic voor objecten relatief aan de robot (Camera frame)
        self.publisher = self.create_publisher(YoloVisionObjectArray, '/vision/objects', 10)

        # Topic voor objecten absoluut in de wereld (Map frame)
        self.abs_publisher = self.create_publisher(YoloVisionObjectArray, '/vision/absolute_position', 10)

        # Debug image
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)


    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def calculate3DPoint(self, x2d, y2d, depth):
        # Berekent punt in het optische frame van de camera
        fx = fy = CAMERA_PARAMS["width"] / (2 * math.tan(CAMERA_PARAMS["FOV"]/2))
        cx, cy = CAMERA_PARAMS["width"]/2, CAMERA_PARAMS["height"]/2
        z3d = depth
        x3d = (x2d - cx) * z3d / fx
        y3d = (y2d - cy) * z3d / fy
        return (x3d, y3d, z3d)

    def transform_to_world(self, x, y, z, from_frame):
        """
        Transformeert een punt van 'from_frame' (camera) naar 'map' (wereld).
        """
        try:
            # We maken een PointStamped (Punt met tijd en frame informatie)
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            # We gebruiken Time() (0) om de allerlaatste beschikbare transform te pakken
            point_stamped.header.stamp = rclpy.time.Time().to_msg() 
            point_stamped.point.x = float(x)
            point_stamped.point.y = float(y)
            point_stamped.point.z = float(z)

            # Zoek de transformatie van camera naar map
            # Let op: 'map' moet bestaan (bijv. via SLAM), anders gebruik 'odom'
            transform = self.tf_buffer.lookup_transform("arm_base_link", from_frame, rclpy.time.Time())            
            # Voer de transformatie uit
            point_world = do_transform_point(point_stamped, transform)
            
            return point_world.point.x, point_world.point.y, point_world.point.z

        except Exception as e:
            self.get_logger().warn(f"Kon transformatie niet berekenen: {e}")
            return None

    def get_identity_depth(self, box):
        if self.latest_depth is None:
            return -1.0
        x1, y1, x2, y2 = [int(v) for v in box]
        # Zorg dat indices binnen de image bounds blijven
        height_img, width_img = self.latest_depth.shape
        x1, x2 = max(0, x1), min(width_img, x2)
        y1, y2 = max(0, y1), min(height_img, y2)

        width = x2 - x1
        height = y2 - y1
        if width <= 0 or height <= 0: return -1.0

        hline_y = y1 + height // 2
        w_start = x1 + int(0.2 * width)
        w_end   = x2 - int(0.2 * width)
        
        # Veiligheid: check bounds opnieuw
        w_start = max(0, w_start)
        w_end = min(width_img, w_end)
        hline_y = min(height_img - 1, hline_y)

        line_depths = self.latest_depth[hline_y, w_start:w_end]
        valid_depths = line_depths[(line_depths > 0) & (line_depths != float('inf'))]
        if len(valid_depths) == 0:
            return -1.0
        return float(valid_depths.min())

    def image_callback(self, msg):
        if self.latest_depth is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False, conf=0.4)
            annotated_frame = results[0].plot()

            # We maken twee berichten: één relatief, één absoluut
            rel_array_msg = YoloVisionObjectArray()
            rel_array_msg.stamp = self.get_clock().now().to_msg()
            
            abs_array_msg = YoloVisionObjectArray()
            abs_array_msg.stamp = self.get_clock().now().to_msg()

            # Haal frame_id uit het camera bericht (belangrijk voor transform!)
            camera_frame_id = msg.header.frame_id 

            for box in results[0].boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                identity_depth = self.get_identity_depth([x1, y1, x2, y2])
                if identity_depth < 0:
                    continue

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # 1. Bereken Relatieve Positie (Camera Frame)
                x3d, y3d, z3d = self.calculate3DPoint(center_x, center_y, identity_depth)

                # Vul relatief bericht
                object_msg = YoloVisionObject()
                object_msg.label = label
                object_msg.xmin, object_msg.ymin = float(x1), float(y1)
                object_msg.xmax, object_msg.ymax = float(x2), float(y2)
                object_msg.identity_depth = identity_depth
                object_msg.x, object_msg.y, object_msg.z = x3d, y3d, z3d
                rel_array_msg.objects.append(object_msg)

                # 2. Bereken Absolute Positie (World/Map Frame)
                world_pos = self.transform_to_world(x3d, y3d, z3d, camera_frame_id)
                
                if world_pos:
                    wx, wy, wz = world_pos
                    
                    abs_object_msg = YoloVisionObject()
                    # Kopieer basis info
                    abs_object_msg.label = label
                    abs_object_msg.xmin, abs_object_msg.ymin = float(x1), float(y1)
                    abs_object_msg.xmax, abs_object_msg.ymax = float(x2), float(y2)
                    abs_object_msg.identity_depth = identity_depth
                    # Gebruik WERELD coordinaten
                    abs_object_msg.x = wx
                    abs_object_msg.y = wy
                    abs_object_msg.z = wz
                    
                    abs_array_msg.objects.append(abs_object_msg)

                    self.get_logger().info(f'Detected {label} at REL({x3d:.2f}, {z3d:.2f}) -> ABS({wx:.2f}, {wy:.2f})')

            # Publish beide
            self.publisher.publish(rel_array_msg)
            self.abs_publisher.publish(abs_array_msg)

            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.debug_publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')
            import traceback
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()