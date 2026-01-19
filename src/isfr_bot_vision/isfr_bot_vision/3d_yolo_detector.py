import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # <--- Nodig voor coördinaten
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 1. Model Laden
        self.get_logger().info("YOLO Model laden...")
        self.model = YOLO("yolov8n.pt") 
        self.get_logger().info("YOLO Model geladen!")

        # 2. Setup ROS connecties
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
        
        # Debug beeld publisher
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)
        
        # 3. DYNAMISCHE PUBLISHERS LIJST
        # Hier bewaren we de publishers die we tijdens het draaien aanmaken
        # Bijv: {'bottle': <publisher_object>, 'cup': <publisher_object>}
        self.object_publishers = {}
        
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def get_depthh(self, center_x, center_y):
        h, w = self.latest_depth.shape
        cx = int(min(max(center_x, 0), w - 1))
        cy = int(min(max(center_y, 0), h - 1))

        depth = float(self.latest_depth[cy, cx])

        if depth == float('inf') or depth == 0.0:
            depth = -1.0  # invalid
        return depth
    
    def calculate3DPoint(self, x2d, y2d, depth):
        # focal length
        fx = fy = CAMERA_PARAMS["width"] / (2 * (math.tan(CAMERA_PARAMS["FOV"]/2)))
        # camera center
        cx, cy = CAMERA_PARAMS["width"]/2, CAMERA_PARAMS["height"]/2
        z3d = depth
        x3d = (x2d - cx) * z3d / fx
        y3d = (y2d - cy) * z3d / fy
        return (x3d, y3d, z3d)


    def image_callback(self, msg):
        if self.latest_depth is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Confidence op 0.4 gezet voor betere filtering
            results = self.model(cv_image, verbose=False, conf=0.4)
            annotated_frame = results[0].plot()

            # Loop door alle gevonden objecten
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name_raw = self.model.names[class_id] # Bijv: "wine glass"
                
                # ROS topics mogen geen spaties hebben, dus vervang spatie door underscore
                # "wine glass" -> "wine_glass"
                topic_name = class_name_raw.replace(" ", "_")
                
                # Bereken coördinaten (Middenpunt van het vierkantje)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                center_x = float((x1 + x2) / 2)
                center_y = float((y1 + y2) / 2)

                depth = self.get_depthh(center_x, center_y)

                if depth < 0:
                    continue

                (x3d, y3d, z3d) = self.calculate3DPoint(center_x, center_y, depth)

                
                # --- HIER IS DE MAGIE: MAAK TOPIC ALS HET NOG NIET BESTAAT ---
                if topic_name not in self.object_publishers:
                    # Maak een nieuwe publisher aan, bijv: /vision/objects/bottle
                    new_topic = f'/vision/objects/{topic_name}'
                    self.get_logger().info(f'Nieuw objecttype gevonden! Topic aanmaken: {new_topic}')
                    
                    self.object_publishers[topic_name] = self.create_publisher(Point, new_topic, 10)
                
                # --- PUBLISH DE COORDINATEN ---
                point_msg = Point()
                point_msg.x = x3d
                point_msg.y = y3d
                point_msg.z = z3d
                
                self.object_publishers[topic_name].publish(point_msg)

                # Loggen voor debug (alleen bottle en glas types)
                if topic_name == 'bottle':
                    self.get_logger().info(f'🍾 Bottle op X:{center_x:.0f}, Y:{center_y:.0f} -> /vision/objects/bottle')
                elif topic_name in ['wine_glass', 'cup']:
                     self.get_logger().info(f'🍷 Glas op X:{center_x:.0f}, Y:{center_y:.0f} -> /vision/objects/{topic_name}')

                cv2.putText(
                    annotated_frame,
                    f"({x3d:.2f}, {y3d:.2f}, {z3d:.2f}) m",
                    (int(x1), int(y2) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
            # Publiceer het debug plaatje
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.debug_publisher.publish(img_msg)
            
            # cv2.imshow("YOLO Camera View", annotated_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Fout in image processing: {str(e)}')

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