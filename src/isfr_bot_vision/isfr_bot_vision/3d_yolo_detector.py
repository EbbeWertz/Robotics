import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
import cv2
import numpy as np
import math
from ultralytics import YOLO
from cv_bridge import CvBridge

# ROS Messages
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

# TF2
import tf2_ros
import tf2_geometry_msgs

# Camera instellingen
CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57 
}

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.get_logger().info("YoloDetector active - With Axis Correction")
        
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map Subscriber
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.latest_map = None 

        # Vision Subscribers
        self.create_subscription(Image, '/isfr/camera_sensor/image_raw/image_color', self.image_callback, 10)
        self.create_subscription(Image, '/isfr/camera_sensor/depth/image', self.depth_callback, 10)
        self.latest_depth = None

        # Publishers
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/vision/knowledge_grid', 10)

        # Geheugen
        self.map_obstacles = [] 

        self.create_timer(0.5, self.publish_knowledge_grid)
        self.get_logger().info("Waiting for data...")

    # --- CALLBACKS ---

    def map_callback(self, msg):
        self.latest_map = msg

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception:
            pass

    def image_callback(self, msg):
        if self.latest_depth is None: return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False, conf=0.5)
            annotated_frame = results[0].plot()

            for box in results[0].boxes:
                label = self.model.names[int(box.cls[0])]
                if label not in ['bottle', 'cup', 'wine_glass']: continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                depth = self.get_identity_depth([x1, y1, x2, y2])
                
                # Filter: Negeer dingen die te ver weg zijn (> 3.5m) voor precisie
                if depth <= 0 or depth > 3.5: continue

                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                
                # 1. Bereken 3D punt in Camera Optical Frame
                # x=Rechts, y=Omlaag, z=Vooruit
                cam_x, cam_y, cam_z = self.calculate3DPoint(cx, cy, depth)

                # 2. Transformeer naar Map Frame (met as-correctie)
                map_point = self.transform_camera_to_map(cam_x, cam_y, cam_z)
                
                if map_point:
                    abs_x, abs_y = map_point
                    self.add_obstacle_to_memory(abs_x, abs_y)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    # --- HIER ZIT DE FIX ---

    def transform_camera_to_map(self, cx, cy, cz):
        try:
            pt = PointStamped()
            pt.header.frame_id = "camera_sensor" # Zorg dat dit matcht met je URDF link naam!
            pt.header.stamp = rclpy.time.Time().to_msg()
            
            # --- CRUCIALE AS-WISSEL (Optical -> Geometrical) ---
            # Camera Taal:     X=Rechts, Y=Omlaag, Z=Vooruit (Diepte)
            # ROS Link Taal:   X=Vooruit, Y=Links, Z=Omhoog
            
            # We mappen de Optical data naar de ROS Link assen:
            pt.point.x = cz       # ROS Vooruit = Camera Diepte
            pt.point.y = -cx      # ROS Links   = -Camera Rechts
            pt.point.z = -cy      # ROS Omhoog  = -Camera Omlaag
            
            # Lookup transform (met timeout om crashes te voorkomen)
            # We vragen: "Waar stond de robot 0 seconden geleden?"
            timeout = Duration(seconds=0.1)
            trans = self.tf_buffer.lookup_transform("map", pt.header.frame_id, rclpy.time.Time(), timeout)
            
            pt_map = tf2_geometry_msgs.do_transform_point(pt, trans)
            return (pt_map.point.x, pt_map.point.y)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    # --- REST VAN DE CODE ---

    def publish_knowledge_grid(self):
        if self.latest_map is None or not self.map_obstacles:
            return

        grid_msg = OccupancyGrid()
        grid_msg.header = self.latest_map.header
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info = self.latest_map.info 

        width = grid_msg.info.width
        height = grid_msg.info.height
        res = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y

        grid_data = bytearray([0] * (width * height))

        for (wx, wy) in self.map_obstacles:
            gx = int((wx - origin_x) / res)
            gy = int((wy - origin_y) / res)

            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        grid_data[ny * width + nx] = 100 

        grid_msg.data = grid_data
        self.grid_pub.publish(grid_msg)

    def add_obstacle_to_memory(self, x, y):
        for ex, ey in self.map_obstacles:
            if math.hypot(x - ex, y - ey) < 0.30: 
                return 
        self.map_obstacles.append((x, y))
        self.get_logger().info(f"📍 Fles toegevoegd: X={x:.2f}, Y={y:.2f}")

    def get_identity_depth(self, box):
        if self.latest_depth is None: return -1.0
        h, w = self.latest_depth.shape
        x1, y1, x2, y2 = map(int, box)
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1: return -1.0
        crop = self.latest_depth[y1:y2, x1:x2]
        valid = crop[(crop > 0.1) & (crop < 10.0)]
        if len(valid) == 0: return -1.0
        return float(np.median(valid))

    def calculate3DPoint(self, x2d, y2d, depth):
        # Dit geeft coördinaten in het CAMERA (OPTICAL) frame
        fov = CAMERA_PARAMS.get("FOV", 1.57)
        width = CAMERA_PARAMS.get("width", 640)
        tan_val = math.tan(fov / 2) or 1.0
        fx = width / (2 * tan_val)
        cx, cy = width / 2, CAMERA_PARAMS.get("height", 480) / 2
        
        x_opt = (x2d - cx) * depth / fx
        y_opt = (y2d - cy) * depth / fx 
        z_opt = depth
        return (x_opt, y_opt, z_opt)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()