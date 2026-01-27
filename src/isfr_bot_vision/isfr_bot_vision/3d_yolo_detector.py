import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from isfr_bot_msgs.msg import YoloVisionObject, YoloVisionObjectArray
from builtin_interfaces.msg import Time

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

        # general object topic
        self.publisher = self.create_publisher(YoloVisionObjectArray, '/vision/objects', 10)

        # debug
        self.debug_publisher = self.create_publisher(Image, '/vision/debug_image', 10)

        

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def calculate3DPoint(self, x2d, y2d, depth):
        fx = fy = CAMERA_PARAMS["width"] / (2 * math.tan(CAMERA_PARAMS["FOV"]/2))
        cx, cy = CAMERA_PARAMS["width"]/2, CAMERA_PARAMS["height"]/2
        z3d = depth
        x3d = (x2d - cx) * z3d / fx
        y3d = (y2d - cy) * z3d / fy
        return (x3d, y3d, z3d)

    def get_identity_depth(self, box):
        if self.latest_depth is None:
            return -1.0
        x1, y1, x2, y2 = [int(v) for v in box]
        width = x2 - x1
        height = y2 - y1
        hline_y = y1 + height // 2
        w_start = x1 + int(0.2 * width)
        w_end   = x2 - int(0.2 * width)
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
            results = self.model(cv_image, verbose=False, conf=0.6)
            annotated_frame = results[0].plot()

            object_array_msg = YoloVisionObjectArray()
            object_array_msg.stamp = self.get_clock().now().to_msg()

            for box in results[0].boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                identity_depth = self.get_identity_depth([x1, y1, x2, y2])
                if identity_depth < 0:
                    continue

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                x3d, y3d, z3d = self.calculate3DPoint(center_x, center_y, identity_depth)

                object_msg = YoloVisionObject()
                object_msg.label = label
                object_msg.xmin = float(x1)
                object_msg.ymin = float(y1)
                object_msg.xmax = float(x2)
                object_msg.ymax = float(y2)
                object_msg.identity_depth = identity_depth
                object_msg.x = x3d
                object_msg.y = y3d
                object_msg.z = z3d
                object_array_msg.objects.append(object_msg)

            self.publisher.publish(object_array_msg)

            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.debug_publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')


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