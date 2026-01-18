import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 1. Laad het Nano model (kleinste en snelste versie)
        # De eerste keer dat je dit runt, downloadt hij 'yolov8n.pt' automatisch.
        self.get_logger().info("YOLO Model laden...")
        self.model = YOLO("yolov8n.pt") 
        self.get_logger().info("YOLO Model geladen!")

        # 2. Setup ROS connecties
        # LET OP: Check met 'ros2 topic list' of jouw camera topic echt zo heet!
        # In Webots is het vaak /NAAM_VAN_ROBOT/NAAM_VAN_CAMERA/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/isfr_bot/camera_sensor/image_raw', 
            self.image_callback,
            10)
        
        # We sturen het beeld met de hokjes eromheen terug naar ROS
        self.publisher = self.create_publisher(Image, '/vision/debug_image', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # 3. Converteer ROS image naar OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 4. Run YOLO (inference)
            # conf=0.5 betekent: we moeten 50% zeker zijn voordat we iets zeggen
            results = self.model(cv_image, verbose=False, conf=0.5)
            
            # 5. Teken de resultaten op het beeld
            annotated_frame = results[0].plot()

            # 6. Check wat we gevonden hebben
            # YOLO classes: 39=bottle, 40=wine glass, 41=cup
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                
                if class_name == 'bottle':
                    self.get_logger().info(f'FLESJE GEVONDEN! (Zekerheid: {float(box.conf[0]):.2f})')
                
                elif class_name in ['wine glass', 'cup']: 
                    self.get_logger().info(f'GLAS GEVONDEN! ({class_name})')
                    
                else:
                    self.get_logger().info(f'Ander object gevonden: {class_name}')

            # 7. Publiceer het beeld terug (zodat jij het kan zien in rviz/rqt)
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.publisher.publish(img_msg)
            
            cv2.imshow("YOLO Camera View", annotated_frame)
            cv2.waitKey(1)

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