import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Image processing imports
from cv_bridge import CvBridge
import cv2

# Standard ROS Messages
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, TransformStamped
from nav2_msgs.action import NavigateToPose

# TF Imports
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_geometry_msgs import do_transform_point

# Jouw Custom Messages
from isfr_bot_msgs.msg import YoloVisionObjectArray 

# --- CONFIGURATIE ---
CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}
# Veiligheidsmarges voor de grijper
MARGIN_M              = 0.02   
OCCLUSION_CLEARANCE_M = 0.05   
BOTTOM_CLEARANCE_M    = 0.10   
STOP_DISTANCE         = 0.50   # Afstand (meter) om te stoppen voor de fles

class SmartFetcher(Node):
    def __init__(self):
        super().__init__('smart_fetcher')
        
        self.is_busy = False 
        self.bridge = CvBridge()
        self.latest_depth_img = None

        # 1. FIX: Maak de ontbrekende Camera Link aan
        # Dit voorkomt dat je URDF bestanden hoeft aan te passen
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.fix_missing_camera_tf()

        # 2. TF Buffer setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Costmap Subscriber (voor checken obstakels)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, qos_map)
        self.latest_costmap = None

        # 4. Depth Camera Subscriber
        self.depth_sub = self.create_subscription(
            Image,
            '/isfr/camera_sensor/depth/image',
            self.depth_callback,
            10
        )

        # 5. YOLO Vision Subscriber
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.vision_sub = self.create_subscription(
            YoloVisionObjectArray,
            '/vision/objects', 
            self.vision_callback,
            qos_sensor
        )

        # 6. Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Smart Fetcher: BRAIN ONLINE. Ready to fetch.')

    def fix_missing_camera_tf(self):
        """Maakt de verbinding base_link -> camera_link handmatig aan."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Positie op de Turtlebot Waffle
        t.transform.translation.x = 0.064
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.094
        t.transform.rotation.w = 1.0 # Geen rotatie

        self.tf_static_broadcaster.sendTransform(t)

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def vision_callback(self, msg):
        # Niet reageren als we al bezig zijn of geen depth hebben
        if self.is_busy or self.latest_depth_img is None:
            return

        for obj in msg.objects:
            if obj.label == "bottle":
                # Stap 1: Is het object 'vrij' om te pakken?
                if self.is_safe_to_grab(obj):
                    
                    # Stap 2: Waar is het object op de kaart?
                    map_x, map_y = self.transform_camera_to_map(obj.x, obj.y, obj.z)
                    
                    if map_x is not None:
                        self.get_logger().info(f"Fles GEVONDEN op: [{map_x:.2f}, {map_y:.2f}]")
                        self.is_busy = True
                        
                        # Stap 3: Bereken route en GA
                        self.find_safe_spot_and_go(map_x, map_y)
                        return # Stop na 1 fles, focus op taak

    def find_safe_spot_and_go(self, target_x, target_y):
        """
        Berekent een positie op de lijn tussen Robot en Fles.
        """
        # Waar ben ik nu?
        try:
            tf_robot = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = tf_robot.transform.translation.x
            robot_y = tf_robot.transform.translation.y
        except Exception as e:
            self.get_logger().error("Kan eigen positie niet bepalen. TF fout.")
            self.is_busy = False
            return

        # Vector berekening: Van Fles -> Richting Robot
        dx = robot_x - target_x
        dy = robot_y - target_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Check: Zijn we er al?
        if dist < STOP_DISTANCE + 0.1: # Kleine marge
            self.get_logger().info("Ik sta al dichtbij genoeg! Tijd om te grijpen.")
            self.is_busy = False
            # HIER: Zou je gripper logica kunnen starten
            return

        # Bereken stoppunt (stop_distance voor de fles)
        # We normaliseren de vector en vermenigvuldigen met de gewenste afstand vanaf de fles
        # Formule: Target = Fles + (Richting_naar_robot * stop_afstand)
        scale = STOP_DISTANCE / dist
        goal_x = target_x + (dx * scale)
        goal_y = target_y + (dy * scale)

        # Bereken rotatie: De robot moet naar de fles kijken
        # Hoek is van goal -> fles (dus tegengesteld aan dx, dy)
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)

        self.get_logger().info(f"Rijden naar positie: [{goal_x:.2f}, {goal_y:.2f}]")

        # Maak Pose bericht
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0 # Altijd 0 voor navigatie
        pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.send_nav_goal(pose)

    def send_nav_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal afgewezen door Nav2.')
            self.is_busy = False
            return
        
        self.get_logger().info('Goal geaccepteerd. Onderweg...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        
        # Status 4 = SUCCEEDED in ROS 2 Action Clients
        if status == 4:
            self.get_logger().info('✅ SUCCES: Aangekomen bij de fles!')
        else:
            self.get_logger().warn(f'❌ MISLUKT of AFGEBROKEN. Status code: {status}')
        
        self.is_busy = False 

    # --- HULPFUNCTIES ---

    def transform_camera_to_map(self, x, y, z):
        try:
            p = PointStamped()
            p.header.frame_id = "camera_link"
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x = x
            p.point.y = y
            p.point.z = z

            if not self.tf_buffer.can_transform('map', 'camera_link', rclpy.time.Time()):
                return None, None

            transform = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            p_map = do_transform_point(p, transform)
            return p_map.point.x, p_map.point.y
        except Exception:
            return None, None

    def is_safe_to_grab(self, obj):
        depth = self.latest_depth_img
        x1, y1, x2, y2 = int(obj.xmin), int(obj.ymin), int(obj.xmax), int(obj.ymax)
        z = obj.identity_depth

        if z <= 0 or x1 < 0 or y1 < 0 or x2 >= CAMERA_PARAMS['width'] or y2 >= CAMERA_PARAMS['height']:
            return False

        px_margin = self.pixels_from_meters(z, MARGIN_M)
        
        # Marges
        y_scan_top = y1
        y_scan_bottom = max(y1, y2 - px_margin)
        left_x = max(x1 - px_margin, 0)
        right_x = min(x2 + px_margin, CAMERA_PARAMS['width'] - 1)
        bottom_y = min(y2 + px_margin, CAMERA_PARAMS['height'] - 1)

        # Checks
        left_clear = np.all(depth[y_scan_top:y_scan_bottom, left_x] > z + OCCLUSION_CLEARANCE_M)
        right_clear = np.all(depth[y_scan_top:y_scan_bottom, right_x] > z + OCCLUSION_CLEARANCE_M)
        bottom_clear = np.all(depth[bottom_y, x1:x2] > z - BOTTOM_CLEARANCE_M)

        # Oppervlakte check
        mask = (depth[y1:y2, x1:x2] <= z + OCCLUSION_CLEARANCE_M)
        has_surface = False
        for r in range(mask.shape[0]):
            if len(np.where(mask[r])[0]) >= 2:
                has_surface = True
                break

        return left_clear and right_clear and bottom_clear and has_surface

    def pixels_from_meters(self, depth, meters):
        fov = CAMERA_PARAMS['FOV']
        w = CAMERA_PARAMS['width']
        px_per_meter = (w / 2) / (math.tan(fov / 2) * depth)
        return int(meters * px_per_meter)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = SmartFetcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()