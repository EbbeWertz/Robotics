import math
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, Quaternion

# Zorg dat deze imports bovenaan staan!
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class LocationChecker:
    def __init__(self, node):
        self.node = node
        self.latest_costmap = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # --- DE OPLOSSING: QoS AANPASSEN ---
        # Nav2 Global Costmap gebruikt vaak TRANSIENT_LOCAL.
        # Als wij dat niet instellen, ontvangen we niks.
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # <--- DIT IS DE SLEUTEL
        )

        self.costmap_sub = node.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.costmap_callback, 
            qos_profile  # Gebruik het nieuwe profiel
        )

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def get_safe_approach_point(self, target_x, target_y, stop_dist=0.50):
        """
        Checkt of we veilig bij dit flesje kunnen komen.
        Returnt: (goal_x, goal_y, goal_yaw) OF None als het onveilig is.
        """
        if self.latest_costmap is None:
            self.node.get_logger().warn("LocationCheck: Nog geen costmap data!")
            return None
        else:
            self.node.get_logger().info("LocationCheck: Costmap data ontvangen.")

        # Huidige positie robot ophalen
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
        except Exception:
            self.node.get_logger().warn("LocationCheck: Kan robot positie niet bepalen.")
            return None

        # Bereken vector van Fles -> Robot
        dx = robot_x - target_x
        dy = robot_y - target_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Bereken het punt op 0.5m afstand
        scale = stop_dist / dist
        goal_x = target_x + (dx * scale)
        goal_y = target_y + (dy * scale)

        # Check in de map data
        if self._is_point_safe_on_map(goal_x, goal_y):
             # Bereken hoek: Robot moet naar de fles kijken
            yaw = math.atan2(target_y - goal_y, target_x - goal_x)
            return (goal_x, goal_y, yaw)
        else:
            return None

    def _is_point_safe_on_map(self, wx, wy):
        """Interne functie om grid cell te checken."""
        info = self.latest_costmap.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        index = my * info.width + mx
        cost = self.latest_costmap.data[index]
        
        # Cost < 50 is veilig (geen muur/tafelrand)
        # 100 is dodelijk, -1 is onbekend
        if cost != -1 and cost < 50:
            return True
        return False