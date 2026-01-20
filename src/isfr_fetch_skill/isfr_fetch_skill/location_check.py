import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

class LocationChecker:
    def __init__(self, node):
        self.node = node
        self.latest_costmap = None
        
        # 1. TF Setup (om te weten waar de robot zelf is)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        # 2. Callback Group voor Multithreading
        # Dit zorgt ervoor dat de map binnenkomt TERWIJL de robot aan het rekenen is
        self.cb_group = ReentrantCallbackGroup()

        # 3. Costmap Subscriber met de JUISTE QoS
        # Nav2 Global Costmap is vaak 'Transient Local'. Wij moeten dat matchen.
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )

        self.costmap_sub = node.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.costmap_callback, 
            qos_profile,
            callback_group=self.cb_group
        )

    def costmap_callback(self, msg):
        # We slaan de kaart alleen op, verwerking doen we in de main loop
        self.latest_costmap = msg
        # Debug printje (kun je later weghalen)
        # self.node.get_logger().info(f"Costmap update ontvangen! ({msg.info.width}x{msg.info.height})")

    def get_safe_approach_point(self, target_x, target_y, stop_dist=0.50):
        if self.latest_costmap is None:
            return None

        # --- SLIMME ZOEKSTRATEGIE ---
        # We proberen 16 punten in een cirkel rond de fles (elke 22.5 graad).
        # We beginnen bij de hoek tussen robot en fles, en waaien dan uit naar links/rechts.
        
        steps = 16 
        
        # Stap A: Bepaal waar de robot nu is (voor de start-hoek)
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            start_angle = math.atan2(ry - target_y, rx - target_x)
        except Exception as e:
            self.node.get_logger().warn(f"TF Fout: {e}")
            start_angle = 0.0

        # Stap B: Loop door de hoeken heen
        for i in range(steps):
            # Bereken offset: 0, +22, -22, +45, -45, etc.
            if i % 2 == 0:
                angle_offset = (math.pi * 2 / steps) * (i/2)
            else:
                angle_offset = -(math.pi * 2 / steps) * ((i+1)/2)
            
            angle = start_angle + angle_offset

            # Bereken het punt op de cirkel (0.5m van de fles)
            goal_x = target_x + math.cos(angle) * stop_dist
            goal_y = target_y + math.sin(angle) * stop_dist

            # Stap C: Is dit punt veilig?
            if self._is_point_safe_on_map(goal_x, goal_y):
                # GEVONDEN!
                # Bereken de oriëntatie: Robot moet NAAR de fles kijken
                final_yaw = math.atan2(target_y - goal_y, target_x - goal_x)
                
                self.node.get_logger().info(f"✅ Route gevonden via hoek {math.degrees(angle):.0f}°")
                return (goal_x, goal_y, final_yaw)

        # Als we hier komen, zijn alle 16 punten onveilig
        return None

    def _is_point_safe_on_map(self, wx, wy):
        info = self.latest_costmap.info
        
        # Omrekenen van Wereld (meters) naar Grid (pixels)
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)

        # Buiten de kaart?
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        index = my * info.width + mx
        cost = self.latest_costmap.data[index]
        
        # --- CRITERIA VOOR VEILIGHEID ---
        # Cost -1  = Onbekend (niet doen)
        # Cost 100 = Muur (niet doen)
        # Cost 99  = Inscribed Obstacle (niet doen)
        # Cost 1-98 = Inflation zone (dichtbij obstakel).
        # Omdat we expres dichtbij de fles (0.5m) willen stoppen, 
        # accepteren we alles behalve een botsing.
        
        if cost == -1: return False
        if cost >= 98: return False # Te dicht op obstakel/muur
        
        return True # Alles < 98 is acceptabel