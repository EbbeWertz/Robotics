import rclpy
import math
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy # <--- BELANGRIJK VOOR MAPS

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid # <--- Nodig voor type hint
from geometry_msgs.msg import Quaternion

# Zorg dat location_check.py in dezelfde map staat
from .location_check import LocationChecker

class BottleManager(Node):
    def __init__(self):
        super().__init__('bottle_manager')
        
        # Hulpklasse voor navigatie checks (blijft luisteren naar de echte costmap voor veiligheid)
        self.checker = LocationChecker(self)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- NIEUW: Subscriber voor jouw Knowledge Grid ---
        # De map wordt vaak als 'Transient Local' verstuurd (bewaard bericht), 
        # dus we moeten de QoS matchen.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid,
            '/vision/knowledge_grid', # <--- JOUW CUSTOM MAP TOPIC
            self.knowledge_map_callback,
            map_qos
        )
        self.latest_knowledge_map = None

        # De lijst begint leeg
        self.bottle_list = []
        self.map_scanned = False 
        self.current_bottle = None
        
        # Timer loop
        self.timer = self.create_timer(1.0, self.control_loop)

        self.get_logger().info("Bottle Manager gestart. Wachten op Knowledge Grid...")

    def knowledge_map_callback(self, msg):
        """Slaat de map op die uit je YoloDetector komt."""
        self.latest_knowledge_map = msg
        # We loggen dit maar 1 keer om spam te voorkomen
        self.get_logger().info("✅ Knowledge Grid ontvangen!", once=True)

    def control_loop(self):
        # Als we al bezig zijn, niets doen
        if self.current_bottle is not None:
            return

        # 1. Wachten op de Knowledge Map (van Yolo) EN de Costmap (van Nav2)
        if self.latest_knowledge_map is None:
            self.get_logger().info("⏳ Wachten op /vision/knowledge_grid...", throttle_duration_sec=3.0)
            return
        
        if self.checker.latest_costmap is None:
            self.get_logger().info("⏳ Wachten op lokale costmap (voor veiligheid)...", throttle_duration_sec=3.0)
            return

        # 2. SCAN DE KNOWLEDGE MAP (Dit doen we maar 1 keer)
        if not self.map_scanned:
            self.get_logger().info("🔍 Knowledge Grid analyseren op zoek naar flessen...")
            
            # Hier roepen we de functie aan met jouw specifieke map
            found_bottles = self.detect_bottles_from_map(self.latest_knowledge_map)
            
            if found_bottles:
                self.bottle_list = found_bottles
                self.get_logger().info(f"✅ {len(found_bottles)} flessen gevonden in de Knowledge Grid!")
                self.map_scanned = True
            else:
                self.get_logger().warn("⚠️ Nog geen flessen in de Knowledge Grid. Ik blijf wachten...")
                # We zetten scanned NIET op true, zodat hij blijft proberen tot YOLO iets vindt
                return

        # 3. Normale routine: Filteren op status 'new'
        candidates = [b for b in self.bottle_list if b['status'] == 'new']
        
        if not candidates:
            self.get_logger().info("🎉 Klaar! Alle detecteerde flessen zijn behandeld.")
            return

        # 4. Sorteren op afstand (Dichtstbijzijnde eerst)
        try:
            # Haal robot positie op
            tf = self.checker.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            # Sorteer lijst
            candidates.sort(key=lambda b: math.hypot(b['x'] - rx, b['y'] - ry))
        except:
            pass 

        # 5. Check bereikbaarheid en rijden
        for bottle in candidates:
            # We gebruiken de checker om te kijken of we er veilig kunnen komen
            # (stop_dist = afstand tot fles waar de robot stopt, bijv 0.5 meter)
            approach = self.checker.get_safe_approach_point(bottle['x'], bottle['y'], stop_dist=0.55)
            
            if approach:
                goal_x, goal_y, goal_yaw = approach
                self.get_logger().info(f"🚀 Fles {bottle['id']} gevonden op [{bottle['x']:.2f}, {bottle['y']:.2f}]. Rijden maar!")
                
                bottle['status'] = 'processing'
                self.current_bottle = bottle
                self.send_goal(goal_x, goal_y, goal_yaw)
                return 
            else:
                self.get_logger().warn(f"❌ Kandidaat {bottle['id']} is onbereikbaar (muur/obstakel).")
                bottle['status'] = 'unreachable'

    def detect_bottles_from_map(self, map_msg):
        """
        Converteert de Knowledge Grid naar coördinaten.
        Omdat jouw YoloDetector hier alleen 0 of 100 in zet, is dit heel makkelijk.
        """
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        # Data omzetten naar numpy matrix
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Maak beeld: Alles wat 100 is (fles) wordt wit (255)
        img = np.zeros((height, width), dtype=np.uint8)
        img[data >= 90] = 255 

        # Zoek de witte vlekken
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_bottles = []
        bottle_id_counter = 1

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Filter: Is het groot genoeg om een fles te zijn? (kleine ruis negeren)
            # Aangezien we in YoloDetector 3x3 pixels tekenen (=9 pixels), 
            # moet de area minstens > 1 zijn.
            if area > 1.0:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"]) # Grid X
                    cY = int(M["m01"] / M["m00"]) # Grid Y

                    # Grid -> World meters
                    world_x = origin_x + (cX * resolution) + (resolution / 2)
                    world_y = origin_y + (cY * resolution) + (resolution / 2)

                    detected_bottles.append({
                        'id': bottle_id_counter,
                        'x': world_x,
                        'y': world_y,
                        'status': 'new'
                    })
                    bottle_id_counter += 1

        return detected_bottles

    # --- NAVIGATIE ACTIES ---

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Doel geweigerd door Nav2.")
            if self.current_bottle: self.current_bottle['status'] = 'unreachable'
            self.current_bottle = None
            return
        
        self.get_logger().info("Doel geaccepteerd, onderweg...")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        # Status 4 = SUCCEEDED
        if result.status == 4:
            self.get_logger().info(f"🎯 Aangekomen bij fles {self.current_bottle['id']}!")
            self.current_bottle['status'] = 'done'
            # HIER ZOU JE JE ARM-CODE KUNNEN TRIGGEREN
        else:
            self.get_logger().warn("Mislukt om doel te bereiken.")
            self.current_bottle['status'] = 'failed'
        
        self.current_bottle = None

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw/2)
        q.w = math.cos(yaw/2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = BottleManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()