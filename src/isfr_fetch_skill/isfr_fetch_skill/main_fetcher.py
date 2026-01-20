import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener  # <--- NIEUW: Nodig om positie te bepalen

# Importeer de klasse uit hetzelfde mapje
from .location_check import LocationChecker

class BottleManager(Node):
    def __init__(self):
        super().__init__('main_fetcher')
        
        # Initialiseer de interne checker (voor costmap checks)
        self.checker = LocationChecker(self)
        
        # --- NIEUW: TF Buffer om eigen positie te bepalen voor afstandsmeting ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # ------------------------------------------------------------------------

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Flessen database
        self.bottle_list = [
            {'id': 1, 'x': 100, 'y': 100, 'status': 'new'},
            {'id': 2, 'x': 4.0, 'y': 3.0,  'status': 'new'}, 
            {'id': 3, 'x': 3.0, 'y': 2.0,  'status': 'new'}  
        ]
        
        #TODO: hier moet dan een subscriber komen die de flessen updates binnenkrijgt

        self.timer = self.create_timer(2.0, self.process_bottles)
        self.is_busy = False

    def process_bottles(self):
        # 1. Ben ik al bezig?
        if self.is_busy: 
            return

        # 2. Heb ik al een kaart?
        if self.checker.latest_costmap is None:
            self.get_logger().info("⏳ Wachten op Nav2 Costmap...", throttle_duration_sec=2.0)
            return

        # 3. Zijn er nog flessen?
        candidates = [b for b in self.bottle_list if b['status'] == 'new']
        if not candidates:
            self.get_logger().info("🎉 Alle flessen zijn afgehandeld!")
            return

        # --- NIEUW: Sorteer op afstand (Dichtstbijzijnde eerst) ---
        try:
            # Waar is de robot nu?
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            
            # Sorteer functie: Bereken afstand (hypothenusa) voor elke fles
            # lambda b: math.hypot(b['x'] - rx, b['y'] - ry)
            candidates.sort(key=lambda b: math.hypot(b['x'] - rx, b['y'] - ry))
            
            self.get_logger().info(f"📏 Lijst gesorteerd! Dichtstbijzijnde is Fles {candidates[0]['id']}")
            
        except Exception as e:
            self.get_logger().warn(f"Kon afstand niet berekenen (TF nog niet klaar), volgorde is willekeurig. Fout: {e}")
        # ----------------------------------------------------------

        self.get_logger().info(f"Start ronde... {len(candidates)} flessen te gaan.")
        
        chosen_approach = None
        chosen_bottle = None

        # 4. Zoek de eerste bereikbare fles (nu dus de dichtstbijzijnde!)
        for bottle in candidates:
            self.get_logger().info(f"Checken van fles {bottle['id']}...")
            
            # LET OP: stop_dist=0.75 toegevoegd om de Collision Monitor error te voorkomen!
            result = self.checker.get_safe_approach_point(bottle['x'], bottle['y'], stop_dist=0.75)
            
            if result is not None:
                goal_x, goal_y, goal_yaw = result
                self.get_logger().info(f"🚀 Fles {bottle['id']} is bereikbaar! Ga naar [{goal_x:.2f}, {goal_y:.2f}]")
                chosen_approach = (goal_x, goal_y, goal_yaw)
                chosen_bottle = bottle
                break 
            else:
                self.get_logger().warn(f"❌ Fles {bottle['id']} is dichtbij maar onbereikbaar.")
                bottle['status'] = 'unreachable'

        # 5. Actie ondernemen
        if chosen_approach:
            self.is_busy = True
            chosen_bottle['status'] = 'processing'
            self.send_goal(*chosen_approach)

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.get_logger().info(f"Navigeren naar: x={x:.2f}, y={y:.2f}...")
        
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 heeft het doel geweigerd!")
            self.is_busy = False
            return
        
        self.get_logger().info("Doel geaccepteerd, rijden maar...")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
            self.get_logger().info("🎯 Aangekomen bij de fles!")
            self.update_bottle_status('processing', 'done')
        else:
            self.get_logger().warn(f"Navigatie mislukt. Status code: {result.status}")
            self.update_bottle_status('processing', 'failed')
        
        self.is_busy = False

    def update_bottle_status(self, old_status, new_status):
        for b in self.bottle_list:
            if b['status'] == old_status:
                b['status'] = new_status

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw/2)
        q.w = math.cos(yaw/2)
        return q

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BottleManager()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        print("INFO: Main Fetcher gestart (Met afstands-sortering).")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if 'executor' in locals(): executor.shutdown()
        if 'node' in locals(): node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()