import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.executors import MultiThreadedExecutor

# Importeer je eigen klasse (zorg dat location_check.py in dezelfde map staat)
from .location_check import LocationChecker

class BottleManager(Node):
    def __init__(self):
        super().__init__('bottle_manager')
                
        # 1. Initialiseer de Checker
        # We geven 'self' mee zodat de checker onze node kan gebruiken voor subscriptions
        self.checker = LocationChecker(self)
        
        # 2. Navigatie Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 3. Dummy Data: De lijst met flessen (uit je map/database)
        # In het echt zou je dit uit een file of message halen
        self.bottle_list = [
            {'id': 1, 'x': 5.4, 'y': -1.5, 'status': 'new'}, # Fles A
            {'id': 2, 'x': 5.0, 'y': 5.0,  'status': 'new'}, # Fles B
            {'id': 3, 'x': 9.5, 'y': -2.0,  'status': 'new'}  # Fles C (Staat in de muur/origin, zou moeten falen)
        ]
        
        #TODO: In echte code zou je hier een subscriber maken die flessen updates krijgt
        #TODO Bijvoorbeeld:
        #TODO self.bottle_sub = self.create_subscription(BottleArray, '/bottles', self.bottle_callback, 10)

        # Start na 1 seconde met denken
        self.timer = self.create_timer(1.0, self.process_bottles)
        self.is_busy = False

    def process_bottles(self):
        if self.is_busy:
            return
        
        if self.checker.latest_costmap is None:
            self.get_logger().info("⏳ Wachten op Nav2 Costmap... (Navigatie start nog op)", throttle_duration_sec=2.0)
            return

        # Stap A: Zoek flessen die we nog moeten doen
        candidates = [b for b in self.bottle_list if b['status'] == 'new']
        if not candidates:
            self.get_logger().info("Alle flessen zijn afgehandeld!")
            return

        self.get_logger().info(f"Nog {len(candidates)} flessen te gaan. Ik zoek de beste...")

        # Stap B: Sorteer op afstand (optioneel, maar slim)
        # Voor nu pakken we gewoon de eerste uit de lijst
        
        chosen_approach = None
        chosen_bottle = None

        # Stap C: Vraag de LocationChecker om advies per fles
        for bottle in candidates:
            self.get_logger().info(f"Checking fles {bottle['id']} op [{bottle['x']}, {bottle['y']}]...")
            
            # HIER GEBRUIKEN WE JOUW NIEUWE KLASSE:
            result = self.checker.get_safe_approach_point(bottle['x'], bottle['y'])
            
            if result is not None:
                # We hebben een veilige plek gevonden!
                goal_x, goal_y, goal_yaw = result
                self.get_logger().info(f"✅ GOEDGEKEURD! We kunnen staan op [{goal_x:.2f}, {goal_y:.2f}]")
                
                chosen_approach = (goal_x, goal_y, goal_yaw)
                chosen_bottle = bottle
                break # We hebben er een gevonden, stop met zoeken
            else:
                self.get_logger().warn(f"❌ AFGEKEURD! Fles {bottle['id']} is onbereikbaar (muur/tafel).")
                bottle['status'] = 'unreachable' # Markeer als onmogelijk

        # Stap D: Voer uit als we iets gevonden hebben
        if chosen_approach:
            self.is_busy = True
            chosen_bottle['status'] = 'processing'
            self.send_goal(*chosen_approach)
        else:
            self.get_logger().info("Geen bereikbare flessen gevonden in deze ronde.")

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.get_logger().info(f"Navigeren naar positie...")
        self._nav_client.wait_for_server()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_busy = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
            self.get_logger().info("🎯 Aangekomen bij de fles!")
            # Zoek de huidige fles en zet op 'done'
            for b in self.bottle_list:
                if b['status'] == 'processing': b['status'] = 'done'
        else:
            self.get_logger().warn("Mislukt.")
            for b in self.bottle_list:
                if b['status'] == 'processing': b['status'] = 'failed'
        
        self.is_busy = False

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw/2); q.w = math.cos(yaw/2)
        return q

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BottleManager()
        
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        print("INFO: Multi-threaded node gestart. Wachten op costmap...")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()