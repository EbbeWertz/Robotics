import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion

# Importeer de klasse uit hetzelfde mapje (let op de punt!)
from .location_check import LocationChecker

class BottleManager(Node):
    def __init__(self):
        super().__init__('main_fetcher')
        
        # Initialiseer de interne checker
        self.checker = LocationChecker(self)
        
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Flessen database
        self.bottle_list = [
            {'id': 1, 'x': 2.5, 'y': -0.5, 'status': 'new'}, 
            {'id': 2, 'x': 3.0, 'y': 2.0,  'status': 'new'}, 
            {'id': 3, 'x': 0.0, 'y': 0.0,  'status': 'new'}  
        ]
        
        #TODO: In echte code zou je hier een subscriber maken die flessen updates krijgt
        #TODO Bijvoorbeeld:
        #TODO self.bottle_sub = self.create_subscription(BottleArray, '/bottles', self.bottle_callback, 10)

        # Timer: checkt elke 2 seconden wat we moeten doen
        self.timer = self.create_timer(2.0, self.process_bottles)
        self.is_busy = False

    def process_bottles(self):
        # 1. Ben ik al aan het rijden?
        if self.is_busy: 
            return

        # 2. Heb ik al een kaart? (Essentiële check!)
        if self.checker.latest_costmap is None:
            self.get_logger().info("⏳ Wachten op Nav2 Costmap... (Nav stack start op)", throttle_duration_sec=2.0)
            return

        # 3. Zijn er nog flessen?
        candidates = [b for b in self.bottle_list if b['status'] == 'new']
        if not candidates:
            self.get_logger().info("🎉 Alle flessen zijn afgehandeld!")
            # self.timer.cancel() # Optioneel: stop de timer
            return

        self.get_logger().info(f"Start ronde... {len(candidates)} flessen te gaan.")
        
        chosen_approach = None
        chosen_bottle = None

        # 4. Zoek de eerste bereikbare fles
        for bottle in candidates:
            self.get_logger().info(f"Checken van fles {bottle['id']}...")
            
            # Hier roepen we de slimme functie aan
            result = self.checker.get_safe_approach_point(bottle['x'], bottle['y'])
            
            if result is not None:
                goal_x, goal_y, goal_yaw = result
                self.get_logger().info(f"🚀 Fles {bottle['id']} is bereikbaar! Ga naar [{goal_x:.2f}, {goal_y:.2f}]")
                chosen_approach = (goal_x, goal_y, goal_yaw)
                chosen_bottle = bottle
                break # We hebben er een gevonden, stop met zoeken
            else:
                self.get_logger().warn(f"❌ Fles {bottle['id']} is momenteel onbereikbaar.")
                bottle['status'] = 'unreachable' # Markeer als gefaald voor nu

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
        # Status 4 = SUCCEEDED
        if result.status == 4: 
            self.get_logger().info("🎯 Aangekomen bij de fles!")
            self.update_bottle_status('processing', 'done')
        else:
            self.get_logger().warn(f"Navigatie mislukt/afgebroken. Status code: {result.status}")
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
        
        # CRUCIAAL: Gebruik de MultiThreadedExecutor
        # Hierdoor kan de Costmap Callback draaien terwijl de Timer/Navigatie bezig is.
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        print("INFO: Main Fetcher gestart (Multi-threaded).")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'executor' in locals():
            executor.shutdown()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()