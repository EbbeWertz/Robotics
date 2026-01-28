#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# TF2 imports for saving start location
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
# Import the FetchObject and the new ApproachAndGrabTask
from isfr_bot_msgs.action import FetchObject, ApproachAndGrabTask 
from .location_check import LocationChecker

map_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

class BottleFetcherActionServer(Node):
    def __init__(self):
        super().__init__('bottle_fetcher_server')

        self.checker = LocationChecker(self)
        
        # 1. TF Listener to get current robot pose (for returning later)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 3. Grabbing Action Client (New)
        self._grab_client = ActionClient(self, ApproachAndGrabTask, 'approach_and_grab')

        # Knowledge Map Subscription
        self.latest_map = None
        self.create_subscription(OccupancyGrid, '/knowledge/occupancy', self.map_callback, map_qos)

        # Our own Action Server
        self._action_server = ActionServer(
            self,
            FetchObject,
            'fetch_object',
            execute_callback=self.execute_callback
        )

        self.get_logger().info("Fetch Action Server started. Ready for missions.")

    def map_callback(self, msg):
        self.latest_map = msg

    def get_current_pose(self):
        """Helper to get current (x, y, yaw) from TF"""
        try:
            # Look up transform from map to base_link
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(), 
                timeout=Duration(seconds=1.0)
            )
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Convert Quaternion to Yaw
            q = t.transform.rotation
            # standard math to convert quaternion to euler yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Could not get current pose via TF: {e}")
            return None

    async def execute_callback(self, goal_handle):
        target_label = goal_handle.request.label
        self.get_logger().info(f"Received mission: Fetch '{target_label}'")

        # --- STEP 1: Save Start Position (Home) ---
        start_pose = self.get_current_pose()
        if start_pose is None:
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Could not determine start pose")
        
        start_x, start_y, start_yaw = start_pose
        self.get_logger().info(f"Start pose saved: ({start_x:.2f}, {start_y:.2f}). Will return here later.")

        # --- STEP 2: Find Object in Map ---
        if self.latest_map is None:
            self.get_logger().error("No knowledge map available!")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Map not found")

        target = self.select_best_cell(self.latest_map)
        
        if target is None:
            self.get_logger().warn(f"No object labeled '{target_label}' found in grid.")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Object not found in grid")

        tx, ty = target
        
        # --- STEP 3: Calculate Approach Pose ---
        approach = self.checker.get_safe_approach_point(tx, ty, stop_dist=0.75)
        
        if not approach:
            self.get_logger().error("Target found, but unreachable!")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Location unreachable")

        gx, gy, yaw = approach
        
        # --- STEP 4: Navigate TO Bottle ---
        self.get_logger().info(f"Navigating to approach point: ({gx:.2f}, {gy:.2f})")
        nav_success = await self.go_to_pose(gx, gy, yaw)

        if not nav_success:
            self.get_logger().error("Navigation to bottle failed.")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Navigation failed")
        
        self.get_logger().info("Arrived at bottle. Initiating grab sequence...")

        # --- STEP 5: Execute GRAB Action ---
        grab_success = await self.execute_grab(target_label)
        
        if not grab_success:
            self.get_logger().error("Grabbing failed.")
            # Depending on your logic, you might still want to return home here. 
            # For now, we abort.
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Grabbing failed")

        self.get_logger().info("Grab successful! Returning to start...")

        # --- STEP 6: Navigate BACK to Start ---
        return_success = await self.go_to_pose(start_x, start_y, start_yaw)

        if return_success:
            self.get_logger().info("Mission Complete: Returned to start with object.")
            goal_handle.succeed()
            return FetchObject.Result(success=True, message="Object fetched and returned")
        else:
            self.get_logger().error("Failed to return to start.")
            goal_handle.abort()
            return FetchObject.Result(success=False, message="Failed to return home")

    def select_best_cell(self, map_msg):
        width, height = map_msg.info.width, map_msg.info.height
        res = map_msg.info.resolution
        ox, oy = map_msg.info.origin.position.x, map_msg.info.origin.position.y
        
        data = np.array(map_msg.data).reshape((height, width))
        if np.max(data) < 30: 
            return None

        y, x = np.unravel_index(np.argmax(data), data.shape)
        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res
        return wx, wy

    async def go_to_pose(self, x, y, yaw):
        """Sends goal to Nav2 and waits for result"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self._nav_client.wait_for_server()
        send_goal_future = await self._nav_client.send_goal_async(goal_msg)
        
        if not send_goal_future.accepted:
            return False

        result_future = await send_goal_future.get_result_async()
        return result_future.status == 4 # STATUS_SUCCEEDED

    async def execute_grab(self, label):
        """Triggers the /approach_and_grab action"""
        if not self._grab_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Grab action server not available!")
            return False

        goal_msg = ApproachAndGrabTask.Goal()
        goal_msg.target_label = label

        self.get_logger().info(f"Sending grab goal for: {label}")
        send_goal_future = await self._grab_client.send_goal_async(goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().error("Grab goal rejected")
            return False

        # We can optionally hook up feedback_callback here if we want to print states
        # send_goal_future.add_done_callback(...) 
        
        result_future = await send_goal_future.get_result_async()
        
        # Check result status (4 = SUCCEEDED)
        if result_future.status == 4:
            self.get_logger().info(f"Grab result: {result_future.result.message}")
            return result_future.result.success
        return False

def main(args=None):
    rclpy.init(args=args)
    node = BottleFetcherActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()