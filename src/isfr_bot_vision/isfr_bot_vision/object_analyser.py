import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from isfr_bot_msgs.msg import YoloVisionObjectArray, GraspSafeObjectArray, GraspSafeObject

CAMERA_PARAMS = {
    "width": 640,
    "height": 480,
    "FOV": 1.57
}
# --- Gripper / safety parameters (meters) ---
MARGIN_M              = 0.02   # 2 cm scan margin (finger width clearance)
OCCLUSION_CLEARANCE_M = 0.05   # 5 cm finger depth clearance
BOTTOM_CLEARANCE_M    = 0.15   # 10 cm table edge rule

DEBUG_WINDOW = False



class GripperSafetyNode(Node):
    def __init__(self):
        super().__init__('gripper_safety')

        self.bridge = CvBridge()
        self.latest_depth = None

        # Subscribe to depth
        self.create_subscription(
            Image,
            '/isfr/camera_sensor/depth/image',
            self.depth_callback,
            10
        )
        # Subscribe to detected objects
        self.create_subscription(
            YoloVisionObjectArray,
            '/vision/objects',
            self.objects_callback,
            10
        )

        self.publisher = self.create_publisher(GraspSafeObjectArray, '/vision/grasp_safe_objects', 10)

        # Debug publisher
        self.debug_pub = self.create_publisher(Image, '/vision/gripper_debug', 10)
        


    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def pixels_from_meters(self, depth, meters):
        # Approximate number of pixels corresponding to meters at given depth
        # px = tan(FOV/2) * 2 * depth * (width / 2) / depth
        fov = CAMERA_PARAMS['FOV']
        w = CAMERA_PARAMS['width']
        h = CAMERA_PARAMS['height']
        px_per_meter = (w / 2) / (math.tan(fov / 2) * depth)
        return int(meters * px_per_meter)
    
    def is_column_clear(self, col, z, clearance):
        valid = np.isfinite(col)  # ignore NaNs and inf
        if not np.any(valid):
            return True  # nothing detected here → treat as clear
        return np.all(col[valid] > z + clearance)


    def objects_callback(self, msg):
        if self.latest_depth is None:
            return

        depth = self.latest_depth
        debug_img = np.zeros(
            (CAMERA_PARAMS["height"], CAMERA_PARAMS["width"], 3),
            dtype=np.uint8
        )

        unsafe_draws = []
        safe_draws = []

        object_array_msg = GraspSafeObjectArray()
        object_array_msg.stamp = self.get_clock().now().to_msg()

        for obj in msg.objects:
            x1, y1, x2, y2 = map(int, [obj.xmin, obj.ymin, obj.xmax, obj.ymax])
            z = obj.identity_depth
            label = obj.label
            bb_width = x2-x1
            bb_height = y2-y1

            px_margin = self.pixels_from_meters(z, MARGIN_M)

            # Vertical scan range (stop above table)
            y_scan_top = y1
            y_scan_bottom = max(y1, y2 - px_margin*2)

            left_x = max(x1 - px_margin, 0)
            right_x = min(x2 + px_margin, CAMERA_PARAMS["width"] - 1)

            left_col = depth[y_scan_top:y_scan_bottom, left_x]
            right_col = depth[y_scan_top:y_scan_bottom, right_x]

            left_clear = self.is_column_clear(left_col, z, OCCLUSION_CLEARANCE_M)
            right_clear = self.is_column_clear(right_col, z, OCCLUSION_CLEARANCE_M)

            if DEBUG_WINDOW:
                column_violation_mask = np.zeros_like(depth, dtype=np.uint8)
                unsafe_left = (np.isfinite(left_col)) & (left_col <= z + OCCLUSION_CLEARANCE_M)
                column_violation_mask[y_scan_top:y_scan_bottom, left_x] = unsafe_left.astype(np.uint8) * 255
                unsafe_right = (np.isfinite(right_col)) & (right_col <= z + OCCLUSION_CLEARANCE_M)
                column_violation_mask[y_scan_top:y_scan_bottom, right_x] = unsafe_right.astype(np.uint8) * 255
                vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                vis_color = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
                vis_color[column_violation_mask == 255] = [0,0,255]  # highlight violations in red
                cv2.imshow("Column Clearance Violations", vis_color)
                cv2.waitKey(1)  # n



            # --- Bottom margin ---
            bottom_y = min(y2 + px_margin, CAMERA_PARAMS["height"] - 1)
            bottom_row = depth[bottom_y, x1:x2]
            bottom_clear = np.all(bottom_row > z - BOTTOM_CLEARANCE_M)

            safe = left_clear and right_clear and bottom_clear
            scan_geom = (left_x, right_x, y_scan_top, y_scan_bottom, bottom_y, x1, x2)


            if not safe:
                unsafe_draws.append((x1,y1,x2,y2,left_clear,right_clear,bottom_clear,scan_geom))
                continue

            # --- Depth mask ---
            mask = (depth[y1:y2, x1:x2] <= z + OCCLUSION_CLEARANCE_M)

            best_y = y1
            best_width = 0
            best_cols = None


            for r in range(mask.shape[0] - px_margin*3, 0, -1):
                cols = np.where(mask[r])[0]
                if len(cols) < 2:
                    continue
                w = cols[-1] - cols[0]
                if w > best_width:
                    best_width = w
                    best_y = y1 + r
                    best_cols = cols

            if best_cols is None:
                # No feasible grasp line
                unsafe_draws.append((x1,y1,x2,y2,True,True,True, scan_geom))
                continue
            
            safe_draws.append((mask, x1, y1, best_y, best_cols, scan_geom))

            object_msg = GraspSafeObject()
            object_msg.label = label
            object_msg.xmin = float(x1)
            object_msg.ymin = float(y1)
            object_msg.xmax = float(x2)
            object_msg.ymax = float(y2)
            object_msg.graspline_u = float(best_cols[0] / bb_width)
            object_msg.graspline_v = float((best_y-y1) / bb_height)
            object_msg.graspline_width = float(best_width / bb_width)
            object_array_msg.objects.append(object_msg)
        

        self.publisher.publish(object_array_msg)
        # --- Draw UNSAFE first ---
        for x1,y1,x2,y2,l_ok,r_ok,b_ok, geom in unsafe_draws:
            cv2.rectangle(debug_img, (x1,y1), (x2,y2), (0,165,255), 2)

            if not l_ok:
                cv2.line(debug_img, (x1,y1), (x1,y2), (0,0,255), 4)
            if not r_ok:
                cv2.line(debug_img, (x2,y1), (x2,y2), (0,0,255), 4)
            if not b_ok:
                cv2.line(debug_img, (x1,y2), (x2,y2), (0,0,255), 4)

            scan_left_x, scan_right_x, scan_y_top, scan_y_bot, scan_bottom_y, scan_x1, scan_x2 = geom
            cv2.line(debug_img, (scan_left_x, scan_y_top), (scan_left_x, scan_y_bot), (255,255,255), 1)
            cv2.line(debug_img, (scan_right_x, scan_y_top), (scan_right_x, scan_y_bot), (255,255,255), 1)
            cv2.line(debug_img, (scan_x1, scan_bottom_y), (scan_x2, scan_bottom_y), (255,255,255), 1)

        # --- Draw SAFE on top ---
        for mask,x1,y1,grab_y,cols, geom in safe_draws:
            green = debug_img.copy()
            green[y1:y1+mask.shape[0], x1:x1+mask.shape[1]][mask] = (0,255,0)
            cv2.addWeighted(green, 0.6, debug_img, 0.4, 0, debug_img)

            cv2.line(
                debug_img,
                (x1 + cols[0], grab_y),
                (x1 + cols[-1], grab_y),
                (255,255,0),
                3
            )
            scan_left_x, scan_right_x, scan_y_top, scan_y_bot, scan_bottom_y, scan_x1, scan_x2 = geom
            cv2.line(debug_img, (scan_left_x, scan_y_top), (scan_left_x, scan_y_bot), (255,255,255), 1)
            cv2.line(debug_img, (scan_right_x, scan_y_top), (scan_right_x, scan_y_bot), (255,255,255), 1)
            cv2.line(debug_img, (scan_x1, scan_bottom_y), (scan_x2, scan_bottom_y), (255,255,255), 1)

        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        )



def main(args=None):
    rclpy.init(args=args)
    node = GripperSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
