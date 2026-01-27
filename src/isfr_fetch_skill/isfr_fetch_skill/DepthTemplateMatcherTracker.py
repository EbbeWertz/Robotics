import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthTemplateRefiner:
    def __init__(self, node, search_margin=0.25):
        self.node = node
        self.bridge = CvBridge()
        self.template = None
        self.initial_z = 1.0
        self.orig_h = 0
        self.orig_w = 0
        self.search_margin = search_margin 
        
        # Geometry offsets
        self.norm_dist_left = 0.0
        self.norm_dist_right = 0.0
        self.norm_dist_top = 0.0
        self.norm_dist_bottom = 0.0

        # Dedicated Publisher
        self.debug_pub = self.node.create_publisher(Image, '/vision/refiner_internal_debug', 10)

    def initialize(self, depth_img, bbox_x, bbox_y, bbox_w, bbox_h, grasp_u, grasp_v, z_depth):
        h_img, w_img = depth_img.shape
        x1, y1 = max(0, int(bbox_x)), max(0, int(bbox_y))
        x2, y2 = min(w_img, int(bbox_x + bbox_w)), min(h_img, int(bbox_y + bbox_h))
        
        if x2 <= x1 or y2 <= y1: return False

        self.template = depth_img[y1:y2, x1:x2].copy()
        self.template = np.nan_to_num(self.template, nan=0.0)
        self.initial_z = z_depth
        self.orig_h, self.orig_w = self.template.shape

        self.norm_dist_left = (grasp_u - x1) / self.orig_w
        self.norm_dist_right = (x2 - grasp_u) / self.orig_w
        self.norm_dist_top = (grasp_v - y1) / self.orig_h
        self.norm_dist_bottom = (y2 - grasp_v) / self.orig_h
        
        return True

    def track(self, depth_img, guess_u, guess_v, current_z_estimate):
        if self.template is None: return None
        img_h, img_w = depth_img.shape

        scale = self.initial_z / max(0.1, current_z_estimate)
        cur_w, cur_h = self.orig_w * scale, self.orig_h * scale

        # Construct Search ROI
        mw, mh = cur_w * self.search_margin, cur_h * self.search_margin
        roi_x1 = max(0, int(guess_u - (cur_w * self.norm_dist_left) - mw))
        roi_x2 = min(img_w, int(guess_u + (cur_w * self.norm_dist_right) + mw))
        roi_y1 = max(0, int(guess_v - (cur_h * self.norm_dist_top) - mh))
        roi_y2 = min(img_h, int(guess_v + (cur_h * self.norm_dist_bottom) + mh))

        scaled_template = cv2.resize(self.template, (int(cur_w), int(cur_h)))
        
        if (roi_x2 - roi_x1) < scaled_template.shape[1] or (roi_y2 - roi_y1) < scaled_template.shape[0]:
            return None

        roi_img = np.nan_to_num(depth_img[roi_y1:roi_y2, roi_x1:roi_x2], nan=0.0)
        res = cv2.matchTemplate(roi_img, scaled_template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        # Map Results
        tlx, tly = roi_x1 + max_loc[0], roi_y1 + max_loc[1]
        refined_u = int(tlx + (cur_w * self.norm_dist_left))
        refined_v = int(tly + (cur_h * self.norm_dist_top))
        
        # Internal Dashboard Publish
        self._publish_internal_debug(scaled_template, roi_img, res, max_val)

        debug_info = {"roi_rect": (roi_x1, roi_y1, roi_x2, roi_y2)}
        return (refined_u, refined_v, (tlx, tly, int(cur_w), int(cur_h)), debug_info)

    def _publish_internal_debug(self, template, roi, heatmap, score):
        """Creates a horizontal dashboard of the CV process with title bars."""
        def norm(img): 
            return cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # 1. Convert to BGR
        t_view = cv2.cvtColor(norm(template), cv2.COLOR_GRAY2BGR)
        r_view = cv2.cvtColor(norm(roi), cv2.COLOR_GRAY2BGR)
        h_view = cv2.applyColorMap(norm(heatmap), cv2.COLORMAP_JET)

        # 2. Standardization params
        panel_h = 240 # Fixed height for horizontal stacking
        title_h = 30  # Height of the black title bar
        
        def prepare_panel(img, label):
            # Resize image to fixed height
            h, w = img.shape[:2]
            new_w = int(w * (panel_h / h))
            img_resized = cv2.resize(img, (new_w, panel_h))
            
            # Create a black title bar
            title_bar = np.zeros((title_h, new_w, 3), dtype=np.uint8)
            cv2.putText(title_bar, label, (5, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Stack title bar on top of image
            return cv2.vconcat([title_bar, img_resized])

        # 3. Process panels
        p1 = prepare_panel(t_view, "TEMPLATE")
        p2 = prepare_panel(r_view, "SEARCH ROI")
        p3 = prepare_panel(h_view, f"MATCH HEATMAP (Score: {score:.2f})")

        # 4. Horizontal Concatenation
        # Add a small 2px white border/spacer between panels for clarity
        spacer = np.ones((panel_h + title_h, 2, 3), dtype=np.uint8) * 255
        dashboard = cv2.hconcat([p1, spacer, p2, spacer, p3])
        
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dashboard, 'bgr8'))