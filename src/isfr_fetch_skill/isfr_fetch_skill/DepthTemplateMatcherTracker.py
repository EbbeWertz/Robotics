import numpy as np
import cv2

class DepthTemplateRefiner:
    def __init__(self):
        self.template = None
        self.initial_z = 1.0
        self.orig_h = 0
        self.orig_w = 0
        
        # Grab offset relative to bbox top-left
        self.grab_offset_u = 0
        self.grab_offset_v = 0

    def initialize(self, depth_img, bbox_x, bbox_y, bbox_w, bbox_h, grasp_u_global, grasp_v_global, z_depth):
        """
        Captures the initial template from the depth image.
        """
        # 1. Sanity check bounds
        h, w = depth_img.shape
        x1 = max(0, int(bbox_x))
        y1 = max(0, int(bbox_y))
        x2 = min(w, int(bbox_x + bbox_w))
        y2 = min(h, int(bbox_y + bbox_h))
        
        if x2 <= x1 or y2 <= y1:
            return False

        # 2. Crop and Store Template
        # We use the raw float depth. It is robust for shape matching.
        # We normalize it to handle lighting/sensor gain changes slightly better, 
        # though raw float matchTemplate works too.
        self.template = depth_img[y1:y2, x1:x2].copy()
        
        # Replace NaNs with 0 or max depth to avoid matching errors
        self.template = np.nan_to_num(self.template, nan=0.0)

        self.initial_z = z_depth
        self.orig_h, self.orig_w = self.template.shape
        
        # Store where the grasp point is relative to the top-left of the box
        self.grab_offset_u = grasp_u_global - x1
        self.grab_offset_v = grasp_v_global - y1
        
        return True

    def track(self, depth_img, guess_u, guess_v, current_z_estimate):
        """
        Refines the (u,v) position by searching around the guess.
        Returns: (refined_u, refined_v, refined_bbox_rect)
        """
        if self.template is None:
            return None

        img_h, img_w = depth_img.shape

        # 1. Calculate Scale Change
        # If object gets closer (smaller Z), scale increases.
        if current_z_estimate < 0.1: current_z_estimate = 0.1 # Safety
        scale = self.initial_z / current_z_estimate
        
        # 2. Resize Template
        # We want the template to match the object's current size in the image
        new_w = int(self.orig_w * scale)
        new_h = int(self.orig_h * scale)
        
        if new_w <= 0 or new_h <= 0 or new_w > img_w or new_h > img_h:
            return None # Object too big/small or out of frame

        scaled_template = cv2.resize(self.template, (new_w, new_h))

        # 3. Define Search ROI (Region of Interest)
        # We trust the Odom guess to be within +/- Search Window pixels
        SEARCH_WINDOW = 80 # px
        
        roi_x1 = max(0, int(guess_u - SEARCH_WINDOW))
        roi_y1 = max(0, int(guess_v - SEARCH_WINDOW))
        roi_x2 = min(img_w, int(guess_u + SEARCH_WINDOW))
        roi_y2 = min(img_h, int(guess_v + SEARCH_WINDOW))

        # Ensure ROI is larger than template
        if (roi_x2 - roi_x1) < new_w or (roi_y2 - roi_y1) < new_h:
            return None

        roi_img = depth_img[roi_y1:roi_y2, roi_x1:roi_x2].copy()
        roi_img = np.nan_to_num(roi_img, nan=0.0)

        # 4. Match Template
        # TM_SQDIFF is usually good for depth (least difference in Z values)
        # But TM_CCOEFF_NORMED is better if the absolute depth values shift slightly
        res = cv2.matchTemplate(roi_img, scaled_template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        # Threshold to ensure we actually found the object (0.5 is arbitrary, tune as needed)
        if max_val < 0.4:
            return None

        # 5. Calculate Refined Global Coordinates
        # max_loc is relative to the ROI top-left
        # We need to find the new top-left of the detected box in the global image
        top_left_x = roi_x1 + max_loc[0]
        top_left_y = roi_y1 + max_loc[1]

        # 6. Recover the Grab Point
        # Scale the original offset
        cur_offset_u = self.grab_offset_u * scale
        cur_offset_v = self.grab_offset_v * scale
        
        refined_u = int(top_left_x + cur_offset_u)
        refined_v = int(top_left_y + cur_offset_v)
        
        return (refined_u, refined_v, (top_left_x, top_left_y, new_w, new_h))