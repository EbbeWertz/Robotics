import numpy as np
import math

def invert_transform(T):
    """Inverts a 4x4 homogeneous transformation matrix."""
    R_inv = T[0:3, 0:3].T
    t_inv = -R_inv @ T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    return T_inv

class OdomObjectTracker:
    """
    Handles the 3D math for tracking a static point in the world 
    using Odometry and Camera Intrinsics.
    """
    def __init__(self, camera_params):
        self.width = camera_params['width']
        self.height = camera_params['height']
        # Pinhole Camera Intrinsics
        self.fx = self.width / (2 * math.tan(camera_params['FOV'] / 2))
        self.fy = self.fx
        self.cx = self.width / 2
        self.cy = self.height / 2
        
        # State: The fixed point in World Frame (4x1 vector [x,y,z,1])
        self.target_point_world = None
        self.is_initialized = False

    def lock_target(self, u, v, z, odom_T_world_base, tf_T_base_cam):
        """
        Initializes the tracker by back-projecting a pixel (u,v,z) 
        from the current robot pose into a fixed World coordinate.
        """
        if z <= 0 or not np.isfinite(z):
            return False

        # 1. Project Pixel -> Camera Frame
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        P_cam = np.array([
            (u - self.cx) * z / self.fx,
            (v - self.cy) * z / self.fy,
            z,
            1.0
        ])

        # 2. Transform Camera Frame -> World Frame
        # T_world_cam = T_world_base * T_base_cam
        # P_world = T_world_cam * P_cam
        T_world_cam = odom_T_world_base @ tf_T_base_cam
        self.target_point_world = T_world_cam @ P_cam
        self.is_initialized = True
        return True

    def get_projected_pixel(self, current_odom_T, current_base_to_cam_T):
        """
        Projects the stored World point back onto the camera image 
        based on the robot's current pose.
        """
        if not self.is_initialized:
            return None

        # 1. World -> Base -> Camera (Current)
        # P_cam = inv(T_world_cam) * P_world
        #       = inv(T_odom * T_base_cam) * P_world
        T_world_cam_now = current_odom_T @ current_base_to_cam_T
        P_cam_now = invert_transform(T_world_cam_now) @ self.target_point_world

        Xc, Yc, Zc, _ = P_cam_now

        # Avoid division by zero if object is behind camera or too close
        if Zc <= 0.01: 
            return None

        # 2. Camera Frame -> Pixel
        u_pred = int(self.fx * Xc / Zc + self.cx)
        v_pred = int(self.fy * Yc / Zc + self.cy)
        
        return (u_pred, v_pred)
