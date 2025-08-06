#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class FlowEstimator(Node):
    def __init__(self):
        super().__init__('flow_estimator')
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('scene_depth', 1.0)
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('manual_fx', 800.0)  # Default focal length
        self.declare_parameter('manual_fy', 800.0)  # Default focal length
        
        self.scene_depth = self.get_parameter('scene_depth').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.manual_fx = self.get_parameter('manual_fx').value
        self.manual_fy = self.get_parameter('manual_fy').value
        
        # IMX708 specifications
        self.pixel_size = 1.12e-6  # meters per pixel
        self.sensor_width = 4608    # pixels
        self.sensor_height = 2592   # pixels
        
        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        
        # Publisher for velocity
        self.pub_twist = self.create_publisher(Twist, '/flow/velocity', 10)
        
        # State variables
        self.prev_gray = None
        self.prev_time = None
        self.fx = None
        self.fy = None
        self.calibration_received = False
        
        # Flow parameters
        self.flow_params = dict(
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0
        )
        
        self.get_logger().info(f'Flow estimator initialized with scene_depth={self.scene_depth}m')

    def camera_info_cb(self, msg: CameraInfo):
        if not self.calibration_received:
            # Extract focal lengths from camera matrix K
            # K = [fx  0 cx]
            #     [ 0 fy cy]
            #     [ 0  0  1]
            fx_from_msg = msg.k[0]
            fy_from_msg = msg.k[4]
            
            # Check if calibration data is valid
            if fx_from_msg > 0 and fy_from_msg > 0:
                if self.use_calibration:
                    self.fx = fx_from_msg
                    self.fy = fy_from_msg
                    self.get_logger().info(f'Using camera calibration: fx={self.fx:.1f}, fy={self.fy:.1f}')
                else:
                    self.fx = self.manual_fx
                    self.fy = self.manual_fy
                    self.get_logger().info(f'Using manual focal lengths: fx={self.fx:.1f}, fy={self.fy:.1f}')
            else:
                # Fallback to manual or estimated values
                self.fx = self.manual_fx
                self.fy = self.manual_fy
                self.get_logger().warn(f'Invalid calibration data, using manual: fx={self.fx:.1f}, fy={self.fy:.1f}')
            
            self.calibration_received = True

    def estimate_focal_length(self, image_width, image_height):
        """Estimate focal length based on typical camera FOV assumptions"""
        # Assume 60-degree horizontal FOV (common for many cameras)
        fov_h_rad = np.deg2rad(60)
        fx_estimated = image_width / (2 * np.tan(fov_h_rad / 2))
        
        # Maintain aspect ratio
        fy_estimated = fx_estimated * (image_height / image_width)
        
        return fx_estimated, fy_estimated

    def image_cb(self, msg: Image):
        if self.fx is None or self.fy is None:
            # If we still don't have focal lengths, estimate them
            image_width = msg.width
            image_height = msg.height
            self.fx, self.fy = self.estimate_focal_length(image_width, image_height)
            self.get_logger().warn(f'Estimated focal lengths: fx={self.fx:.1f}, fy={self.fy:.1f} (based on {image_width}x{image_height})')
        
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        current_time = time.time()
        
        if self.prev_gray is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            
            if dt <= 0 or dt > 1.0:  # Skip if time step is invalid or too large
                self.prev_gray = gray
                self.prev_time = current_time
                return
            
            # Calculate optical flow
            try:
                flow = cv2.calcOpticalFlowFarneback(
                    self.prev_gray, gray, None, **self.flow_params
                )
                
                # Calculate mean flow (excluding outliers)
                flow_magnitude = np.sqrt(flow[...,0]**2 + flow[...,1]**2)
                
                # Remove outliers (flows > 3 standard deviations from mean)
                mean_mag = np.mean(flow_magnitude)
                std_mag = np.std(flow_magnitude)
                mask = flow_magnitude < (mean_mag + 3 * std_mag)
                
                if np.sum(mask) > 0:
                    dx = float(np.mean(flow[...,0][mask]))
                    dy = float(np.mean(flow[...,1][mask]))
                else:
                    dx = float(np.mean(flow[...,0]))
                    dy = float(np.mean(flow[...,1]))
                
                # Sanity check on flow values
                if abs(dx) > 1000 or abs(dy) > 1000:
                    self.get_logger().warn(f'Large flow detected: dx={dx:.2f}, dy={dy:.2f}, skipping')
                    self.prev_gray = gray
                    self.prev_time = current_time
                    return
                
                # Calculate velocity using depth-scaled approach
                Z = self.scene_depth
                
                # Convert pixel flow to angular flow, then to linear velocity
                vx = (dx / self.fx) * Z / dt  # meters per second
                vy = (dy / self.fy) * Z / dt  # meters per second
                
                # Alternative method using pixel size (commented out)
                # vx_pixel = (dx * self.pixel_size) / dt
                # vy_pixel = (dy * self.pixel_size) / dt
                
                # Check for reasonable velocity values
                max_reasonable_velocity = 50.0  # m/s
                if abs(vx) > max_reasonable_velocity or abs(vy) > max_reasonable_velocity:
                    self.get_logger().warn(f'Unreasonable velocity: vx={vx:.2f}, vy={vy:.2f}, skipping')
                    self.prev_gray = gray
                    self.prev_time = current_time
                    return
                
                # Publish velocity
                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                self.pub_twist.publish(twist)
                
                # Log with more detail
                self.get_logger().info(
                    f'Flow: dx={dx:.1f}px, dy={dy:.1f}px, dt={dt*1000:.1f}ms -> '
                    f'v_x={vx:.3f}m/s, v_y={vy:.3f}m/s'
                )
                
            except Exception as e:
                self.get_logger().error(f'Error in optical flow calculation: {e}')
        
        # Update previous frame and time
        self.prev_gray = gray
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = FlowEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()