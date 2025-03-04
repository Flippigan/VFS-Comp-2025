#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math
import rclpy.qos

class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera info and state
        self.camera_info = None
        self.current_state = None
        self.current_pose = None
        self.target_detected = False
        self.target_pose = None
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        # Create QoS profile with BEST_EFFORT reliability
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.get_logger().info('Creating pose subscription to /ap/pose/filtered')
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.local_pos_callback,
            qos)  # Use the QoS profile here
        self.get_logger().info('Pose subscription created')
            
        # Create publishers
        self.debug_image_pub = self.create_publisher(
            Image,
            '/circle_detector/debug_image',
            10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/circle_detector/target_pose',
            10)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10)
            
        # Initialize service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
            
        # Initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Control parameters
        self.CIRCLE_DIAMETER = 0.5  # meters
        self.TARGET_ALTITUDE = 5.0  # meters
        self.VELOCITY_P_GAIN = 0.5  # Proportional gain for velocity control
        self.THRESHOLD_DISTANCE = 0.1  # meters
        
        # Create control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        self.get_logger().info('Circle detector node initialized')
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def local_pos_callback(self, msg):
        self.get_logger().info('Received pose data: x={}, y={}, z={}'.format(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ))
        self.current_pose = msg
        
    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def control_loop(self):
        """Main control loop for drone movement"""
        if not self.target_detected or self.target_pose is None:
            # No target detected, hover in place
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return
            
        if self.current_pose is None:
            # No pose data yet, hover in place
            self.get_logger().warn('No pose_1 data received yet')
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
            return
            
        try:
            # Calculate error between current position and target
            dx = float(self.target_pose.pose.position.x)
            dy = float(self.target_pose.pose.position.y)
            dz = float(self.TARGET_ALTITUDE - self.current_pose.pose.position.z)
            
            # Calculate velocities using proportional control
            vx = float(self.VELOCITY_P_GAIN * dx)
            vy = float(self.VELOCITY_P_GAIN * dy)
            vz = float(self.VELOCITY_P_GAIN * dz)
            
            # Limit maximum velocity
            max_vel = 1.0  # m/s
            scale = float(min(max_vel / max(abs(vx), abs(vy), abs(vz), 0.1), 1.0))
            vx = float(vx * scale)
            vy = float(vy * scale)
            vz = float(vz * scale)
            
            # Calculate yaw to face target
            yaw = float(math.atan2(dy, dx))
            
            # Publish velocity command
            self.publish_velocity(vx, vy, vz, yaw)
            
            # Check if we're close enough to drop payload
            distance = float(math.sqrt(dx*dx + dy*dy))
            if distance < self.THRESHOLD_DISTANCE:
                self.get_logger().info('Target reached! Ready for payload drop')
                # TODO: Add payload drop command
                
        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')
    
    def publish_velocity(self, vx, vy, vz, yaw):
        """Publish velocity command to MAVROS"""
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = 'base_link'
        
        # Linear velocity
        cmd_vel.twist.linear.x = float(vx)
        cmd_vel.twist.linear.y = float(vy)
        cmd_vel.twist.linear.z = float(vz)
        
        # Angular velocity (yaw)
        cmd_vel.twist.angular.z = float(yaw)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
    def image_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn('No camera info received yet')
            return
            
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # Detect circles using Hough Circle Transform
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=50,
                param1=50,
                param2=30,
                minRadius=20,
                maxRadius=100
            )
            
            debug_image = cv_image.copy()
            self.target_detected = False
            
            if circles is not None:
                circles = np.uint16(np.around(circles))
                
                # Get the largest circle (assuming it's the target)
                largest_circle = max(circles[0, :], key=lambda x: x[2])
                x, y, r = largest_circle
                
                # Draw the circle
                cv2.circle(debug_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(debug_image, (x, y), 2, (0, 0, 255), 3)
                
                # Calculate 3D position
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]
                
                # Create pose message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                
                # X and Y in camera frame
                x_normalized = (x - cx) / fx
                y_normalized = (y - cy) / fy
                
                # Estimate depth using circle size
                depth = (self.CIRCLE_DIAMETER * fx) / (2 * r)
                
                # 3D position in camera frame
                pose_msg.pose.position.x = x_normalized * depth
                pose_msg.pose.position.y = y_normalized * depth
                pose_msg.pose.position.z = depth
                
                # Update target information
                self.target_detected = True
                self.target_pose = pose_msg
                
                # Publish the pose
                self.target_pose_pub.publish(pose_msg)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
