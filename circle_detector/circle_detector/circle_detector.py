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
from std_msgs.msg import Int8
from std_msgs.msg import Float64

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
        
        # Flag to track if pose data has been received
        self.pose_data_confirmed = False
        self.camera_data_count = 0
        
        # Mode control state variables
        self.in_circle_approach_mode = False
        self.drop_completed = False
        self.mode_switch_requested = False
        self.last_mode_request_time = self.get_clock().now()
        self.MODE_REQUEST_TIMEOUT = 5.0  # seconds
        
        # MAVROS connection status
        self.mavros_connected = False
        self.last_state_warning_time = self.get_clock().now()
        self.state_warning_interval = 5.0  # seconds
        
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
        
        # Create timer to check MAVROS connection status
        self.mavros_check_timer = self.create_timer(1.0, self.check_mavros_connection)
        
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
        
        # Initialize drop system
        self.drops_remaining = 4
        self.last_drop_time = self.get_clock().now()
        self.DROP_COOLDOWN = 2.0  # seconds between drops
        
        # Create servo publisher
        self.servo_pub = self.create_publisher(
            Float64,
            '/iris/servo1/position',  # Adjust topic based on your Gazebo model
            10
        )
        
        # Create drops remaining publisher
        self.drops_pub = self.create_publisher(
            Int8,
            '/circle_detector/drops_remaining',
            10
        )
        
        self.get_logger().info('Circle detector node initialized')
        
    def check_mavros_connection(self):
        """Periodically check and report if MAVROS state is not being received"""
        current_time = self.get_clock().now()
        time_since_warning = (current_time - self.last_state_warning_time).nanoseconds / 1e9
        
        if self.current_state is None and time_since_warning > self.state_warning_interval:
            self.get_logger().warn('No MAVROS state received yet. Check if MAVROS is running and connected.')
            self.last_state_warning_time = current_time
            
        # Check if we've just connected to MAVROS
        if self.current_state is not None and not self.mavros_connected:
            self.mavros_connected = True
            self.get_logger().info('MAVROS connection established! Current flight mode: ' + 
                                  self.current_state.mode)

    def state_callback(self, msg):
        # Log when flight mode changes
        if self.current_state is None or self.current_state.mode != msg.mode:
            self.get_logger().info(f'Flight mode changed to: {msg.mode}')
            
            # Reset mode switch flag when mode actually changes
            if self.mode_switch_requested and (
                (self.in_circle_approach_mode and msg.mode == "GUIDED") or
                (self.drop_completed and msg.mode == "AUTO")):
                self.mode_switch_requested = False
                
        # Update state
        self.current_state = msg
        self.mavros_connected = True  # Mark as connected when we receive state
        
    def local_pos_callback(self, msg):
        # Only log once to confirm connection is working
        if not self.pose_data_confirmed:
            self.get_logger().info('Pose data connection confirmed: x={}, y={}, z={}'.format(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ))
            self.pose_data_confirmed = True
        
        self.current_pose = msg
        
    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def control_loop(self):
        """Main control loop for drone movement"""
        # Publish drops remaining periodically
        remaining_msg = Int8()
        remaining_msg.data = self.drops_remaining
        self.drops_pub.publish(remaining_msg)
        
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
            
            # Calculate distance to target
            distance = float(math.sqrt(dx*dx + dy*dy))
            
            # Check if we're close enough to stop and drop payload
            if distance < self.THRESHOLD_DISTANCE:
                # We're close enough - hover in place
                self.get_logger().info(f'Hovering over target at distance: {distance:.2f}m')
                
                # Stop the drone by setting zero velocity
                self.publish_velocity(0.0, 0.0, vz=0.1*dz, yaw=0.0)  # Only maintain altitude
                
                # Attempt to drop payload
                self.trigger_drop()
                return
            
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
                minRadius=30,
                maxRadius=100
            )
            
            debug_image = cv_image.copy()
            was_target_detected = self.target_detected
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
                
                # Log when circle is first detected
                if not was_target_detected:
                    self.get_logger().info(f'Circle detected! Position: x={pose_msg.pose.position.x:.2f}, y={pose_msg.pose.position.y:.2f}, z={pose_msg.pose.position.z:.2f}, radius={r}')
                    
                    # Switch to GUIDED mode when circle is first detected and we're not already in circle approach mode
                    if not self.in_circle_approach_mode and not self.drop_completed:
                        # Check if MAVROS is connected before attempting mode change
                        if self.mavros_connected and self.current_state is not None:
                            self.get_logger().info('Circle detected! Switching to GUIDED mode for approach')
                            if self.set_mode("GUIDED"):
                                self.in_circle_approach_mode = True
                                self.drop_completed = False
                        else:
                            self.get_logger().warn('Circle detected but MAVROS not connected. Cannot switch to GUIDED mode.')
                
                # Draw additional information on debug image
                text = f"X: {pose_msg.pose.position.x:.2f}m Y: {pose_msg.pose.position.y:.2f}m Z: {pose_msg.pose.position.z:.2f}m"
                cv2.putText(debug_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show mode on debug image
                if self.current_state:
                    mode_text = f"Mode: {self.current_state.mode}"
                    cv2.putText(debug_image, mode_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                else:
                    cv2.putText(debug_image, "Mode: MAVROS not connected", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                # Publish the pose
                self.target_pose_pub.publish(pose_msg)
            else:
                # Log when circle is lost
                if was_target_detected:
                    self.get_logger().info('Circle lost from view')
                
                # Draw status on debug image when no circle detected
                cv2.putText(debug_image, "No circle detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Always show drops remaining and mode on the debug image
            drops_text = f"Drops remaining: {self.drops_remaining}"
            cv2.putText(debug_image, drops_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            # Show MAVROS connection status
            if self.mavros_connected:
                if self.current_state:
                    mode_text = f"Mode: {self.current_state.mode}"
                    connection_status = "MAVROS: Connected"
                else:
                    mode_text = "Mode: Unknown"
                    connection_status = "MAVROS: Connected (no state)"
            else:
                mode_text = "Mode: Unknown"
                connection_status = "MAVROS: Not connected"
            
            cv2.putText(debug_image, mode_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
            cv2.putText(debug_image, connection_status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def trigger_drop(self):
        """Trigger payload drop if conditions are met"""
        if self.drops_remaining <= 0:
            self.get_logger().warn('No drops remaining!')
            
            # If we're in circle approach mode but have no drops left, switch back to AUTO
            if self.in_circle_approach_mode and not self.drop_completed:
                self.get_logger().info('No drops left, returning to AUTO mode')
                if self.set_mode("AUTO"):
                    self.in_circle_approach_mode = False
                    self.drop_completed = True
            
            return False
            
        current_time = self.get_clock().now()
        time_since_last_drop = (current_time - self.last_drop_time).nanoseconds / 1e9
        
        if time_since_last_drop < self.DROP_COOLDOWN:
            self.get_logger().info('Drop cooldown in progress')
            return False
            
        # Trigger servo movement
        servo_msg = Float64()
        servo_msg.data = 1.0  # Open position
        self.servo_pub.publish(servo_msg)
        
        # Reset servo after 0.5 seconds
        self.create_timer(0.5, lambda: self.reset_servo())
        
        # Update drop count
        self.drops_remaining -= 1
        self.last_drop_time = current_time
        
        # Publish remaining drops
        remaining_msg = Int8()
        remaining_msg.data = self.drops_remaining
        self.drops_pub.publish(remaining_msg)
        
        self.get_logger().info(f'Drop triggered! {self.drops_remaining} drops remaining')
        
        # If drop was successful, mark the drop as completed and
        # schedule a return to AUTO mode after a short delay to allow the drop to complete
        if self.in_circle_approach_mode:
            self.drop_completed = True
            self.create_timer(2.0, lambda: self.return_to_auto_mode())
        
        return True
    
    def reset_servo(self):
        """Reset servo to closed position"""
        servo_msg = Float64()
        servo_msg.data = 0.0  # Closed position
        self.servo_pub.publish(servo_msg)

    def set_mode(self, mode):
        """Request mode change to MAVROS"""
        if self.current_state is None:
            self.get_logger().warn('Cannot change mode: No state information available from MAVROS')
            self.get_logger().warn('Check if MAVROS is running and the drone is connected')
            return False
            
        if self.current_state.mode == mode:
            self.get_logger().info(f'Already in {mode} mode')
            return True
            
        # Only accept mode changes if connected
        if not self.current_state.connected:
            self.get_logger().warn(f'Cannot change to {mode} mode: Drone is not connected')
            return False
            
        # Don't spam mode requests
        current_time = self.get_clock().now()
        time_since_last_request = (current_time - self.last_mode_request_time).nanoseconds / 1e9
        
        if self.mode_switch_requested and time_since_last_request < self.MODE_REQUEST_TIMEOUT:
            return False
            
        # Create mode request
        req = SetMode.Request()
        req.custom_mode = mode
        
        # Send request
        self.get_logger().info(f'Requesting mode change to {mode}')
        self.set_mode_client.call_async(req).add_done_callback(
            lambda future: self.mode_change_callback(future, mode))
        
        # Update flags
        self.mode_switch_requested = True
        self.last_mode_request_time = current_time
        return True
        
    def mode_change_callback(self, future, requested_mode):
        """Handle mode change response"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode change to {requested_mode} accepted')
            else:
                self.get_logger().error(f'Mode change to {requested_mode} rejected')
                self.mode_switch_requested = False
        except Exception as e:
            self.get_logger().error(f'Mode change service call failed: {str(e)}')
            self.mode_switch_requested = False
    
    def return_to_auto_mode(self):
        """Switch back to AUTO mode if drop is completed"""
        if self.drop_completed and self.in_circle_approach_mode:
            self.get_logger().info('Drop completed. Returning to AUTO mode to resume mission')
            if self.set_mode("AUTO"):
                self.in_circle_approach_mode = False

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
