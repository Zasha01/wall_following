import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Twist
import math
import numpy as np
from cv_bridge import CvBridge
import cv2


MAX_RANGE = 0.15
BACK_SENSOR_MAX_RANGE = 0.5


class LeaderFollower(Node):
    def __init__(self):
        super().__init__('leader_follower')

        # Get the namespace to construct the correct topic names
        namespace = self.get_namespace()
        if namespace == '/':
            # If no namespace, use global topics
            left_topic = '/ds0'
            right_topic = '/ds1'
            back_topic = '/back_sensor'
            color_topic = '/color_sensor/image_color'
            cmd_topic = '/cmd_vel'
        else:
            # If in namespace, use namespaced topics
            left_topic = f'{namespace}/ds0'
            right_topic = f'{namespace}/ds1'
            back_topic = f'{namespace}/back_sensor'
            color_topic = f'{namespace}/color_sensor/image_color'
            cmd_topic = f'{namespace}/cmd_vel'

        self.__publisher = self.create_publisher(Twist, cmd_topic, 1)

        self.create_subscription(Range, left_topic, self.__left_sensor_callback, 1)
        self.create_subscription(Range, right_topic, self.__right_sensor_callback, 1)
        self.create_subscription(Range, back_topic, self.__back_sensor_callback, 1)
        self.create_subscription(Image, color_topic, self.__color_callback, 1)
        
        # Debug: Test if camera topic exists
        self.get_logger().info(f'Testing camera topic: {color_topic}')
        
        # Debug output
        self.get_logger().info(f'Leader-follower subscribing to: {left_topic}, {right_topic}, {back_topic}, {color_topic}')
        self.get_logger().info(f'Leader-follower publishing to: {cmd_topic}')
        
        # Initialize sensor values
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE
        self.__back_sensor_value = BACK_SENSOR_MAX_RANGE
        self.__green_detected = False
        
        # Initialize CV bridge for image processing
        self.__cv_bridge = CvBridge()
        
        # Debug counters
        self.__image_count = 0
        self.__debug_interval = 5  # Save every 5th image for more frequent debugging
        
        # Leader-follower parameters
        self.__base_speed = 0.12  # Higher base speed for leader
        self.__wait_speed = 0.01 # Slower speed when waiting for follower
        self.__follow_distance_threshold = 0.4  # Distance threshold to wait for follower
        self.__avoidance_angular_speed = -2.0
        self.__avoidance_threshold = 0.9 * MAX_RANGE
        
        # State tracking
        self.__is_waiting_for_follower = False
        
        # Timer for periodic logging (every 2 seconds)
        self.__log_timer = self.create_timer(2.0, self.__log_status)

    def __log_status(self):
        """Periodic logging of sensor values and status"""
        self.get_logger().info(f'Sensors - Left: {self.__left_sensor_value:.3f}, Right: {self.__right_sensor_value:.3f}, Back: {self.__back_sensor_value:.3f}')
        self.get_logger().info(f'Color detection - Green detected: {self.__green_detected}')
        if self.__is_waiting_for_follower:
            if self.__green_detected:
                self.get_logger().info(f'Status: WAITING for Robot2 (green detected, distance: {self.__back_sensor_value:.3f})')
            else:
                self.get_logger().info(f'Status: WAITING for Robot2 (no green detected, distance: {self.__back_sensor_value:.3f})')
        else:
            self.get_logger().info(f'Status: NORMAL speed (Robot2 close, distance: {self.__back_sensor_value:.3f})')

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range
        #self.get_logger().info(f'Left sensor: {self.__left_sensor_value}')

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
        #self.get_logger().info(f'Right sensor: {self.__right_sensor_value}')

    def __color_callback(self, message):
        """Process color image to detect green Robot2"""
        #self.get_logger().info(f'Camera callback triggered! Image count: {self.__image_count}')
        self.__image_count += 1
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.__cv_bridge.imgmsg_to_cv2(message, "bgr8")
            
            # Debug: Save image occasionally
            if self.__image_count % self.__debug_interval == 0:
                #cv2.imwrite(f'/tmp/camera_image_{self.__image_count}.png', cv_image)
                #self.get_logger().info(f'Saved debug image: camera_image_{self.__image_count}.png')
                
                # Analyze RGB values
                if cv_image.size > 0:
                    # Get average RGB values
                    avg_color = cv_image.mean(axis=(0, 1))
                    self.get_logger().info(f'Image {self.__image_count} - Average RGB: R={avg_color[2]:.1f}, G={avg_color[1]:.1f}, B={avg_color[0]:.1f}')
                    
                    # Check if image is mostly black (no object detected)
                    if avg_color.mean() < 10:
                        self.get_logger().warn(f'Image {self.__image_count} appears to be mostly black - camera might not be working')
            
            # Convert to HSV for better color detection
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Simple green detection - standard HSV range for green colors
            # Green hue is around 60 degrees in OpenCV HSV
            # Use a broad but standard range for green detection
            lower_green = np.array([40, 40, 40])   # Standard green lower bound
            upper_green = np.array([80, 255, 255]) # Standard green upper bound
            
            # Create mask for green color
            green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
            
            # Count green pixels
            green_pixels = cv2.countNonZero(green_mask)
            total_pixels = cv_image.shape[0] * cv_image.shape[1]
            green_ratio = green_pixels / total_pixels
            
            # With higher resolution, we can use a lower threshold
            # If more than 0.05% of image is green, consider Robot2 detected
            self.__green_detected = green_ratio > 0.006
            
            # Debug: log the green ratio occasionally
            if self.__image_count % self.__debug_interval == 0:
                # Get average RGB values for detected green pixels
                if green_pixels > 0:
                    green_pixel_coords = np.where(green_mask > 0)
                    if len(green_pixel_coords[0]) > 0:
                        green_rgb_values = cv_image[green_pixel_coords[0], green_pixel_coords[1]]
                        avg_green_rgb = green_rgb_values.mean(axis=0)
                        #self.get_logger().info(f'Detected green RGB: R={avg_green_rgb[2]:.1f}, G={avg_green_rgb[1]:.1f}, B={avg_green_rgb[0]:.1f}')
                        
                        # Convert detected RGB to HSV for comparison
                        detected_hsv = cv2.cvtColor(np.uint8([[avg_green_rgb]]), cv2.COLOR_BGR2HSV)[0][0]
                        #self.get_logger().info(f'Detected HSV: H={detected_hsv[0]}, S={detected_hsv[1]}, V={detected_hsv[2]}')
                        
                        # Check if this looks like sky (blue-ish)
                        # Sky colors: rgba(115,133,168), rgba(87,106,152), rgba(55,68,105)
                        # Blue colors have: B > G and B > R
                        #if (avg_green_rgb[0] > avg_green_rgb[1] and avg_green_rgb[0] > avg_green_rgb[2] and
                        #    avg_green_rgb[0] > 100):  # High blue component
                        #    self.get_logger().warn(f'Detected color looks like sky/blue! RGB: R={avg_green_rgb[2]:.1f}, G={avg_green_rgb[1]:.1f}, B={avg_green_rgb[0]:.1f}')
                else:
                    # No green pixels detected - let's see what colors are in the image
                    avg_color = cv_image.mean(axis=(0, 1))
                    avg_hsv = cv2.cvtColor(np.uint8([[avg_color]]), cv2.COLOR_BGR2HSV)[0][0]
                    #self.get_logger().info(f'No green detected. Average RGB: R={avg_color[2]:.1f}, G={avg_color[1]:.1f}, B={avg_color[0]:.1f}')
                    #self.get_logger().info(f'Average HSV: H={avg_hsv[0]}, S={avg_hsv[1]}, V={avg_hsv[2]}')
                
                #self.get_logger().info(f'Image {self.__image_count} - Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}, Green pixels: {green_pixels}/{total_pixels}, ratio: {green_ratio:.4f}, detected: {self.__green_detected}')
                
                # Save the green mask for debugging
                #cv2.imwrite(f'/tmp/green_mask_{self.__image_count}.png', green_mask)
                #self.get_logger().info(f'Saved green mask: green_mask_{self.__image_count}.png')
                
                # Also save the original image for debugging
                #cv2.imwrite(f'/tmp/camera_image_{self.__image_count}.png', cv_image)
                #self.get_logger().info(f'Saved debug image: camera_image_{self.__image_count}.png')
            
        except Exception as e:
            self.get_logger().warn(f'Color processing error: {e}')
            self.__green_detected = False

    def __back_sensor_callback(self, message):
        self.__back_sensor_value = message.range

        command_message = Twist()
        previous_waiting_state = self.__is_waiting_for_follower

        # Determine if we should wait for the follower
        # Only wait if BOTH conditions are met:
        # 1. Something detected behind us (distance sensor)
        # 2. Green color detected (Robot2)
        if (self.__back_sensor_value < BACK_SENSOR_MAX_RANGE and 
            self.__green_detected):
            # Robot2 detected behind us
            if self.__back_sensor_value > self.__follow_distance_threshold:
                # Robot2 is too far away, slow down
                command_message.linear.x = self.__wait_speed
                self.__is_waiting_for_follower = True
                # Only log when state changes
                if not previous_waiting_state:
                    self.get_logger().info(f'Started waiting for Robot2 (green detected) - distance: {self.__back_sensor_value:.3f}')
            else:
                # Robot2 is close enough, normal speed
                command_message.linear.x = self.__base_speed
                self.__is_waiting_for_follower = False
                # Only log when state changes
                if previous_waiting_state:
                    self.get_logger().info(f'Robot2 caught up (green detected) - distance: {self.__back_sensor_value:.3f}')
        else:
            # No Robot2 detected (either no object or no green color), slow down to wait
            command_message.linear.x = self.__wait_speed
            self.__is_waiting_for_follower = True
            # Only log when state changes
            if not previous_waiting_state:
                if not self.__green_detected:
                    self.get_logger().info('Lost Robot2 (no green detected), slowing down to wait')
                else:
                    self.get_logger().info('Lost Robot2 (no object detected), slowing down to wait')

        # Obstacle avoidance logic (always active)
        if self.__left_sensor_value < self.__avoidance_threshold or self.__right_sensor_value < self.__avoidance_threshold:
            command_message.angular.z = self.__avoidance_angular_speed
            self.get_logger().info('Avoiding obstacle!')
        else:
            command_message.angular.z = 0.0  # No turning when no obstacles

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    leader = LeaderFollower()
    rclpy.spin(leader)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
