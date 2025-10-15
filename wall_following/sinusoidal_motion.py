import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math


MAX_RANGE = 0.15


class SinusoidalMotion(Node):
    def __init__(self):
        super().__init__('sinusoidal_motion')

        # Get the namespace to construct the correct topic names
        namespace = self.get_namespace()
        if namespace == '/':
            # If no namespace, use global topics
            left_topic = '/ds0'
            right_topic = '/ds1'
            cmd_topic = '/cmd_vel'
        else:
            # If in namespace, use namespaced topics
            left_topic = f'{namespace}/ds0'
            right_topic = f'{namespace}/ds1'
            cmd_topic = f'{namespace}/cmd_vel'

        self.__publisher = self.create_publisher(Twist, cmd_topic, 1)

        self.create_subscription(Range, left_topic, self.__left_sensor_callback, 1)
        self.create_subscription(Range, right_topic, self.__right_sensor_callback, 1)
        
        # Debug output
        self.get_logger().info(f'Sinusoidal motion subscribing to: {left_topic} and {right_topic}')
        self.get_logger().info(f'Sinusoidal motion publishing to: {cmd_topic}')
        
        # Initialize sensor values
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE
        
        # Sinusoidal motion parameters
        self.__time_start = self.get_clock().now()
        self.__base_speed = 0.05 # Base forward speed
        self.__speed_amplitude = 0.05  # Amplitude of speed variation
        self.__frequency = 0.5  # Frequency of oscillation (Hz)
        
        # Obstacle avoidance parameters
        self.__avoidance_angular_speed = -2.0
        self.__avoidance_threshold = 0.9 * MAX_RANGE
        
        # Timer for periodic logging (every 3 seconds)
        self.__log_timer = self.create_timer(3.0, self.__log_status)

    def __log_status(self):
        """Periodic logging of sensor values and current speed"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.__time_start).nanoseconds / 1e9
        current_speed = self.__base_speed + self.__speed_amplitude * math.sin(2 * math.pi * self.__frequency * elapsed_time)
        self.get_logger().info(f'Sensors - Left: {self.__left_sensor_value:.3f}, Right: {self.__right_sensor_value:.3f}, Speed: {current_speed:.3f}')

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range
        #self.get_logger().info(f'Left sensor: {self.__left_sensor_value}')

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
        #self.get_logger().info(f'Right sensor: {self.__right_sensor_value}')

        command_message = Twist()

        # Calculate sinusoidal speed (always positive, no backwards)
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.__time_start).nanoseconds / 1e9  # Convert to seconds
        
        # Sinusoidal speed: base_speed + amplitude * sin(frequency * time)
        # Ensure speed is always positive (no backwards movement)
        sinusoidal_speed = self.__base_speed + self.__speed_amplitude * math.sin(2 * math.pi * self.__frequency * elapsed_time)
        command_message.linear.x = max(0.0, sinusoidal_speed)  # Ensure non-negative speed

        # Obstacle avoidance logic
        if self.__left_sensor_value < self.__avoidance_threshold or self.__right_sensor_value < self.__avoidance_threshold:
            command_message.angular.z = self.__avoidance_angular_speed
            self.get_logger().info('Avoiding obstacle!')
        else:
            command_message.angular.z = 0.0  # No turning when no obstacles

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    motion = SinusoidalMotion()
    rclpy.spin(motion)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
