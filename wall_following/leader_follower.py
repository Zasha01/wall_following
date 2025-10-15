import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import math


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
            cmd_topic = '/cmd_vel'
        else:
            # If in namespace, use namespaced topics
            left_topic = f'{namespace}/ds0'
            right_topic = f'{namespace}/ds1'
            back_topic = f'{namespace}/back_sensor'
            cmd_topic = f'{namespace}/cmd_vel'

        self.__publisher = self.create_publisher(Twist, cmd_topic, 1)

        self.create_subscription(Range, left_topic, self.__left_sensor_callback, 1)
        self.create_subscription(Range, right_topic, self.__right_sensor_callback, 1)
        self.create_subscription(Range, back_topic, self.__back_sensor_callback, 1)
        
        # Debug output
        self.get_logger().info(f'Leader-follower subscribing to: {left_topic}, {right_topic}, {back_topic}')
        self.get_logger().info(f'Leader-follower publishing to: {cmd_topic}')
        
        # Initialize sensor values
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE
        self.__back_sensor_value = BACK_SENSOR_MAX_RANGE
        
        # Leader-follower parameters
        self.__base_speed = 0.12  # Higher base speed for leader
        self.__wait_speed = 0.01 # Slower speed when waiting for follower
        self.__follow_distance_threshold = 0.3  # Distance threshold to wait for follower
        self.__avoidance_angular_speed = -2.0
        self.__avoidance_threshold = 0.9 * MAX_RANGE
        
        # State tracking
        self.__is_waiting_for_follower = False

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range
        self.get_logger().info(f'Left sensor: {self.__left_sensor_value}')

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
        self.get_logger().info(f'Right sensor: {self.__right_sensor_value}')

    def __back_sensor_callback(self, message):
        self.__back_sensor_value = message.range
        self.get_logger().info(f'Back sensor: {self.__back_sensor_value}')

        command_message = Twist()

        # Determine if we should wait for the follower
        # If back sensor detects something close (likely Robot2), check distance
        if self.__back_sensor_value < BACK_SENSOR_MAX_RANGE:
            # Something detected behind us
            if self.__back_sensor_value > self.__follow_distance_threshold:
                # Follower is too far away, slow down
                command_message.linear.x = self.__wait_speed
                self.__is_waiting_for_follower = True
                self.get_logger().info(f'Waiting for follower - distance: {self.__back_sensor_value}')
            else:
                # Follower is close enough, normal speed
                command_message.linear.x = self.__base_speed
                self.__is_waiting_for_follower = False
                self.get_logger().info(f'Follower close enough - distance: {self.__back_sensor_value}')
        else:
            # No follower detected, normal speed
            command_message.linear.x = self.__base_speed
            self.__is_waiting_for_follower = False
            self.get_logger().info('No follower detected, normal speed')

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
