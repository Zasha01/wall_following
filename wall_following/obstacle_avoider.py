import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

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
        self.get_logger().info(f'Obstacle avoider subscribing to: {left_topic} and {right_topic}')
        self.get_logger().info(f'Obstacle avoider publishing to: {cmd_topic}')
        
        # Initialize sensor values
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range
        #self.get_logger().info(f'Left sensor: {self.__left_sensor_value}')

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
       #self.get_logger().info(f'Right sensor: {self.__right_sensor_value}')

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0
            self.get_logger().info('Turning!')

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()