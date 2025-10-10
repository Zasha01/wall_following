import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        # Get robot name from the robot object itself
        robot_name = self.__robot.getName()
        self.__node = rclpy.create_node(f'{robot_name}_driver')
        self.__node.get_logger().info(f'Robot driver initialized for robot: {robot_name}')
        self.__node.get_logger().info(f'All properties: {properties}')
        # Subscribe to the namespaced cmd_vel topic
        cmd_vel_topic = f'/{robot_name}/cmd_vel'
        self.__node.create_subscription(Twist, cmd_vel_topic, self.__cmd_vel_callback, 1)
        self.__node.get_logger().info(f'Robot driver subscribing to: {cmd_vel_topic}')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        self.__node.get_logger().info(f'Received cmd_vel: linear={twist.linear.x}, angular={twist.angular.z}')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)