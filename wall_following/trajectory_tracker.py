import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Range
import math
import json
import os
from datetime import datetime


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # Get the namespace to construct the correct topic names
        namespace = self.get_namespace()
        if namespace == '/':
            # If no namespace, use global topics
            cmd_topic = '/cmd_vel'
            path_topic = '/robot_path'
            pose_topic = '/robot_pose'
        else:
            # If in namespace, use namespaced topics
            cmd_topic = f'{namespace}/cmd_vel'
            path_topic = f'{namespace}/robot_path'
            pose_topic = f'{namespace}/robot_pose'

        # Publishers
        self.__path_publisher = self.create_publisher(Path, path_topic, 10)
        self.__pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)

        # Subscriber to cmd_vel
        self.create_subscription(Twist, cmd_topic, self.__cmd_vel_callback, 1)

        # Robot state
        self.__x = 0.0  # Current x position
        self.__y = 0.0  # Current y position
        self.__theta = 0.0  # Current orientation
        self.__last_time = None

        # Path storage
        self.__path = Path()
        self.__path.header.frame_id = 'map'
        self.__trajectory_data = []  # For logging

        # Robot parameters (from my_robot_driver.py)
        self.__wheel_radius = 0.025
        self.__half_distance_between_wheels = 0.045

        # Timer for publishing path
        self.__path_timer = self.create_timer(0.1, self.__publish_path)  # 10 Hz

        # Logging setup
        self.__log_file = f'/tmp/robot_trajectory_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        self.__log_timer = self.create_timer(1.0, self.__log_trajectory)  # Log every second

        self.get_logger().info(f'Trajectory tracker initialized for namespace: {namespace}')
        self.get_logger().info(f'Publishing path to: {path_topic}')
        self.get_logger().info(f'Logging to: {self.__log_file}')

    def __cmd_vel_callback(self, twist):
        """Update robot position based on velocity commands"""
        current_time = self.get_clock().now()
        
        # Initialize last_time on first call
        if self.__last_time is None:
            self.__last_time = current_time
            return
            
        dt = (current_time - self.__last_time).nanoseconds / 1e9  # Convert to seconds
        
        if dt > 0.0:  # Avoid division by zero
            # Extract velocities
            linear_vel = twist.linear.x
            angular_vel = twist.angular.z

            # Update position using simple integration
            # This is a basic dead reckoning approach
            self.__x += linear_vel * math.cos(self.__theta) * dt
            self.__y += linear_vel * math.sin(self.__theta) * dt
            self.__theta += angular_vel * dt

            # Normalize angle to [-pi, pi]
            self.__theta = math.atan2(math.sin(self.__theta), math.cos(self.__theta))

            # Update last time
            self.__last_time = current_time

            # Add pose to path
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = self.__x
            pose_stamped.pose.position.y = self.__y
            pose_stamped.pose.position.z = 0.0
            
            # Convert theta to quaternion
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(self.__theta / 2.0)
            pose_stamped.pose.orientation.w = math.cos(self.__theta / 2.0)

            # Add to path
            self.__path.poses.append(pose_stamped)
            self.__path.header.stamp = current_time

            # Keep only last 1000 poses to avoid memory issues
            if len(self.__path.poses) > 1000:
                self.__path.poses.pop(0)

            # Publish current pose
            self.__pose_publisher.publish(pose_stamped)

    def __publish_path(self):
        """Publish the current path for visualization"""
        if len(self.__path.poses) > 0:
            self.__path_publisher.publish(self.__path)

    def __log_trajectory(self):
        """Log trajectory data to file"""
        if len(self.__path.poses) > 0:
            trajectory_point = {
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'x': self.__x,
                'y': self.__y,
                'theta': self.__theta,
                'pose_count': len(self.__path.poses)
            }
            self.__trajectory_data.append(trajectory_point)

            # Write to file periodically
            if len(self.__trajectory_data) % 10 == 0:  # Every 10 seconds
                try:
                    with open(self.__log_file, 'w') as f:
                        json.dump(self.__trajectory_data, f, indent=2)
                except Exception as e:
                    self.get_logger().warn(f'Failed to write trajectory log: {e}')

    def get_current_pose(self):
        """Get current robot pose"""
        return (self.__x, self.__y, self.__theta)

    def get_path_length(self):
        """Calculate total path length"""
        if len(self.__path.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(self.__path.poses)):
            prev_pose = self.__path.poses[i-1]
            curr_pose = self.__path.poses[i]
            dx = curr_pose.pose.position.x - prev_pose.pose.position.x
            dy = curr_pose.pose.position.y - prev_pose.pose.position.y
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length


def main(args=None):
    rclpy.init(args=args)
    tracker = TrajectoryTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        # Save final trajectory data
        try:
            with open(tracker._TrajectoryTracker__log_file, 'w') as f:
                json.dump(tracker._TrajectoryTracker__trajectory_data, f, indent=2)
            tracker.get_logger().info(f'Final trajectory saved to: {tracker._TrajectoryTracker__log_file}')
            tracker.get_logger().info(f'Total path length: {tracker.get_path_length():.2f} meters')
        except Exception as e:
            tracker.get_logger().warn(f'Failed to save final trajectory: {e}')
        
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
