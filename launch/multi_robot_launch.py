import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('wall_following')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    # Robot 1
    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot_description_path},
            {'robot_name': 'robot1'},
        ],
        respawn=True
    )

    robot1_obstacle_avoider = Node(
        package='wall_following',
        executable='obstacle_avoider',
        namespace='robot1',
        name='obstacle_avoider'
    )

    # Robot 2
    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot_description_path},
            {'robot_name': 'robot2'},
        ],
        respawn=True
    )

    robot2_sinusoidal_motion = Node(
        package='wall_following',
        executable='sinusoidal_motion',
        namespace='robot2',
        name='sinusoidal_motion'
    )

    return LaunchDescription([
        webots,
        robot1_driver,
        robot1_obstacle_avoider,
        robot2_driver,
        robot2_sinusoidal_motion,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
