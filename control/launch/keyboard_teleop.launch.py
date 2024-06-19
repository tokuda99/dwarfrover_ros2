from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        remappings=[("/cmd_vel", "/rover_twist")],
    )

    return LaunchDescription([keyboard_teleop_node])
