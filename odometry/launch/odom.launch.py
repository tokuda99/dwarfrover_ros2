from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, LogInfo

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


def launch_setup(context, *args, **kwarg):
    return [
        LogInfo(msg=f"{GREEN}Launching odometry node{RESET}"),
        Node(package="odometry", executable="pub_odom", name="pub_odom"),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(
                function=launch_setup,
            )
        ]
    )
