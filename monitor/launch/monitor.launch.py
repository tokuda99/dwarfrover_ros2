from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch_ros.parameter_descriptions import ParameterValue

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(msg=f"{GREEN}Launching monitor node{RESET}"),
            Node(
                package="monitor",
                executable="monitor_node",
                name="monitor_node",
                output="screen",
                parameters=[
                    {"use_sensor_data_qos": ParameterValue(False, value_type=bool)}
                ],
            ),
        ]
    )
