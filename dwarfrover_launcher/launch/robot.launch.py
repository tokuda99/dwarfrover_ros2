import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

parameters = [
    {
        "name": "model",
        "default": "mega3",
        "description": "model of rover",
        "choices": "'dwarfrover', 'zestyrover', 'mega3'",
    },
    {
        "name": "controller",
        "default": "joycon",
        "description": "controller",
        "choices": "'joycon', 'keyboard', 'mouse'",
    },
    {
        "name": "launch_rviz",
        "default": "false",
        "description": "launch rviz",
        "choices": "'true', 'false'",
    },
]


def launch_setup(context, *args, **kwarg):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("description"),
                    "launch",
                    "description.launch.py",
                )
            ),
            launch_arguments={
                "model": LaunchConfiguration("model").perform(context),
                "launch_rviz": LaunchConfiguration("launch_rviz").perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("odometry"),
                    "launch",
                    "odom.launch.py",
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("monitor"),
                    "launch",
                    "monitor.launch.py",
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("control"),
                    "launch",
                    "joycon_teleop.launch.py",
                )
            ),
            condition=LaunchConfigurationEquals("controller", "joycon"),
            launch_arguments={"cmd_vel": "rover_twist"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("control"),
                    "launch",
                    "keyboard_teleop.launch.py",
                )
            ),
            condition=LaunchConfigurationEquals("controller", "keyboard"),
            launch_arguments={"cmd_vel": "rover_twist"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("control"),
                    "launch",
                    "mouse_teleop.launch.py",
                )
            ),
            condition=LaunchConfigurationEquals("controller", "mouse"),
            launch_arguments={"mouse_vel": "rover_twist"}.items(),
        ),
    ]


def add_launch_arg(param):
    return DeclareLaunchArgument(
        param["name"],
        default_value=param["default"],
        description=param["description"],
        choices=param["choices"],
    )


def generate_launch_description():
    return LaunchDescription(
        [add_launch_arg(param) for param in parameters]
        + [
            OpaqueFunction(
                function=launch_setup,
            )
        ]
    )
