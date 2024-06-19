import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

parameters = [
    {
        "name": "model",
        "default": "mega3",
        "description": "model of rover",
        "choices": "'dwarfrover', 'megarover3', 'mega3'",
    },
    {
        "name": "launch_rviz",
        "default": "false",
        "description": "launch rviz",
        "choices": "'true', 'false'",
    },
]


def launch_setup(context, *args, **kwarg):
    log_message = ""
    model_str = LaunchConfiguration("model").perform(context)

    xacro_file_path = os.path.join(
        get_package_share_directory("description"),
        "urdf",
        model_str,
        f"{model_str}.xacro",
    )
    urdf_file_path = os.path.join(
        get_package_share_directory("description"),
        "urdf",
        model_str,
        f"{model_str}.urdf",
    )

    if os.path.exists(xacro_file_path):
        robot_description_path = xacro_file_path
        robot_description = ParameterValue(
            Command(["xacro ", str(robot_description_path)]), value_type=str
        )
        log_message = f"{GREEN}xacro file found: {robot_description_path}{RESET}"
    elif os.path.exists(urdf_file_path):
        robot_description_path = urdf_file_path
        with open(robot_description_path, "r") as file:
            robot_description = file.read()
        robot_description = ParameterValue(robot_description, value_type=str)
        log_message = f"{GREEN}urdf file found: {robot_description_path}{RESET}"
    else:
        raise FileNotFoundError(
            f"{RED}Neither xacro nor urdf file found for {model_str}{RESET}"
        )

    rviz_config_path = os.path.join(
        get_package_share_directory("description"),
        "rviz",
        f"{model_str}.rviz",
    )
    if not os.path.exists(rviz_config_path):
        raise FileNotFoundError(f"{RED}rviz file not found for {model_str}{RESET}")

    if LaunchConfiguration("launch_rviz").perform(context) == "true":
        log_message += f"\n{GREEN}Launching rviz{RESET}"

    log_message += f"\n{GREEN}Launching description(tf_static) node{RESET}"

    return [
        LogInfo(msg=log_message),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher_node",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            condition=LaunchConfigurationEquals("launch_rviz", "true"),
        ),
    ]


def add_launch_arg(param):
    return DeclareLaunchArgument(
        param["name"],
        default_value=param["default"],
        description=param["description"],
        choices=param["choices"],
    )


def set_parameter(param):
    return (param["name"], LaunchConfiguration(param["name"]))


def generate_launch_description():
    return LaunchDescription(
        [add_launch_arg(param) for param in parameters]
        + [
            OpaqueFunction(
                function=launch_setup,
            )
        ]
    )
