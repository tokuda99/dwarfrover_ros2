from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"


parameters = [
    {
        "name": "dev",
        "default": "'/dev/input/js0'",
        "description": "device file",
        "type": "string",
    },
    {
        "name": "axis_linear",
        "default": 4,
        "description": "axis for linear",
        "type": "int",
    },
    {
        "name": "axis_angular",
        "default": 0,
        "description": "axis for angular",
        "type": "int",
    },
    {
        "name": "scale_linear",
        "default": 0.8,
        "description": "scale for linear",
        "type": "double",
    },
    {
        "name": "scale_angular",
        "default": 0.6,
        "description": "scale for angular",
        "type": "double",
    },
    {
        "name": "forward_button",
        "default": 1,
        "description": "forward button",
        "type": "int",
    },
    {
        "name": "backward_button",
        "default": 0,
        "description": "backward button",
        "type": "int",
    },
]


def launch_setup(context, *args, **kwarg):
    params = kwarg["params"]
    log_message = ""
    log_message = f'{GREEN}Launching joycon node{RESET}'

    return [
        LogInfo(msg=log_message),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),  
        Node(
            package="control",
            executable="joycon_node",
            name="joycon_node",
            output="screen",
            parameters=[params],
            remappings=[("/cmd_vel", "/rover_twist")],
        ),
    ]


def add_launch_arg(param):
    if "choices" not in param:
        return DeclareLaunchArgument(
            param["name"],
            default_value=str(param["default"]),
            description=param["description"],
        )
    return DeclareLaunchArgument(
        param["name"],
        default_value=str(param["default"]),
        description=param["description"],
        choices=param["choices"],
    )


def set_parameter(param):
    param_value = LaunchConfiguration(param["name"])
    param_type = param.get("type", "string")
    if param_type == "string":
        param_value = ParameterValue(param_value, value_type=str)
    elif param_type == "int":
        param_value = ParameterValue(param_value, value_type=int)
    elif param_type == "float" or param_type == "double":
        param_value = ParameterValue(param_value, value_type=float)
    elif param_type == "bool":
        param_value = ParameterValue(param_value, value_type=bool)
    else:
        raise ValueError(f"Unsupported parameter type: {param_type}")
    return (param["name"], param_value)


def generate_launch_description():
    return LaunchDescription(
        [add_launch_arg(param) for param in parameters]
        + [
            OpaqueFunction(
                function=launch_setup,
                kwargs={"params": dict([set_parameter(param) for param in parameters])},
            )
        ]
    )
