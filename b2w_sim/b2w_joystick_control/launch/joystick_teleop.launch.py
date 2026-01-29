import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='ps5_config.yaml',
        description='Configuration file name (in config folder)'
    )

    input_mode_arg = DeclareLaunchArgument(
        'input_mode',
        default_value='keyboard',
        description='Input mode: keyboard or joystick'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('b2w_joystick_control'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # Joy node - publishes joystick raw inputs to /joy topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 10.0,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('input_mode'), "' == 'joystick'"])
        ),
    )

    # Joystick teleop node - converts /joy to /cmd_vel
    teleop_node = Node(
        package='b2w_joystick_control',
        executable='joystick_teleop_node',
        name='joystick_teleop',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('input_mode'), "' == 'joystick'"])
        ),
    )

    keyboard_node = Node(
        package='b2w_joystick_control',
        executable='keyboard_teleop_tk.py',
        name='keyboard_teleop_tk',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('input_mode'), "' == 'keyboard'"])
        ),
    )

    return LaunchDescription([
        joy_dev_arg,
        config_file_arg,
        input_mode_arg,
        joy_node,
        teleop_node,
        keyboard_node,
    ])
