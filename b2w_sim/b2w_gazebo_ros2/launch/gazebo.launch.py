import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare all launch arguments
    is_macos = platform.system() == "Darwin"
    
    # world_file_path = PathJoinSubstitution([
    #     FindPackageShare("b2w_gazebo"),
    #     "worlds",
    #     LaunchConfiguration("world_file"),
    # ])

    world_file_path = PathJoinSubstitution([
        FindPackageShare("b2w_sim_worlds"),
        "worlds",
        LaunchConfiguration("world_file"),
    ])
    
    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value="ISAACLAB_TRAIN.world",
            description="World file to load from b2w_sim_worlds/worlds",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="2.5"),  # Increased from 1.8 to 2.5
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument("run_gui", default_value="true"),
    ]

    # Include the existing load.launch.py
    load_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("b2w_description_ros2"),
                "launch",
                "load.launch.py"
            ])
        ])
    )

    gz_sim_launch_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    )

    # macOS: force split mode. Other platforms: keep single-process mode.
    # macOS: 强制 server/gui 分离。其他平台：使用单进程模式。
    if is_macos:
        gzserver = IncludeLaunchDescription(
            gz_sim_launch_source,
            launch_arguments={
                "gz_args": [
                    "-s ",
                    PythonExpression(
                        ["'-r ' if '", LaunchConfiguration("paused"), "' == 'false' else ''"]
                    ),
                    PythonExpression(
                        ["'-v4 ' if '", LaunchConfiguration("verbose"), "' == 'true' else ''"]
                    ),
                    world_file_path,
                ],
                "on_exit_shutdown": "true",
            }.items(),
        )

        gzclient = IncludeLaunchDescription(
            gz_sim_launch_source,
            launch_arguments={
                "gz_args": [
                    "-g ",
                    PythonExpression(
                        ["'-v4 ' if '", LaunchConfiguration("verbose"), "' == 'true' else ''"]
                    ),
                ],
            }.items(),
            condition=IfCondition(LaunchConfiguration("run_gui")),
        )

        gz_actions = [gzserver, gzclient]
    else:
        gz_sim = IncludeLaunchDescription(
            gz_sim_launch_source,
            launch_arguments={
                "gz_args": [
                    PythonExpression(
                        ["'-r ' if '", LaunchConfiguration("paused"), "' == 'false' else ''"]
                    ),
                    PythonExpression(
                        ["'-v4 ' if '", LaunchConfiguration("verbose"), "' == 'true' else ''"]
                    ),
                    world_file_path,
                ],
                "on_exit_shutdown": "true",
            }.items(),
        )

        gz_actions = [gz_sim]

    # Spawn robot model
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",  # Use the topic from load.launch.py
            "-name", "b2w",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    # ros_gz_bridge config
    ros_gz_bridge_config = PathJoinSubstitution([
        FindPackageShare("b2w_gazebo_ros2"), "config", "b2w_gz_bridge.yaml"
    ])

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "--ros-args",
            "-p",
            ["config_file:=", ros_gz_bridge_config],
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            load_launch,
        ]
        + gz_actions
        + [
            spawn_robot,
            ros_gz_bridge,
        ]
    )
