#!/usr/bin/env python3

"""
Simplified multi-UAV launch file using separate standard SITL instances.
Each UAV uses completely independent standard ArduPilot-Gazebo configuration.
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate simplified launch description for multi-UAV iris swarm."""
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Launch arguments
    declare_mavproxy_arg = DeclareLaunchArgument(
        "mavproxy", default_value="true", description="Launch MAVProxy for each UAV."
    )

    # Gazebo server and client
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway_swarm.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # Three separate SITL instances - each completely independent
    # UAV 0: Standard ports (5760, 9002/9003)
    sitl_0 = ExecuteProcess(
        cmd=[
            "arducopter",
            "--model=gazebo-iris",
            "--speedup=1",
            "--home=-35.3632621,149.1652374,584.0,353.0",
            "--synthetic-clock",
        ],
        output="screen",
        name="sitl_0",
    )

    mavproxy_0 = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master=tcp:127.0.0.1:5760",
            "--console",
            "--map",
            "--load-module=gimbal",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mavproxy")),
        name="mavproxy_0",
    )

    # UAV 1: Offset ports (5761, 9004/9005) 
    sitl_1 = ExecuteProcess(
        cmd=[
            "arducopter",
            "--model=gazebo-iris",
            "--speedup=1",
            "--home=-35.3632621,149.1652374,584.0,353.0",
            "--synthetic-clock",
            "--serial0=tcp:5761",  # Custom MAVLink port
            "--sim-address=127.0.0.1:9004",  # Custom sim port
        ],
        output="screen",
        name="sitl_1",
    )

    mavproxy_1 = ExecuteProcess(
        cmd=[
            "mavproxy.py", 
            "--master=tcp:127.0.0.1:5761",
            "--console",
            "--map",
            "--load-module=gimbal",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mavproxy")),
        name="mavproxy_1",
    )

    # UAV 2: Offset ports (5762, 9006/9007)
    sitl_2 = ExecuteProcess(
        cmd=[
            "arducopter",
            "--model=gazebo-iris",
            "--speedup=1", 
            "--home=-35.3632621,149.1652374,584.0,353.0",
            "--synthetic-clock",
            "--serial0=tcp:5762",  # Custom MAVLink port
            "--sim-address=127.0.0.1:9006",  # Custom sim port
        ],
        output="screen",
        name="sitl_2",
    )

    mavproxy_2 = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master=tcp:127.0.0.1:5762", 
            "--console",
            "--map",
            "--load-module=gimbal",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mavproxy")),
        name="mavproxy_2",
    )

    return LaunchDescription([
        declare_mavproxy_arg,
        gz_sim_server,
        gz_sim_gui,
        sitl_0,
        mavproxy_0,
        sitl_1,
        mavproxy_1, 
        sitl_2,
        mavproxy_2,
    ])