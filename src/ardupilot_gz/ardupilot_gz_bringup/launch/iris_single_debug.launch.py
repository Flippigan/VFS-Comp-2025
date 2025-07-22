#!/usr/bin/env python3

"""
Single UAV debug launch file.
Tests basic ArduPilot-Gazebo communication without multi-UAV complexity.
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate single UAV debug launch description."""
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Launch arguments
    declare_mavproxy_arg = DeclareLaunchArgument(
        "mavproxy", default_value="true", description="Launch MAVProxy."
    )

    # Gazebo server and client
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_single_test.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # Single SITL instance with standard configuration
    sitl = ExecuteProcess(
        cmd=[
            "arducopter",
            "--model=gazebo-iris",
            "--speedup=1",
            "--home=-35.3632621,149.1652374,584.0,353.0",
            "--synthetic-clock",
        ],
        output="screen",
    )

    # MAVProxy for the UAV
    mavproxy = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master=tcp:127.0.0.1:5760",
            "--console",
            "--map",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mavproxy")),
    )

    return LaunchDescription([
        declare_mavproxy_arg,
        gz_sim_server,
        gz_sim_gui,
        sitl,
        mavproxy,
    ])