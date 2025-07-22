#!/usr/bin/env python3

"""Test launch file for single UAV debugging."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate test launch description for single UAV."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Gazebo server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway_swarm_minimal.sdf"}'
        }.items(),
    )

    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # ArduPilot SITL - Single instance, completely standard
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ardupilot_sitl"),
                "launch",
                "sitl.launch.py",
            ]
        ),
        launch_arguments={
            "synthetic_clock": "True",
            "wipe": "False", 
            "model": "gazebo-iris",  # Standard frame
            "speedup": "1",
            "slave": "0",
            "instance": "0",  # Single instance
            "defaults": os.path.join(
                pkg_ardupilot_gazebo,
                "config",
                "gazebo-iris-gimbal.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "sim_address": "127.0.0.1",
        }.items(),
    )

    return LaunchDescription([
        gz_sim_server,
        gz_sim_gui,
        sitl,
    ])