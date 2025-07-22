#!/usr/bin/env python3

"""
Final multi-UAV launch file using custom models with explicit port configurations.
This resolves the "Incorrect protocol magic" errors by ensuring SITL and Gazebo
use matching communication ports.
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare


def generate_launch_description():
    """Generate launch description for multi-UAV iris swarm with fixed port configuration."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")

    # Launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Open RViz."
    )
    declare_mavproxy_arg = DeclareLaunchArgument(
        "mavproxy", default_value="true", description="Launch MAVProxy for each UAV."
    )

    # Gazebo server with custom world containing properly configured models
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway_swarm_fixed.sdf"}'
        }.items(),
    )

    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # Default parameters for all UAVs
    default_params = os.path.join(
        pkg_ardupilot_gazebo,
        "config",
        "gazebo-iris-gimbal.parm",
    ) + "," + os.path.join(
        pkg_ardupilot_sitl,
        "config",
        "default_params",
        "dds_udp.parm",
    )

    # Launch each UAV with its SITL instance
    uavs = []
    for i in range(3):
        namespace = f"iris_{i}"
        uav_id = f"iris_{i}"
        
        # ArduPilot SITL launch for each UAV
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
                "instance": str(i),  # This auto-calculates ports: 9002+i*10
                "defaults": default_params,
                "sim_address": "127.0.0.1",
            }.items(),
        )
        
        # MAVProxy for each UAV (optional)
        mavlink_port = 5760 + i * 10
        mavproxy = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "launch",
                    "mavproxy.launch.py",
                ]
            ),
            launch_arguments={
                "master": f"tcp:127.0.0.1:{mavlink_port}",
                "load_modules": "gimbal",
            }.items(),
            condition=IfCondition(LaunchConfiguration("mavproxy")),
        )
        
        uavs.extend([sitl, mavproxy])

    return LaunchDescription([
        declare_rviz_arg,
        declare_mavproxy_arg,
        gz_sim_server,
        gz_sim_gui,
        *uavs,
    ])