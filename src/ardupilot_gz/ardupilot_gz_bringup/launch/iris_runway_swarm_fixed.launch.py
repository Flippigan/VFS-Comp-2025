#!/usr/bin/env python3

"""
Fixed multi-UAV launch file with explicit port configuration.
This version ensures SITL and Gazebo ArduPilot plugin use matching ports.
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate fixed launch description for multi-UAV iris swarm."""
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

    # UAV configurations with explicit ports to match Gazebo models
    uavs = []
    
    for i in range(3):
        # Explicit port configuration that matches standard ArduPilot-Gazebo setup
        # Each UAV gets unique ports but using standard increments
        mavlink_port = 5760 + i * 10
        sim_port_in = 9003 + i * 10  # Gazebo -> SITL
        sim_port_out = 9002 + i * 10  # SITL -> Gazebo
        
        # ArduPilot SITL with explicit port configuration
        sitl = ExecuteProcess(
            cmd=[
                "arducopter",
                "--model=gazebo-iris",
                "--speedup=1", 
                "--slave=0",
                f"--instance={i}",
                f"--serial0=tcp:{mavlink_port}",  # Explicit MAVLink port
                f"--sim-address=127.0.0.1:{sim_port_out}",  # Explicit sim output port
                f"--sim-port-in={sim_port_in}",  # Explicit sim input port  
                f"--sim-port-out={sim_port_out}",  # Explicit sim output port
                f"--defaults={os.path.join(pkg_ardupilot_gazebo, 'config', 'gazebo-iris-gimbal.parm')},{os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'dds_udp.parm')}",
                "--synthetic-clock",
                f"--home=-35.3632621,149.1652374,584.0,353.0",  # Explicit home location
            ],
            output="screen",
        )

        # MAVProxy for each UAV
        mavproxy = ExecuteProcess(
            cmd=[
                "mavproxy.py",
                f"--master=tcp:127.0.0.1:{mavlink_port}",
                "--console",
                "--map",
                "--load-module=gimbal",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("mavproxy")),
        )
        
        uavs.extend([sitl, mavproxy])

    # Add launch arguments and UAV processes
    return LaunchDescription([
        declare_rviz_arg,
        declare_mavproxy_arg,
        gz_sim_server,
        gz_sim_gui,
        *uavs,
    ])