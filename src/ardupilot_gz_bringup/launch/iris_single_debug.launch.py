#!/usr/bin/env python3

"""
Single UAV debug launch file for testing ArduPilot-Gazebo communication.
This is a simplified version for debugging basic functionality.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Generate single UAV debug launch description."""
    
    # Simple SITL launch for testing
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

    # Simple MAVProxy launch for testing
    mavproxy = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master=tcp:127.0.0.1:5760",
            "--console",
            "--map",
        ],
        output="screen",
    )

    return LaunchDescription([
        sitl,
        mavproxy,
    ])