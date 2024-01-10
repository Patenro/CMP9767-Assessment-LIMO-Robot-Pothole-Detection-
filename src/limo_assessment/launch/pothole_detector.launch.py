#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_assessment',
            executable='adeola_autonomous_navigation1'
        ),
        Node(
            package='limo_assessment',
            executable='adeola_pothole_counter',
            output='log'
        ),
        Node(
            package='limo_assessment',
            executable='adeola_posedetector1'
        ),
        Node(
            package='limo_assessment',
            executable='adeola_marker'
        ),
        Node(
            package='limo_assessment',
            executable='adeola_severity_report',
            output='screen'
        ),
        Node(
            package='limo_assessment',
            executable='adeola_marker',
        )
        
    ])