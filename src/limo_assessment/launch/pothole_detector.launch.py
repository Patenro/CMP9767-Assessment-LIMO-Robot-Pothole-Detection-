#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_assessment',
            executable='/home/adeola/CMP9767-Assessment/src/limo_assessment/limo_assessment/adeola_autonomous_navigation.py',
            output='log'
        ),
        Node(
            package='limo_assessment',
            executable='/home/adeola/CMP9767-Assessment/src/limo_assessment/limo_assessment/adeola_pothole_counter.py',
            output='log'
        ),
        Node(
            package='limo_assessment',
            executable='/home/adeola/CMP9767-Assessment/src/limo_assessment/limo_assessment/adeola_posedetector1.py',
        ),
        Node(
            package='limo_assessment',
            executable='/home/adeola/CMP9767-Assessment/src/limo_assessment/limo_assessment/adeola_marker.py',
        ),
        Node(
            package='limo_assessment',
            executable='/home/adeola/CMP9767-Assessment/src/limo_assessment/limo_assessment/adeola_severity_report.py',
            output='screen'
        )
    ])