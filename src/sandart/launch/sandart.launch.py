#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    sand_clear = Node(
        package="sandart",
        executable="sand_clear_node",
        name="sand_clear_node",
        output="screen",
    )
    path_plan = Node(
        package="sandart",
        executable="path_plan_node",
        name="path_plan_node",
        output="screen",
    )

    skeleton_processor = Node(
        package="sandart",
        executable="skeleton_processor_node",
        name="skeleton_processor_node",
        output="screen",
    )

    lifecycle_manager = Node(
        package="sandart",
        executable="lifecycle_manage_node",
        name="lifecycle_manage_node",
        output="screen",
    )

    sandart_movesx = Node(
        package="sandart",
        executable="sandart_movesx_node",
        name="sandart_movesx_node",
        output="screen",
    )

    return LaunchDescription([
        lifecycle_manager,
        skeleton_processor,
        path_plan,
        sandart_movesx,
        sand_clear,
    ])