import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_service_node = Node(
        package='project',
        executable='nav_srv',
        name='navigation_service',
        output='screen',
    )

    explore_service_node = Node(
        package='project',
        executable='explore_srv',
        name='exploration_service',
        output='screen',
    )

    explorer_node = Node(
        package='project',
        executable='explorer',
        name='explorer',
        output='screen',
    )

    # mapping_pub_node = Node(
    #     package='project',
    #     executable='', # Fill this
    #     name='', # Fill this
    #     output='screen',
    # )

    return LaunchDescription([
        # mapping_pub_node,
        nav_service_node,
        explore_service_node,
        explorer_node
    ])
