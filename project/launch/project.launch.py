from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='11'
    )

    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    turtlebot3_gazebo_model_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'models'
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=turtlebot3_gazebo_model_path
    )

    turtlebot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        set_domain_id,
        set_turtlebot3_model,
        set_gazebo_model_path,
        turtlebot_simulation_launch,
        nav_service_node,
        explore_service_node,
        explorer_node
    ])

