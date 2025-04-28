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

    mapping_pub_node = Node(
        package='project',
        executable='', # Fill this
        name='', # Fill this
        output='screen',
    )

    """ Turtlebot 3 config"""

    """
    1. export ROS_DOMAIN_ID=11
    2. export TURTLEBOT3_MODEL=burger
    3. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
    4. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
    5. ros2 launch my_turtlebot turtlebot_simulation.launch.py
    """

    # export ROS_DOMAIN_ID=11
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='11'
    )

    # export TURTLEBOT3_MODEL=burger
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    my_turtlebot_model_path = os.path.join(
        FindPackageShare('my_turtlebot'),
        'share',
        'my_turtlebot',
        'models'
    )

    turtlebot3_gazebo_model_path = os.path.join(
        FindPackageShare('turtlebot3_gazebo'),
        'share',
        'turtlebot3_gazebo',
        'models'
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join([
            os.environ.get('GAZEBO_MODEL_PATH', ''),
            my_turtlebot_model_path,
            turtlebot3_gazebo_model_path
        ])
    )

    # ros2 launch my_turtlebot turtlebot_simulation.launch.py
    turtlebot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('my_turtlebot'),
            'launch',
            'turtlebot_simulation.launch.py'

        ])
    )

    return LaunchDescription([
        set_domain_id,
        set_turtlebot3_model,
        set_gazebo_model_path,
        turtlebot_simulation_launch,

        mapping_pub_node,
        nav_service_node,
        explore_service_node,
        explorer_node
    ])
