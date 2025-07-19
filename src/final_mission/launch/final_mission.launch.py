from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    final_mission_node = Node(
        package="final_mission",
        executable="test.py",
        name="final_mission",
        output="screen",
    )


    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(final_mission_node)
    
    return launchDescriptionObject