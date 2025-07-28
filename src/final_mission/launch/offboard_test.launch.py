import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    final_mission = get_package_share_directory("final_mission")
    param_file = os.path.join(final_mission, "config", "waypoint.yaml")

    offboard_test_node = Node(
        package = "final_mission",
        executable="autonomous_flight_mission_offboard.py",
        name="offboard_test",
        output="screen",
        parameters=[param_file]
    )

    ld = LaunchDescription()
    ld.add_action(offboard_test_node)

    return ld
    
