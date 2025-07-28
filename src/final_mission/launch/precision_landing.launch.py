import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    final_mission = get_package_share_directory("final_mission")
    param_file = os.path.join(final_mission, "config", "waypoint.yaml")

    aruco_tracker_share = get_package_share_directory('aruco_tracker')
    precision_landing_share = get_package_share_directory('precision_land')

    fsm_node = Node(
        package = "final_mission",
        executable="fsm",
        name="fsm_node",
        output="screen",
        prefix=['gnome-terminal -- ']
    )

    offboard_test_node = Node(
        package = "final_mission",
        executable="autonomous_precision_landing.py",
        name="offboard_control_node",
        output="screen",
        parameters=[param_file],
        prefix=['gnome-terminal -- ']
    )


    aruco_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_tracker_share, 'launch', 
                                  'sim_lifecycle_aruco_tracker.launch.py'])
        ),
    )

    precision_land = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([precision_landing_share, 'launch', 
                                  'sim_lifecycle_precision_land.launch.py'])
        ),
    )

    ld = LaunchDescription()
    ld.add_action(offboard_test_node)
    ld.add_action(fsm_node)
    ld.add_action(aruco_tracker)
    ld.add_action(precision_land)

    return ld
    
