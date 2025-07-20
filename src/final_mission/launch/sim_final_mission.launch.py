import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description():
    aruco_tracker = get_package_share_directory('aruco_tracker')
    aruco_tracker_launch_dir = os.path.join(aruco_tracker, 'launch')
    precision_landing = get_package_share_directory('precision_land')
    precision_landing_launch_dir = os.path.join(precision_landing, 'launch')

    final_mission_node = Node(
        package="final_mission",
        executable="final_mission.py",
        name="offboard_control",
        output="screen",
    )

    # Define bringup actions
    bringup_cmd_group = GroupAction([
        # SLAM launch unconditionally
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aruco_tracker_launch_dir, 'sim_lifecycle_aruco_tracker.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(precision_landing_launch_dir, 'sim_lifecycle_precision_land.launch.py')
            ),
        ),
    ])


    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(final_mission_node)
    launchDescriptionObject.add_action(bringup_cmd_group)
    
    return launchDescriptionObject