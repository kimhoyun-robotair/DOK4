import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) 먼저 시뮬레이션 시간 플래그를 선언
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )

    # 2) 이제 각 Node 에서 use_sim_time 인자를 참조할 수 있다.
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/vertiport/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/world/vertiport/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/world/vertiport/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info', '/camera/camera_info'),
            ('/world/vertiport/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image', '/camera/image'),
        ]
    )

    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['/camera/camera_info', '/camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    aruco_detector_node = Node(
        package='precision_landing',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'tag_id': 96},        # AprilTag ID
            {'marker_size': 0.5},  # 마커 크기 (미터)
        ],
    )

    ld = LaunchDescription()

    # → DeclareLaunchArgument 은 반드시 다른 Action(노드)보다 먼저 등록해야 한다.
    ld.add_action(sim_time_arg)
    ld.add_action(gz_bridge_node)
    ld.add_action(relay_camera_info_node)
    ld.add_action(aruco_detector_node)

    return ld
