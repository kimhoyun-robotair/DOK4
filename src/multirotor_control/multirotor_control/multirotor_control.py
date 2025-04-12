import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition

from tf_transformations import euler_from_quaternion, quaternion_from_euler

EARTH_RADIUS = 6371000.0  # 지구 반경 (미터)

class OffboardControl(Node):
    def __init__(self):
        """ Initialize Node """
        """ Setting QoS Profile and Topic Pub / Sub """
        super().__init__("Offboard_control")

        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameter Declaration
        self.declare_parameter("topic_offboard_control_mode", "/fmu/in/offboard_control_mode")
        self.declare_parameter("topic_vehicle_attitude_setpoint", "/fmu/in/vehicle_attitude_setpoint")
        self.declare_parameter("topic_trajectory_setpoint", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("topic_vehicle_local_position", "/fmu/out/vehicle_local_position")
        self.declare_parameter("topic_vehicle_attitude", "/fmu/out/vehicle_attitude")
        self.declare_parameter("topic_vehicle_status", "/fmu/out/vehicle_status_v1")

        # Parameter Value
        topic_offboard_control_mode = self.get_parameter("topic_offboard_control_mode").value
        topic_vehicle_attitude_setpoint = self.get_parameter("topic_vehicle_attitude_setpoint").value
        topic_trajectory_setpoint = self.get_parameter("topic_trajectory_setpoint").value
        topic_vehicle_command = self.get_parameter("topic_vehicle_command").value
        topic_vehicle_local_position = self.get_parameter("topic_vehicle_local_position").value
        topic_vehicle_attitude = self.get_parameter("topic_vehicle_attitude").value
        topic_vehicle_status = self.get_parameter("topic_vehicle_status").value

        # Topic Publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, topic_offboard_control_mode, qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, topic_vehicle_attitude_setpoint, qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, topic_trajectory_setpoint, qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, topic_vehicle_command, qos_profile)

        # Topic Subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, topic_vehicle_local_position, self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, topic_vehicle_attitude, self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, topic_vehicle_status, self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_position_callback, qos_profile)

        # WGS84 Waypoints
        self.wgs84_waypoints = {
            "WP0": {"lat": 37.449187, "lon": 126.653021, "alt": 0.0},
            "WP1": {"lat": 37.449187, "lon": 126.653021, "alt": 20.0},
            "WP2": {"lat": 37.452717, "lon": 126.653195, "alt": 20.0},
            "WP3": {"lat": 37.454559, "lon": 126.657573, "alt": 20.0},
            "WP4": {"lat": 37.454490, "lon": 126.648858, "alt": 20.0},
            "WP5": {"lat": 37.452717, "lon": 126.653195, "alt": 20.0},
            "WP6": {"lat": 37.452059, "lon": 126.647888, "alt": 3.0},
            "WP7": {"lat": 37.451786, "lon": 126.659597, "alt": 20.0},
            "WP8": {"lat": 37.452717, "lon": 126.653195, "alt": 10.0},
            "WP9": {"lat": 37.449187, "lon": 126.653021, "alt": 0.0}
        }
        # NED Waypoints
        self.ned_waypoints = {}
        # Mission Profile
        self.mission_waypoints = {
            "WP0": {"action": "TAKEOFF", "transition": None},
            "WP1": {"action": None, "transition": None},
            "WP2": {"action": None, "transition": None},
            "WP3": {"action": None, "transition": None},
            "WP4": {"action": None, "transition": None},
            "WP5": {"action": None, "transition": None},
            "WP6": {"action": None, "transition": None},
            "WP7": {"action": None, "transition": None},
            "WP8": {"action": None, "transition": None},
            "WP9": {"action": "LAND", "transition": None}
        }
        self.pre_calculate_waypoint_and_initialize_variable()

####################### Initialize and Coordinate Conversion Functions #######################
    def pre_calculate_waypoint_and_initialize_variable(self):
        """ Pre-calculate waypoints and ready vehicle """
        """ Calculate WGS84 to NED Coordinate for using in px4-msgs"""
        """ And Initialize various variables, Setting Timer Callback"""
        # 경로기준범 초기화
        self.ref_lat = math.radians(self.wgs84_waypoints["WP0"]["lat"])
        self.ref_lon = math.radians(self.wgs84_waypoints["WP0"]["lon"])
        self.ref_sin_lat = math.sin(self.ref_lat)
        self.ref_cos_lat = math.cos(self.ref_lat)
        self.ref_init_done = True

        # WGS84 좌표를 NED 좌표로 변환한 후에 저장
        for wp_name, wp_data in self.wgs84_waypoints.items():
            lat = wp_data["lat"]
            lon = wp_data["lon"]
            alt = wp_data["alt"]
            x, y = self.wgs84_to_ned(lat, lon)
            self.ned_waypoints[wp_name] = {"x": x, "y": y, "z": alt}
        self.get_logger().info(f"Pre-calculated waypoints in NED coordinates:\n{self.ned_waypoints}")

        # 각종 변수들 초기화 및 선언
        self.state = None
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.dist = 0.0

        # timer 콜백 함수 설정
        self.timer = self.create_timer(0.01, self.timer_callback)

    def wgs84_to_ned(self, lat, lon):
        """
        주어진 위도와 경도(degrees)를 기준점에 대해 Azimuthal Equidistant Projection 방식으로 투영하여
        로컬 좌표 (x, y) 를 미터 단위로 계산
        """
        # 입력 좌표를 라디안으로 변환
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        cos_d_lon = math.cos(lon_rad - self.ref_lon)

        # 내적 값 계산 후 -1~1 범위로 제한
        arg = self.ref_sin_lat * sin_lat + self.ref_cos_lat * cos_lat * cos_d_lon
        arg = max(min(arg, 1.0), -1.0)
        c = math.acos(arg)

        # c가 0이 아니면 비율 k 계산 (c/sin(c))
        k = c / math.sin(c) if abs(c) > 1e-6 else 1.0

        x = k * (self.ref_cos_lat * sin_lat - self.ref_sin_lat * cos_lat * cos_d_lon) * EARTH_RADIUS
        y = k * cos_lat * math.sin(lon_rad - self.ref_lon) * EARTH_RADIUS

        return x, y

    def get_distance(self):
        pass

    def calculate_bearing():
        pass

    def is_waypoint_reached():
        pass
######################## Callback Functions #######################
                # Functions for subscribing messages"""
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback for vehicle attitude."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status."""
        self.vehicle_status = vehicle_status

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback for vehicle global position."""
        self.vehicle_global_position = vehicle_global_position

########################## Command Functions #######################
                    # Functions for Offboard Control
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, params=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Send a command to engage offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Send a command to land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_heartbeat_ob_pos_sp(self):
        """Publish heartbeat message for offboard control."""
        msg = OffboardControlMode()
        msg.position    = True
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.clip(np.deg2rad(yaw_d), -np.pi, np.pi)
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
############################## Timer Callback Function #######################
# Functions for Finite State Machine and Sequential Offboard Control
    def timer_callback(self):
        self.vehicle_euler = euler_from_quaternion(self.vehicle_attitude.q)
        self._roll_d  = np.rad2deg(self.vehicle_euler[2])
        self._pitch_d = np.rad2deg(self.vehicle_euler[1])
        self._yaw_d   = np.rad2deg(self.vehicle_euler[0])
        print("S{:d} Time {:.2f}, ".format(self.state, (self.get_clock().now().nanoseconds/1000000000)%1000.0), end=' ')

        # Take-off 상태 처리
        if self.state == 0:
            self.publish_heartbeat_ob_pos_sp()
            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
            self.offboard_setpoint_counter %= 11
            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 9:
                self.arm()

            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if (self.vehicle_local_position.z <= self.takeoff_height + 1 and
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.state = 1

        elif self.state == 1:
            self.publish_heartbeat_ob_pos_sp()
            self.land()
            exit(0)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
