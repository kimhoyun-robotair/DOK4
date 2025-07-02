#!/usr/bin/env python3
import rclpy
import math
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, 
    QoSDurabilityPolicy, QoSReliabilityPolicy
)

from px4_msgs.msg import (
    VehicleCommand, VehicleGlobalPosition, 
    VehicleStatus, VehicleAttitude
)
from pymavlink import mavutil
import geometry_utils as gu
import vehicle_command as vc

from pathlib import Path
import yaml
from mav_mission import MissionUploader, Waypoint

PKG_NAME = 'final_mission'

def load_config():
    """
    `share/<pkg>/config/config.yaml` 읽어서 dict 로 반환
    """
    share_dir = Path(
        get_package_share_directory(PKG_NAME)
    )
    cfg_path = share_dir / 'config' / 'config.yaml'
    with open(cfg_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


class OffboardControl(Node):
    def __init__(self):
        """ Initialize Node """
        """ Setting QoS Profile and Topic Pub / Sub """
        super().__init__("Offboard_control")

        # PX4와의 통신을 위한 QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Topic Publisher
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Topic Subscriber
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_position_callback, qos_profile)

        # MAVLink Connection
        try:
            self.master = mavutil.mavlink_connection('udpin:localhost:14540', timeout=10)
            self.get_logger().info("MAVLink connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to establish MAVLink connection: {e}")
            raise

        self.load_parameter()
        self.pre_calculate_waypoint()
        self.upload_mission()
        self.get_logger().info("Initialization Completed")

        self.timer = self.create_timer(0.2, self.timer_callback)
    
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡInitializationㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
    def pre_calculate_waypoint(self):
        for idx, (cur_wp, next_wp) in enumerate(
            zip(self.waypoints[:-1], self.waypoints[1:])
        ):
            bearing = gu.calculate_bearing(
                cur_wp['lat'],  cur_wp['lon'],
                next_wp['lat'], next_wp['lon']
            )
            cur_wp['next_wp_bearing'] = bearing
            self.get_logger().info(
                f"Waypoint {idx} → {idx+1} bearing: {math.degrees(bearing):.1f}°"
            )

    def load_parameter(self):
        cfg = load_config()
        self.waypoints = [
            w for w in cfg['waypoints'] if w.get('enabled', True)
        ]

        p = cfg['parameters']
        
        # 고도 파라미터
        self.initial_takeoff_altitude = p['initial_takeoff_altitude']
        self.final_altitude           = p['final_altitude']
        self.altitude_increment       = p['altitude_increment']
        self.current_target_altitude = self.initial_takeoff_altitude

        # yaw 정렬을 위한 파라미터
        self.yaw_adjustment_duration  = p['yaw_adjustment_duration']
        self.yaw_adjustment_start_time = None

        # 선회를 위한 파라미터
        self.loiter_radius            = p['loiter_radius']
        self.loiter_time              = p['loiter_time']

        # 경로점 도착 판단을 위한 파라미터
        self.threshold_range          = p['threshold_range']
        self.threshold_height         = p['threshold_height']

        # 기준점이 될 좌표
        self.ref_lat                  = p['ref_lat']
        self.ref_lon                  = p['ref_lon']

        # Flight State Initialization
        self.current_position = None
        self.current_attitude = None
        self.current_heading = 0.0
        self.flight_state = "INIT"
        self.current_wp_idx = 0

        # mission 모드 설정을 위한 파라미터
        self.mission_item_count = 0
        self.waypoint_error = 0
        self.previous_seq = 0
        self.point = 0

    def upload_mission(self) -> bool:
        # YAML → Waypoint dataclass 변환
        waypoints = [Waypoint(**d) for d in self.waypoints]

        uploader = MissionUploader(
            master         = self.master,           # pymavlink 연결 객체
            loiter_time    = self.loiter_time,      # YAML 파라미터 그대로
            loiter_radius  = self.loiter_radius,
            logger         = lambda m, e=False: (
                self.get_logger().error(m) if e else self.get_logger().info(m)
            )
        )
        return uploader.upload(waypoints)
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡCallback Groupㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback for vehicle attitude."""
        self.current_attitude = vehicle_attitude
        w, x, y, z = vehicle_attitude.q
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.current_heading = yaw  # current_heading 업데이트
        self.get_logger().debug(f"Current heading: {math.degrees(self.current_heading)} degrees")

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status."""
        self.vehicle_status = vehicle_status

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback for vehicle global position."""
        self.current_position = {"lat": vehicle_global_position.lat,
                                 "lon": vehicle_global_position.lon,
                                 "alt": vehicle_global_position.alt}
        
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡMAVLink Groupㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
# ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ#
    def wait_heartbeat(self):
        self.get_logger().info("Waiting for heartbeat...")
        try:
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info("Heartbeat received")
            return True
        except mavutil.mavlink.MAVError:
            self.get_logger().error("No heartbeat received")
            return False

    def start_mission(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Mission start command sent")

    def timer_callback(self):
        """Main FSM loop (runs every 0.2 s)."""
        # ── 0. 데이터 수신 대기 ──────────────────────────────────
        if None in (self.current_position,
                    self.vehicle_status,
                    self.current_attitude):
            self.get_logger().debug("Waiting for vehicle data…")
            return

        s = self.flight_state  # 단축 표기

        # ── 1. 상태 머신 ───────────────────────────────────────
        if s == "INIT":
            # Arm 이 끝나면 바로 TAKEOFF 단계로
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                vc.arm(self.vehicle_command_publisher, self.get_clock())
            else:
                self.flight_state = "TAKEOFF"

        elif s == "TAKEOFF":
            vc.takeoff(self.vehicle_command_publisher,
                       self.get_clock(),
                       self.initial_takeoff_altitude)
            self.flight_state = "WAIT_FOR_TAKEOFF"

        elif s == "WAIT_FOR_TAKEOFF":
            if (self.current_position["alt"] >=
                self.initial_takeoff_altitude - self.threshold_height):
                self.current_target_altitude = \
                    self.initial_takeoff_altitude + self.altitude_increment
                self.flight_state = "ADJUST_YAW_AND_ALTITUDE"

        elif s == "ADJUST_YAW_AND_ALTITUDE":
            # 목표 고도(↓)에 도달할 때까지 반복 상승
            if self.current_target_altitude <= self.final_altitude:
                first_wp = self.waypoints[0]
                yaw = gu.calculate_bearing(
                    self.current_position["lat"], self.current_position["lon"],
                    first_wp["lat"], first_wp["lon"]
                )
                vc.adjust_yaw_and_altitude(
                    self.vehicle_command_publisher, self.get_clock(),
                    yaw, self.current_target_altitude,
                    self.current_heading
                )
                self.yaw_adjustment_start_time = self.get_clock().now()
                self.flight_state = "WAIT_FOR_YAW_AND_ALTITUDE"
            else:
                self.flight_state = "START_MISSION"

        elif s == "WAIT_FOR_YAW_AND_ALTITUDE":
            elapsed = (self.get_clock().now() -
                       self.yaw_adjustment_start_time).nanoseconds / 1e9
            if elapsed >= self.yaw_adjustment_duration:
                self.current_target_altitude += self.altitude_increment
                self.flight_state = "ADJUST_YAW_AND_ALTITUDE"

        elif s == "START_MISSION":
            self.start_mission()
            self.flight_state = "MONITOR_MISSION"

        elif s == "MONITOR_MISSION":
            hit = self.master.recv_match(type='MISSION_ITEM_REACHED',
                                         blocking=False)
            if hit:
                seq = hit.seq - self.waypoint_error
                wp  = self.waypoints[seq]
                self.get_logger().info(f"Reached WP {seq} ({wp.get('id','')})")

                # ── Transition 처리 ──
                if wp.transition == "transition_fw":
                    vc.transition_fw(self.vehicle_command_publisher,
                                     self.get_clock())
                elif wp.transition == "transition_mc":
                    vc.transition_mc(self.vehicle_command_publisher,
                                     self.get_clock())

                # ── Action 처리 ──
                if wp.action == "loiter":
                    self.get_logger().info("Loiter maneuver…")
                elif wp.action == "land":
                    vc.land(self.vehicle_command_publisher, self.get_clock())
                    self.get_logger().info("Mission complete – landing")
                    self.flight_state = "MISSION_COMPLETE"

        elif s == "MISSION_COMPLETE":
            pass  # 추가 작업 없음

        else:
            self.get_logger().warn(f"Unknown flight_state: {s}")

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
