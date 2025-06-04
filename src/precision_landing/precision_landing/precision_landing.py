#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, 
                       QoSDurabilityPolicy, QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleLocalPosition,VehicleAttitude,
    VehicleLandDetected,TrajectorySetpoint,
    OffboardControlMode, VehicleStatus, VehicleCommand
)


class PrecisionLandNode(Node):
    def __init__(self):
        super().__init__('precision_land_node')

        # ----------------------------------------
        # PX4와의 통신을 위한 QoS 프로파일 설정
        # ----------------------------------------
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------
        # 1) 파라미터 선언 및 로드
        # ----------------------------------------
        self.declare_parameter('descent_vel', 1.0)       # m/s
        self.declare_parameter('vel_p_gain', 1.5)
        self.declare_parameter('vel_i_gain', 0.0)
        self.declare_parameter('max_velocity', 3.0)      # m/s (한계 속도)
        self.declare_parameter('target_timeout', 3.0)    # 초
        self.declare_parameter('delta_position', 0.25)   # m (위치 도달 기준)
        self.declare_parameter('delta_velocity', 0.25)   # m/s (속도 안정 기준)

        self._param_descent_vel = self.get_parameter('descent_vel').value
        self._param_vel_p_gain = self.get_parameter('vel_p_gain').value
        self._param_vel_i_gain = self.get_parameter('vel_i_gain').value
        self._param_max_velocity = self.get_parameter('max_velocity').value
        self._param_target_timeout = self.get_parameter('target_timeout').value
        self._param_delta_position = self.get_parameter('delta_position').value
        self._param_delta_velocity = self.get_parameter('delta_velocity').value

        self.get_logger().info(f'descent_vel: {self._param_descent_vel}')
        self.get_logger().info(f'vel_p_gain: {self._param_vel_p_gain}')
        self.get_logger().info(f'vel_i_gain: {self._param_vel_i_gain}')

        # ----------------------------------------
        # 2) 내부 상태 변수 초기화
        # ----------------------------------------
        # ArUco 마커(타겟) 정보
        self._tag_position = None      # [x, y, z] (NED)
        self._tag_orientation = None   # quaternion (w, x, y, z)
        self._tag_timestamp = None     # rclpy.Time

        # 드론 현재 상태
        self._vehicle_position = None  # [x, y, z] (NED)
        self._vehicle_velocity = None  # [vx, vy, vz] (NED)
        self._vehicle_attitude = None  # quaternion (w, x, y, z)
        self._land_detected = False
        self.vehicle_status = VehicleStatus()

        # 통신 플래그
        self._target_lost_prev = True

        # 검색 패턴(스파이럴) 관련
        self._search_waypoints = []
        self._search_waypoint_index = 0
        self._search_started = False
        self._approach_altitude = 0.0

        # PI 제어기 내부 상태
        self._vel_x_integral = 0.0
        self._vel_y_integral = 0.0

        # NED로 주어지는 비행기의 경로점
        self.ned_waypoints = {
            "WP0": {"x": 0.0, "y": 0.0, "z": -5.0},
            "WP1": {"x": 1.4, "y": 0.0, "z": -5.0}
        }

        # 기체의 현재 오프보드 제어 State 추정
        from enum import Enum
        class State(Enum):
            IDLE     = 0
            SEARCH   = 1
            APPROACH = 2
            DESCEND  = 3
            FINISHED = 4

        self._state = State.SEARCH

        # ----------------------------------------
        # 3) 구독자 선언 !! 
        # ----------------------------------------
        # (1) ArUco 마커 포즈
        self._target_pose_sub = self.create_subscription(
            PoseStamped,
            '/apriltag_pose',
            self._target_pose_callback,
            1
        )

        # (2) 드론의 로컬 위치 (NED)
        self._vehicle_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._vehicle_local_position_callback,
            qos_profile
        )

        # (3) 드론의 자세
        self._vehicle_att_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._vehicle_attitude_callback,
            qos_profile
        )

        # (4) 착륙 감지
        self._land_detected_sub = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self._vehicle_land_detected_callback,
            qos_profile
        )
        
        # (5) 드론 상태
        self._vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            qos_profile
        )

        # ----------------------------------------
        # 4) 퍼블리셔 선언 !!
        # ----------------------------------------
        # TrajectorySetpoint: Offboard용 위치/속도/요 셋포인트
        self._traj_sp_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        # 오프보드 제어 모드 퍼블리셔
        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile
        )
        # 기체에 제어 신호를 주기 위해서 사용
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        # ----------------------------------------
        # 5) 타이머 설정 (업데이트 루프)
        # ----------------------------------------
        timer_period = 1.0 / 20.0  # 20Hz
        self._timer = self.create_timer(timer_period, self._update_callback)

        self.get_logger().info('PrecisionLandNode 초기화 완료 — SEARCH 상태로 시작')
# -------------------------------------------------------------
# 기체의 각종 좌표 처리를 위한 함수들
# -------------------------------------------------------------
    def get_distance_between_ned(self, x_now, y_now, z_now, x_next, y_next, z_next):
        # NED 좌표로 주어진 두 점 사이의 거리 구하기
        dx = x_next - x_now
        dy = y_next - y_now
        dz = z_next - z_now

        dist_xy = math.sqrt(dx*dx+dy*dy)
        dist_z = abs(dz)
        total_dist = math.sqrt(dist_xy*dist_xy + dist_z*dist_z)

        return total_dist, dist_xy, dist_z

    def _position_reached(self, target):
        if self._vehicle_position is None or self._vehicle_velocity is None:
            return False

        dx = target[0] - self._vehicle_position[0]
        dy = target[1] - self._vehicle_position[1]
        dz = target[2] - self._vehicle_position[2]

        # 위치 오차와 현재 속도 크기 확인
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        vel_norm = math.sqrt(
            self._vehicle_velocity[0]**2 +
            self._vehicle_velocity[1]**2 +
            self._vehicle_velocity[2]**2
        )

        return (dist < self._param_delta_position) and (vel_norm < self._param_delta_velocity)
        
    def get_attitude(self, current_wp, next_wp):
        # NED 좌표로 주어진 두 점 사이의 yaw 값 계산하기 (제어에 사용)
        dx = next_wp["x"] - current_wp["x"]
        dy = next_wp["y"] - current_wp["y"]

        # arctan2는 (dy, dx)를 인자로 받아 라디안 단위의 각도를 반환.
        yaw_rad = math.atan2(dy, dx)

        # 라디안을 도 단위로 변환
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg

# -------------------------------------------------------------
# 콜백 함수들의 그룹                                              |
# -------------------------------------------------------------
    # -----------------------------
    # 아루코 마커 포즈 콜백
    # -----------------------------
    def _target_pose_callback(self, msg: PoseStamped):
        # 여기서는 이미 “msg.pose.position”이 NED 좌표라고 가정
        self._tag_position = [msg.pose.position.x,
                              msg.pose.position.y,
                              msg.pose.position.z]
        self._tag_orientation = [msg.pose.orientation.w,
                                 msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z]
        # 타임스탬프를 ROS2 시간으로 기록
        self._tag_timestamp = Time.from_msg(msg.header.stamp)

    # -----------------------------
    # 콜백: 드론 로컬 위치 수신 (NED)
    # -----------------------------
    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        # msg.x, msg.y, msg.z 는 NED 좌표 (m)
        # msg.vx, msg.vy, msg.vz 는 NED 속도 (m/s)
        self._vehicle_position = [msg.x, msg.y, msg.z]
        self._vehicle_velocity = [msg.vx, msg.vy, msg.vz]

    # -----------------------------
    # 콜백: 드론 자세(쿼터니언) 수신
    # -----------------------------
    def _vehicle_attitude_callback(self, msg: VehicleAttitude):
        # msg.q[0]=w, [1]=x, [2]=y, [3]=z
        self._vehicle_attitude = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]

    # -----------------------------
    # 콜백: 착륙 감지 수신
    # -----------------------------
    def _vehicle_land_detected_callback(self, msg: VehicleLandDetected):
        self._land_detected = bool(msg.landed)

    # -----------------------------
    # 콜백: 드론 상태 수신
    # -----------------------------
    def _vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
# -------------------------------------------------------------
# 드론을 제어하기 위한 함수들 모임                                   |
# -------------------------------------------------------------
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
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
        self._offboard_control_mode_pub.publish(msg)
    
    def publish_heartbeat_ob_vel_sp(self):
        """Publish heartbeat message for offboard velocity control."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = True
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.clip(np.deg2rad(yaw_d), -np.pi, np.pi)
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self._traj_sp_pub.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, yaw_v: float):
        """Publish velocity setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [np.nan, np.nan, np.nan]
        msg.velocity    = [float(vx), float(vy), float(vz)]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = yaw_v if yaw_v is not None else np.nan
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self._traj_sp_pub.publish(msg)

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
        self._vehicle_command_pub.publish(msg)

# -------------------------------------------------------------
# 정밀착륙을 위한 각종 유틸 함수들의 구현                              |
# -------------------------------------------------------------
    # --------------------------------
    # 타겟 “타임아웃” 체크
    # --------------------------------
    def _check_target_timeout(self) -> bool:
        if self._tag_position is None or self._tag_timestamp is None:
            return True
        elapsed = (self.get_clock().now() - self._tag_timestamp).seconds
        return elapsed > self._param_target_timeout

    # --------------------------------
    # PI 제어기로 XY 속도 계산
    # --------------------------------
    def _calculate_velocity_setpoint_xy(self):
        if self._vehicle_position is None:
            return 0.0, 0.0

        # 드론 위치 정보
        dx = self._vehicle_position[0] - self._tag_position[0]
        dy = self._vehicle_position[1] - self._tag_position[1]

        # I 항 (누적 오차)
        self._vel_x_integral += dx
        self._vel_y_integral += dy
        max_i = self._param_max_velocity
        self._vel_x_integral = max(-max_i, min(max_i, self._vel_x_integral))
        self._vel_y_integral = max(-max_i, min(max_i, self._vel_y_integral))

        # P 항
        Xp = dx * self._param_vel_p_gain
        Yp = dy * self._param_vel_p_gain

        # I 항
        Xi = self._vel_x_integral * self._param_vel_i_gain
        Yi = self._vel_y_integral * self._param_vel_i_gain

        # 속도 (태그를 향하도록 부호 반전)
        vx = -1.0 * (Xp + Xi)
        vy = -1.0 * (Yp + Yi)

        # 속도 한계 clamp
        mv = self._param_max_velocity
        vx = max(-mv, min(mv, vx))
        vy = max(-mv, min(mv, vy))

        return vx, vy

    # --------------------------------
    # 스파이럴 검색 웨이포인트 생성
    # --------------------------------
    def _generate_search_waypoints(self):
        # 드론의 현재 고도(음수 NED Z 값) 가져오기
        if self._vehicle_position is None:
            start_z = 0.0
        else:
            start_z = self._vehicle_position[2]

        min_z = -1.0  # 가장 낮은 고도 (예: -1 m)
        max_radius = 2.0  # 최대 반경 (m)
        layer_spacing = 0.5  # 레이어 간격 (m)
        points_per_layer = 16

        waypoints = []
        current_z = start_z

        # 레이어 수 계산 (Outward + Inward 용)
        num_layers = int(abs(min_z - start_z) / layer_spacing)
        if num_layers < 1:
            num_layers = 1

        for layer in range(num_layers):
            layer_waypoints = []
            radius = 0.0

            # 외부로 나가는(Outward) 스파이럴
            for i in range(points_per_layer + 1):
                angle = 2.0 * math.pi * i / points_per_layer
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z = current_z
                layer_waypoints.append([x, y, z])
                radius += max_radius / points_per_layer

            waypoints.extend(layer_waypoints)

            # 높이를 한 단계 낮춘 뒤, 반대로 스파이럴 Inward
            current_z -= layer_spacing
            layer_waypoints.reverse()
            for p in layer_waypoints:
                p[2] = current_z
            waypoints.extend(layer_waypoints)

            # 다음 레이어로 이동
            current_z -= layer_spacing

        self._search_waypoints = waypoints
        self._search_waypoint_index = 0
        self.get_logger().info(f"검색 웨이포인트 {len(waypoints)}개 생성 완료")

    # --------------------------------
    # “정밀착륙 완료” 처리 (성공 or 실패)
    # --------------------------------
    def _switch_to_finished(self, success: bool):
        if success:
            self.get_logger().info("Precision Land: SUCCESS")
        else:
            self.get_logger().info("Precision Land: FAILURE")

        # 마지막에는 IDLE 상태로 전환
        self._state = self.State.IDLE

    # --------------------------------
    # 상태 전환 유틸
    # --------------------------------
    def _switch_state(self, new_state):
        self._state = new_state
        self.get_logger().info(f"Switching to { self._state_name(new_state) }")


    def _state_name(self, state):
        return state.name

    # -----------------------------
    # 주기적 업데이트 (20Hz)
    # -----------------------------
    def _update_callback(self):
        # (1) 타겟이 “타임아웃”되었는지 체크
        target_lost = self._check_target_timeout()

        if target_lost and not self._target_lost_prev:
            self.get_logger().info(f"Target lost: State {self._state_name(self._state)}")
        elif not target_lost and self._target_lost_prev:
            self.get_logger().info("Target acquired")

        self._target_lost_prev = target_lost

        # (2) 상태 머신
        if self._state == self.State.IDLE:
            # 대기만 함
            return

        elif self._state == self.State.SEARCH:
            # (a) 태그를 찾았으면 APPROACH로 전환
            if self._tag_position is not None:
                # 드론 현재 고도를 저장 → 나중에 하강 단계의 기준 고도로 쓴다
                if self._vehicle_position:
                    self._approach_altitude = self._vehicle_position[2]
                self._switch_state(self, self.State.APPROACH)
                return

            # (b) 타겟을 못 찾았으면 미리 생성한 웨이포인트 따라 순환
            if not self._search_started:
                self._generate_search_waypoints()
                self._search_started = True

            current_wp = self._search_waypoints[self._search_waypoint_index]
            # 위치 셋포인트로 퍼블리시
            self._publish_position_setpoint(current_wp[0], current_wp[1], current_wp[2], yaw=None)

            # 현재 드론 위치와 비교해 위치 도달했는지, 속도 안정했는지 확인
            if self._position_reached(current_wp):
                self._search_waypoint_index += 1
                if self._search_waypoint_index >= len(self._search_waypoints):
                    self._search_waypoint_index = 0

            return

        elif self._state == self.State.APPROACH:
            # (a) 이미 타겟을 잃으면 실패 처리 후 IDLE
            if target_lost:
                self.get_logger().info(f"Failed! Target lost during APPROACH")
                self._switch_to_finished(success=False)
                return

            # (b) 타겟 직상호(수평 위치), 미리 저장한 접근 고도 유지
            tx, ty, _ = self._tag_position
            tz = self._approach_altitude
            self._publish_position_setpoint(tx, ty, tz, yaw=None)

            # 드론 위치 정보가 들어온다면 도달 여부 확인
            if self._vehicle_position and self._position_reached([tx, ty, tz]):
                self._switch_state(self.State.DESCEND)
            return

        elif self._state == self.State.DESCEND:
            # (a) 타겟을 잃으면 실패 처리 후 IDLE
            if target_lost:
                self.get_logger().info(f"Failed! Target lost during DESCEND")
                self._switch_to_finished(success=False)
                return

            # (b) PI 제어로 XY 속도 산출, Z는 상수 하강
            vx, vy = self._calculate_velocity_setpoint_xy()
            vz = self._param_descent_vel  # 하강 속도 (m/s, 음수 방향이 Down인 경우 -로 바꿔주세요)

            # 태그의 현재 yaw를 사용 (orientation w,x,y,z 중 yaw만 뽑아서 퍼블리시)
            yaw = None
            if self._tag_orientation:
                # quaternion → yaw(라디안) 변환
                w, x, y, z = self._tag_orientation
                # tf2 대신 수식으로 yaw 계산 (NED frame에서)
                ysqr = y * y
                t0 = +2.0 * (w * z + x * y)
                t1 = +1.0 - 2.0 * (ysqr + z * z)
                yaw = math.atan2(t0, t1)

            self._publish_velocity_setpoint(vx, vy, vz, yaw=yaw)

            # 착륙 감지되면 성공 처리
            if self._land_detected:
                self._switch_to_finished(success=True)
            return

        elif self._state == self.State.FINISHED:
            # 일단 성공 혹은 실패 신호를 이미 보냈으므로 IDLE로 대기
            return

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
