#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PrecisionLandNode — revised 2025‑06‑20
=====================================
* PX4 Offboard 기반 AprilTag 정밀착륙 (SEARCH → APPROACH → DESCEND)
* 피드백 반영 사항
  - Heartbeat(Position/Velocity) 혼선 제거 → 상태별 단일 메시지 전송
  - 토픽명 PX4 v2.x(Harmonic) 기본 네임스페이스로 갱신
  - 타임아웃·적분 wind‑up·Yaw 처리 등 안정성 보강
  - Spiral 웨이포인트를 드론 **현재 위치 기준**으로 생성
  - 착륙 후 2 s 지연 뒤 Disarm (스핀다운 확보)
  - 코드 모듈화 & 파라미터화(변환 행렬, 적분 한계 등)

사용 전 확인 ⚠️
----------------
1. 카메라 → 바디 변환 `transform_camera_to_local()` 를 기체 마운트
   방향에 맞게 수정하거나 YAML 파라미터로 주입하세요.
2. '/fmu' 접두어는 PX4‑ROS2 bridge 기본값입니다. 다른 네임스페이스
   를 쓰면 `FMU_NS` 파라미터로 덮어쓰면 됩니다.
"""

import math
from enum import Enum
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleLandDetected,
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleStatus,
    VehicleCommand,
)


class PrecisionLandNode(Node):
    def __init__(self):
        super().__init__("precision_land_node")

        # ──────────────────────────────────────────────
        # 1) 파라미터 선언 ▸ YAML 로 덮어쓰기 가능
        # ──────────────────────────────────────────────
        p = self.declare_parameter  # alias
        p("descent_vel", 1.0)               # m/s (NED Down → 양수)
        p("vel_p_gain", 1.5)
        p("vel_i_gain", 0.0)
        p("max_velocity", 2.5)              # xy 최대 속도 제한
        p("integral_limit", 3.0)            # ∑error 제한 (m·s)
        p("target_timeout", 3.0)            # s
        p("marker_xy_tol", 0.1)             # m (수평 수렴 조건)
        p("altitude_tol", 0.1)              # m
        p("hb_initial_count", 10)           # Heartbeat 선행 횟수
        p("spiral_radius", 2.0)             # m (SEARCH 최대 반경)
        p("spiral_layer", 0.5)              # m (Z spacing)
        p("spiral_pts", 16)                 # pts/layer
        p("fmu_ns", "/fmu")                # FMU 네임스페이스 접두어

        # (보조) 카메라→바디 변환 행렬(열전달) [3×3] row‑major
        p("cam_transform", [1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0])

        # 읽기 단축
        gp = self.get_parameter
        self._param_descent_vel = gp("descent_vel").value
        self._param_vel_p_gain = gp("vel_p_gain").value
        self._param_vel_i_gain = gp("vel_i_gain").value
        self._param_max_velocity = gp("max_velocity").value
        self._param_integral_lim = gp("integral_limit").value
        self._param_target_timeout = gp("target_timeout").value
        self._param_marker_xy_tol = gp("marker_xy_tol").value
        self._param_altitude_tol = gp("altitude_tol").value
        self._param_hb_initial_count = gp("hb_initial_count").value
        self._spiral_radius = gp("spiral_radius").value
        self._spiral_layer = gp("spiral_layer").value
        self._spiral_pts = gp("spiral_pts").value
        self._fmu_ns = gp("fmu_ns").value.rstrip("/")
        self._R_cam2ned = np.array(gp("cam_transform").value, dtype=float).reshape(3, 3)

        # ──────────────────────────────────────────────
        # 2) 내부 상태 변수
        # ──────────────────────────────────────────────
        self._tag_position: Optional[List[float]] = None  # [x, y, z] (NED)
        self._tag_orientation: Optional[List[float]] = None
        self._tag_timestamp: Optional[Time] = None

        self._vehicle_position: Optional[List[float]] = None
        self._vehicle_velocity: Optional[List[float]] = None
        self._vehicle_attitude: Optional[List[float]] = None
        self._land_detected: bool = False

        self._hb_sent_count = 0
        self._mode_engaged = False

        self._search_waypoints: List[List[float]] = []
        self._search_index = 0
        self._search_started = False
        self._approach_altitude = 0.0

        self._vel_x_int = 0.0
        self._vel_y_int = 0.0

        class State(Enum):
            IDLE = 0
            SEARCH = 1
            APPROACH = 2
            DESCEND = 3
            FINISHED = 4

        self.State = State
        self._state = State.SEARCH

        # ──────────────────────────────────────────────
        # 3) QoS & 토픽 설정
        # ──────────────────────────────────────────────
        qos_px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST,
                              depth=1)

        # ▸ AprilTag pose (카메라 프레임)
        self.create_subscription(PoseStamped, "/apriltag_pose",
                                 self._target_pose_cb, 10)

        # ▸ PX4 → ROS2 state
        self.create_subscription(VehicleLocalPosition,
                                 f"{self._fmu_ns}/vehicle_local_position",
                                 self._veh_local_pos_cb, qos_px4)
        self.create_subscription(VehicleAttitude,
                                 f"{self._fmu_ns}/vehicle_attitude",
                                 self._veh_attitude_cb, qos_px4)
        self.create_subscription(VehicleLandDetected,
                                 f"{self._fmu_ns}/vehicle_land_detected",
                                 self._veh_land_cb, qos_px4)

        # ▸ publishers
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{self._fmu_ns}/in/trajectory_setpoint",
            qos_px4,
        )
        self._ob_ctrl_pub = self.create_publisher(
            OffboardControlMode,
            f"{self._fmu_ns}/in/offboard_control_mode",
            qos_px4,
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            f"{self._fmu_ns}/in/vehicle_command",
            qos_px4,
        )

        # ──────────────────────────────────────────────
        # 4) 타이머 (20 Hz)
        # ──────────────────────────────────────────────
        self.create_timer(0.05, self._update_cb)  # 20 Hz ↔ 0.05 s

        self.get_logger().info("PrecisionLandNode ready → SEARCH state")

    # ==============================================================
    #   콜백들
    # ==============================================================
    def _target_pose_cb(self, msg: PoseStamped):
        """AprilTag Pose → NED 오프셋으로 변환"""
        cam_vec = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z], dtype=float)
        ned_vec = self._R_cam2ned @ cam_vec  # 3×3 행렬곱
        self._tag_position = ned_vec.tolist()
        self._tag_orientation = [msg.pose.orientation.w,
                                 msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z]
        try:
            self._tag_timestamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self._tag_timestamp = self.get_clock().now()

    def _veh_local_pos_cb(self, msg: VehicleLocalPosition):
        self._vehicle_position = [msg.x, msg.y, msg.z]
        self._vehicle_velocity = [msg.vx, msg.vy, msg.vz]

    def _veh_attitude_cb(self, msg: VehicleAttitude):
        self._vehicle_attitude = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]

    def _veh_land_cb(self, msg: VehicleLandDetected):
        self._land_detected = bool(msg.landed)

    # ==============================================================
    #   Offboard helper – VehicleCommand
    # ==============================================================
    def _publish_vehicle_cmd(self, cmd_id: int, param1: float = 0.0, param2: float = 0.0):
        cmd = VehicleCommand()
        cmd.command = cmd_id
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        self._cmd_pub.publish(cmd)

    def arm(self):
        self._publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("[CMD] Arm")

    def disarm(self):
        self._publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("[CMD] Disarm")

    def engage_offboard(self):
        self._publish_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("[CMD] Switch to OFFBOARD")

    # ==============================================================
    #   Heartbeat (OffboardControlMode)
    # ==============================================================
    def _heartbeat(self, pos_ctrl: bool):
        msg = OffboardControlMode()
        msg.position = bool(pos_ctrl)
        msg.velocity = not pos_ctrl
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self._ob_ctrl_pub.publish(msg)

    # ==============================================================
    #   Setpoint helpers
    # ==============================================================
    def _pub_position_sp(self, x: float, y: float, z: float, yaw_deg: Optional[float]):
        sp = TrajectorySetpoint()
        sp.position = [x, y, z]
        sp.velocity = [math.nan] * 3
        sp.acceleration = [math.nan] * 3
        sp.yaw = math.radians(yaw_deg) if yaw_deg is not None else math.nan
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self._traj_pub.publish(sp)

    def _pub_velocity_sp(self, vx: float, vy: float, vz: float):
        sp = TrajectorySetpoint()
        sp.position = [math.nan] * 3
        sp.velocity = [vx, vy, vz]
        sp.acceleration = [math.nan] * 3
        sp.yaw = math.nan  # yaw hold
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self._traj_pub.publish(sp)

    # ==============================================================
    #   상태 머신 업데이트 (20 Hz)
    # ==============================================================
    def _update_cb(self):
        # ▸ Heartbeat (상태에 맞춰 한 번만)
        self._heartbeat(self._state in (self.State.SEARCH, self.State.APPROACH))
        self._hb_sent_count += 1

        # ▸ Offboard 모드 & Arm 진입
        if (not self._mode_engaged) and (self._hb_sent_count > self._param_hb_initial_count):
            self.engage_offboard()
            self.arm()
            self._mode_engaged = True

        # ▸ 상태 분기
        timeout = self._is_target_timeout()
        if self._state == self.State.SEARCH:
            self._run_search(timeout)
        elif self._state == self.State.APPROACH:
            self._run_approach(timeout)
        elif self._state == self.State.DESCEND:
            self._run_descend(timeout)
        # FINISHED/IDLE → no‑op

    # ==============================================================
    #   상태 로직
    # ==============================================================
    def _run_search(self, target_lost: bool):
        if not target_lost and self._tag_position is not None:
            # 태그 발견 ▸ APPROACH
            if self._vehicle_position:
                self._approach_altitude = self._vehicle_position[2]
            self._switch_state(self.State.APPROACH)
            return

        if not self._search_started:
            self._make_spiral_waypoints()
            self._search_started = True

        wp = self._search_waypoints[self._search_index]
        self._pub_position_sp(*wp, yaw_deg=None)

        if self._has_reached_xy(wp[:2]):
            self._search_index = (self._search_index + 1) % len(self._search_waypoints)

    def _run_approach(self, target_lost: bool):
        if target_lost:
            self.get_logger().warning("[APPROACH] Tag lost → abort")
            self._finish(False)
            return

        tx, ty, _ = self._tag_position
        tz = self._approach_altitude
        self._pub_position_sp(tx, ty, tz, yaw_deg=None)

        if self._has_reached_xy([tx, ty]):
            self._switch_state(self.State.DESCEND)

    def _run_descend(self, target_lost: bool):
        if target_lost:
            self.get_logger().warning("[DESCEND] Tag lost → abort")
            self._finish(False)
            return

        vx, vy = self._pid_xy()
        vz = abs(self._param_descent_vel)  # NED Down +
        self._pub_velocity_sp(vx, vy, vz)

        if self._land_detected:
            self._finish(True)

    # ==============================================================
    #   유틸리티 함수
    # ==============================================================
    def _is_target_timeout(self) -> bool:
        if self._tag_timestamp is None:
            return True
        elapsed: Duration = self.get_clock().now() - self._tag_timestamp
        return elapsed.nanoseconds * 1e-9 > self._param_target_timeout

    def _has_reached_xy(self, tgt_xy):
        if self._vehicle_position is None:
            return False
        dx = tgt_xy[0] - self._vehicle_position[0]
        dy = tgt_xy[1] - self._vehicle_position[1]
        return math.hypot(dx, dy) < self._param_marker_xy_tol

    def _pid_xy(self):
        if self._vehicle_position is None or self._tag_position is None:
            return 0.0, 0.0
        ex = self._vehicle_position[0] - self._tag_position[0]
        ey = self._vehicle_position[1] - self._tag_position[1]
        # P
        vx = ex * self._param_vel_p_gain
        vy = ey * self._param_vel_p_gain
        # I
        self._vel_x_int += ex
        self._vel_y_int += ey
        lim = self._param_integral_lim
        self._vel_x_int = max(-lim, min(lim, self._vel_x_int))
        self._vel_y_int = max(-lim, min(lim, self._vel_y_int))
        vx += self._vel_x_int * self._param_vel_i_gain
        vy += self._vel_y_int * self._param_vel_i_gain
        # clip
        mv = self._param_max_velocity
        return (max(-mv, min(mv, -vx)),  # 부호 ‑ Tag 방향(+)
                max(-mv, min(mv, -vy)))

    def _make_spiral_waypoints(self):
        x0, y0 = (self._vehicle_position[:2] if self._vehicle_position else [0.0, 0.0])
        z0 = self._vehicle_position[2] if self._vehicle_position else 0.0
        max_r = self._spiral_radius
        dz = self._spiral_layer
        npt = int(self._spiral_pts)
        waypoints = []
        radius = 0.0
        for i in range(npt):
            angle = 2 * math.pi * i / npt
            waypoints.append([x0 + radius * math.cos(angle),
                               y0 + radius * math.sin(angle),
                               z0])
            radius += max_r / npt
        self._search_waypoints = waypoints
        self._search_index = 0
        self.get_logger().info(f"[SEARCH] {len(waypoints)} spiral points generated")

    def _switch_state(self, new_state: Enum):
        self._state = new_state
        self.get_logger().info(f"[STATE] → {new_state.name}")

    def _finish(self, success: bool):
        self._switch_state(self.State.FINISHED)
        msg = "SUCCESS" if success else "FAILURE"
        self.get_logger().info(f"[RESULT] Precision Landing {msg}")
        # 2 s 후 Disarm (모터 스핀다운 확보)
        self.create_timer(2.0, lambda: self.disarm(), callback_group=None, oneshot=True)


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


if __name__ == "__main__":
    main()
