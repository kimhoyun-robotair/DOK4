#!/usr/bin/env python3
# Offboard 제어 기반 회전익 장거리 자율비행 테스트 코드
import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import (
    VehicleStatus, VehicleAttitude, VehicleCommand, OffboardControlMode,
    VehicleLocalPosition, VehicleGlobalPosition, TrajectorySetpoint
)

import geometry_utils
import vehicle_command

class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")
        # QoS and Publisher/Subscribe Initialization
        # Qos for PX4
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter("WP1_lat", 37.405071)
        self.declare_parameter("WP1_lon", 126.613924)
        self.declare_parameter("WP2_lat", 37.403633)
        self.declare_parameter("WP2_lon", 126.616342)
        self.declare_parameter("WP3_lat", 37.403001)
        self.declare_parameter("WP3_lon", 126.614233)
        self.declare_parameter("WP4_lat", 37.404870)
        self.declare_parameter("WP4_lon", 126.615856)

        wp1_lat = self.get_parameter("WP1_lat").value
        wp1_lon = self.get_parameter("WP1_lon").value
        wp2_lat = self.get_parameter("WP2_lat").value
        wp2_lon = self.get_parameter("WP2_lon").value
        wp3_lat = self.get_parameter("WP3_lat").value
        wp3_lon = self.get_parameter("WP3_lon").value
        wp4_lat = self.get_parameter("WP4_lat").value
        wp4_lon = self.get_parameter("WP4_lon").value

        # publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', self.qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)

        # subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, self.qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, self.qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.qos_profile)

        # WGS84 Waypoints
        self.wgs84_waypoints = {
            "WP0": {"lat": wp1_lat, "lon": wp1_lon, "alt": 0.0},
            "WP1": {"lat": wp1_lat, "lon": wp1_lon, "alt": -20.0},
            "WP2": {"lat": wp2_lat, "lon": wp2_lon, "alt": -20.0},
            "WP3": {"lat": wp3_lat, "lon": wp3_lon, "alt": -20.0},
            "WP4": {"lat": wp4_lat, "lon": wp4_lon, "alt": -20.0},
        }

        # NED Waypoints and Reference Point
        self.ned_waypoints = {}
        self.ref_lat = math.radians(self.wgs84_waypoints["WP0"]["lat"])
        self.ref_lon = math.radians(self.wgs84_waypoints["WP0"]["lon"])

        # WGS84 to NED Coordinate
        for wp_name, wp_data in self.wgs84_waypoints.items():
            lat = wp_data["lat"]
            lon = wp_data["lon"]
            alt = wp_data["alt"]
            x, y = geometry_utils.wgs84_to_ned(lat, lon, self.ref_lat, self.ref_lon)
            self.ned_waypoints[wp_name] = {"x": x, "y": y, "z": alt}
        self.get_logger().info(f"Pre-calculated waypoints in NED coordinates:\n{self.ned_waypoints}")

        # Variable Initialization
        self.state = "NOT_READY"
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.global_dist = 0.0
        self.takeoff_height = self.wgs84_waypoints["WP1"]["alt"]
        self.waypoint_reach_or_not = False
        self.height_reach_or_not = False

        self.transition_to_fc = False
        self.transition_to_mc = False

        self.threshold_range = 3.0
        self.threshold_height = abs(1.0)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("Configure complete.")
    
    # Callback Group ---------------------------------------------------
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback for vehicle attitude."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status."""
        self.vehicle_status = vehicle_status

    # Timer Callback ---------------------------------------------------
    def timer_callback(self):
        # Take-off 상태 처리
        if self.state == "NOT_READY":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
            self.offboard_setpoint_counter %= 11
            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                # self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP0"], self.ned_waypoints["WP1"])
                vehicle_command.engage_offboard_mode(
                    self.vehicle_command_publisher, self.get_clock())
            if self.offboard_setpoint_counter == 9:
                vehicle_command.arm(self.vehicle_command_publisher, self.get_clock())

            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.height_reach_or_not = geometry_utils.is_height_reached(self.vehicle_local_position.z, 
                                                                        self.takeoff_height, self.threshold_height)
            if (self.height_reach_or_not == True and
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.get_logger().info("WP1 REACHED")
                self.state = "WAYPOINT_1"

        elif self.state == "WAYPOINT_1":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP1"], self.ned_waypoints["WP2"])
            self.pos_x = self.ned_waypoints["WP2"]["x"]
            self.pos_y = self.ned_waypoints["WP2"]["y"]
            self.pos_z = self.ned_waypoints["WP2"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP2"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP2 REACHED")
                self.state = "WAYPOINT_2"

        elif self.state == "WAYPOINT_2":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP2"], self.ned_waypoints["WP3"])
            self.pos_x = self.ned_waypoints["WP3"]["x"]
            self.pos_y = self.ned_waypoints["WP3"]["y"]
            self.pos_z = self.ned_waypoints["WP3"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP3"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP3 REACHED")
                self.state = "WAYPOINT_3"

        elif self.state == "WAYPOINT_3":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP3"], self.ned_waypoints["WP4"])
            self.pos_x = self.ned_waypoints["WP4"]["x"]
            self.pos_y = self.ned_waypoints["WP4"]["y"]
            self.pos_z = self.ned_waypoints["WP4"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP4"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP4 REACHED")
                self.state = "WAYPOINT_4"

        elif self.state == "WAYPOINT_4":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP4"], self.ned_waypoints["WP1"])
            self.pos_x = self.ned_waypoints["WP1"]["x"]
            self.pos_y = self.ned_waypoints["WP1"]["y"]
            self.pos_z = self.ned_waypoints["WP1"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP1"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP1 REACHED")
                self.state = "LAND"

        elif self.state == "LAND":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            vehicle_command.land(self.vehicle_command_publisher, self.get_clock())
        

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()