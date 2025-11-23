#!/usr/bin/env python3
# DOK4 제어 코드 중 
import rclpy
import numpy as np
import math

import rclpy.executors
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from px4_msgs.msg import (
    VehicleStatus, VehicleAttitude, VehicleCommand, OffboardControlMode,
    VehicleLocalPosition, VehicleGlobalPosition, TrajectorySetpoint
)

import vehicle_command
import geometry_utils

class OffboardControl(LifecycleNode):
    def __init__(self):
        super().__init__("offboard_control")
        # QoS and Publisher/Subscribe Initialization
        self.qos_profile = None
        self.offboard_control_mode_publisher = None
        self.trajectory_setpoint_publisher = None
        self.vehicle_command_publisher = None
        self.readiness_publisher = None
        self.vehicle_local_position_subscriber = None
        self.vehicle_attitude_subscriber = None

    def on_configure(self, state : State) -> TransitionCallbackReturn:
        # Logging
        self.get_logger().info("Configuring Offboard Control Mode")

        # Qos for PX4
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # publisher
        self.offboard_control_mode_publisher = self.create_lifecycle_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_profile)
        self.trajectory_setpoint_publisher = self.create_lifecycle_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', self.qos_profile)
        self.vehicle_command_publisher = self.create_lifecycle_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        self.readiness_publisher = self.create_lifecycle_publisher(
            Bool, '/lifecycle_aruco_tracker/ready', qos_profile
        )
        
        # subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, self.qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, self.qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.qos_profile)

        # WGS84 Waypoints
        self.wgs84_waypoints = {
            "WP0": {"lat": 37.449187, "lon": 126.653021, "alt": -0.0},
            "WP1": {"lat": 37.449187, "lon": 126.653021, "alt": -20.0},
            "WP2": {"lat": 37.452717, "lon": 126.653195, "alt": -20.0},
            "WP3": {"lat": 37.454559, "lon": 126.657573, "alt": -20.0},
            "WP4": {"lat": 37.454490, "lon": 126.648858, "alt": -20.0},
            "WP5": {"lat": 37.452717, "lon": 126.653195, "alt": -20.0},
            "WP6": {"lat": 37.452059, "lon": 126.647888, "alt": -20.0},
            "WP7": {"lat": 37.452059, "lon": 126.647888, "alt": -5.0},
            "WP8": {"lat": 37.452059, "lon": 126.647888, "alt": -20.0},
            "WP9": {"lat": 37.451786, "lon": 126.659597, "alt": -20.0},
            "WP10": {"lat": 37.452717, "lon": 126.653195, "alt": -10.0},
            "WP11": {"lat": 37.449187, "lon": 126.653021, "alt": -5.0}
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

        self.get_logger().info("Configure complete.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activating Offboard Control Node")
        self.timer = self.create_timer(0.01, self.timer_callback)

        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.get_logger().info("Deactivating Offboard Control Node")
        self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info("on_cleanup() is called")
        self.destroy_publisher(self.offboard_control_mode_publisher)
        self.destroy_publisher(self.trajectory_setpoint_publisher)
        self.destroy_publisher(self.vehicle_command_publisher)
        self.get_logger().info("Cleanup Complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info("on_shutdown() is called")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS
    
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

    # Flight Control ----------------------------------------------------
    # def flight_control(self, current_waypoint)

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
            vehicle_command.transition_fw(self.vehicle_command_publisher, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP2"],
                                                                            self.threshold_range)
            self.get_logger().info("Transition to FW Mode")
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
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP4"], self.ned_waypoints["WP5"])
            self.pos_x = self.ned_waypoints["WP5"]["x"]
            self.pos_y = self.ned_waypoints["WP5"]["y"]
            self.pos_z = self.ned_waypoints["WP5"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP5"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP5 REACHED")
                self.state = "WAYPOINT_5"

        elif self.state == "WAYPOINT_5":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            self.pos_yaw = geometry_utils.get_attitude(self.ned_waypoints["WP5"], self.ned_waypoints["WP11"])
            self.pos_x = self.ned_waypoints["WP11"]["x"]
            self.pos_y = self.ned_waypoints["WP11"]["y"]
            self.pos_z = self.ned_waypoints["WP11"]["z"]
            
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
            vehicle_command.transition_mc(self.vehicle_command_publisher, self.get_clock())
            self.waypoint_reach_or_not = geometry_utils.is_waypoint_reached(self.vehicle_local_position.x,
                                                                            self.vehicle_local_position.y,
                                                                            self.vehicle_local_position.z,
                                                                            self.ned_waypoints["WP11"],
                                                                            self.threshold_range)
            if self.waypoint_reach_or_not == True:
                self.get_logger().info("WP11 REACHED")
                self.state = "WAYPOINT_11"

        elif self.state == "WAYPOINT_11":
            vehicle_command.publish_heartbeat_ob_pos_sp(
                self.offboard_control_mode_publisher, self.get_clock())
            msg = Bool()
            msg.data = True
            self.readiness_publisher.publish(msg)

            self.pos_yaw = 0
            self.pos_x = self.ned_waypoints["WP11"]["x"]
            self.pos_y = self.ned_waypoints["WP11"]["y"]
            self.pos_z = self.ned_waypoints["WP11"]["z"]
            vehicle_command.publish_position_setpoint(
                self.trajectory_setpoint_publisher, self.pos_x, self.pos_y, self.pos_z, self.pos_yaw, self.get_clock())
        

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    executors = rclpy.executors.MultiThreadedExecutor()
    executors.add_node(offboard_control)

    try:
        executors.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        offboard_control.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()