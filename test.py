###############################################################
# WGS84 좌표 기반으로 장거리 멀티로터 자율비행 (~5km)을 구현하는 코드 #
###############################################################
import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import VehicleCommand, VehicleGlobalPosition, VehicleStatus, VehicleAttitude
from pymavlink import mavutil

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

        # config 파일 활용을 위한 파라미터 선언 및 디폴트 값 지정
        self.declare_parameter("topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("topic_vehicle_attitude", "/fmu/out/vehicle_attitude")
        self.declare_parameter("topic_vehicle_status", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("topic_vehicle_global_position", "/fmu/out/vehicle_global_position")

        # 파라미터 파일을 읽어오기
        topic_vehicle_command = self.get_parameter("topic_vehicle_command").value
        topic_vehicle_attitude = self.get_parameter("topic_vehicle_attitude").value
        topic_vehicle_status = self.get_parameter("topic_vehicle_status").value
        topic_vehicle_global_position = self.get_parameter("topic_vehicle_global_position").value

        # Topic Publisher
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, topic_vehicle_command, qos_profile)

        # Topic Subscriber
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, topic_vehicle_attitude, self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, topic_vehicle_status, self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, topic_vehicle_global_position, self.vehicle_global_position_callback, qos_profile)

        # MAVLink Connection
        try:
            self.master = mavutil.mavlink_connection('udpin:localhost:14540', timeout=10)
            self.get_logger().info("MAVLink connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to establish MAVLink connection: {e}")
            raise

        self.pre_calculate_waypoint_and_initialize_variable()
        self.get_logger().info("Initialization complete.")

####################### Initialize and Coordinate Conversion Functions #######################
    def pre_calculate_waypoint_and_initialize_variable(self):
        """ Pre-calculate waypoints and ready vehicle """
        """ Calculate WGS84 Coordinate for using in MAVLink"""
        """ And Initialize various variables, Setting Timer Callback"""
        for i in range(len(self.waypoints) - 1):
            current_wp = self.waypoints[i]
            next_wp = self.waypoints[i + 1]
            bearing = self.calculate_bearing(current_wp["lat"], 
                                             current_wp["lon"], 
                                             next_wp["lat"], 
                                             next_wp["lon"])
            self.waypoints[i]["next_wp_bearing"] = bearing
            self.get_logger().info(f"Waypoint {i} to {i+1} bearing: {math.degrees(bearing)} degrees")

        self.current_position = None
        self.vehicle_status = None
        self.current_attitude= None
        self.flight_stage = "INIT"

        self.initial_takeoff_altitude = 10.0
        # initial takeoff altitude in meters
        self.final_altitude = self.waypoints[0]['alt']
        # final desired altitude in meters
        self.altitude_increment = 20.0
        # altitude increment in meters
        self.current_target_altitude = self.initial_takeoff_altitude
        
        self.yaw_adjustment_duration = 5.0 # duration for yaw adjustment in seconds
        self.yaw_adjustment_start_time = None

        self.current_waypoint = 0
        self.current_waypoint_index = 0
        self.current_heading = 0.0

        # 선회 반경과 선회 시간 지정
        self.loiter_radius = 30.0
        self.loiter_time = 10.0

        # 경로점 및 지정 고도 도달 판정 기준
        self.declare_parameter("threshold_range", 2.0)
        self.threshold_range = self.get_parameter("threshold_range").value

        self.declare_parameter("threshold_height", 0.3)
        self.threshold_height = self.get_parameter("threshold_height").value

        self.mission_item_count = 0
        self.waypoint_error = 0
        self.previous_seq = 0
        
        self.current_wp = self.waypoints[0]
        self.point = 0

        # 초기 YAW 계산을 위한 고정값 설정
        self.ref_lat = 35.069019
        self.ref_lon = 128.086364

        # timer 콜백 함수 설정   
        self.timer = self.create_timer(0.2, self.timer_callback)



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
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def land(self):
        """Send a command to land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def takeoff(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=float('nan'),  # Yaw
            param5=float('nan'),  # Latitude
            param6=float('nan'),  # Longitude
            param7=self.initial_takeoff_altitude
        )
        self.get_logger().info(f"Takeoff command sent, target altitude: {self.initial_takeoff_altitude}")

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def transition_fw(self):
        """Publish FW Transition Command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)
        self.get_logger().info("Transition to fixed-wing command sent")

    def transition_mc(self):
        """Publish MC Transition Command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)
        self.get_logger().info("Transition to multi-copter command sent")

    def adjust_yaw_and_altitude(self, target_yaw, target_altitude):
        target_yaw_deg = math.degrees(target_yaw)
        target_yaw_deg += math.degrees(self.current_heading)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
            param1=-1.0,  # Default ground speed
            param2=1,  # Bitmask: 0b0001 for custom yaw
            param3=0.0,  # Reserved
            param4=target_yaw_deg,  # Yaw in degrees
            param5=float('nan'),  # Latitude (use current)
            param6=float('nan'),  # Longitude (use current)
            param7=target_altitude
        )
        self.get_logger().info(f"Yaw and altitude adjustment command sent, target yaw: {target_yaw_deg} degrees, target altitude: {target_altitude}m")

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

    def upload_mission(self):
        self.get_logger().info("Starting mission upload...")

        if not self.wait_heartbeat():
            self.get_logger().error("Failed to receive heartbeat, aborting mission upload")
            return False

        try:
            self.get_logger().info("Clearing existing mission...")
            self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
            self.get_logger().info("Waiting for mission clear acknowledgement...")
            ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if not ack:
                self.get_logger().error("No acknowledgement received for mission clear")
                return False
            self.get_logger().info("Existing mission cleared")

            mission_items = []
            for i, wp in enumerate(self.waypoints):
                # Add normal waypoint
                mission_items.append(
                    self.master.mav.mission_item_int_encode(
                        self.master.target_system,
                        self.master.target_component,
                        len(mission_items),
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0, 0, 0, float('nan'),
                        int(wp['lat'] * 1e7), int(wp['lon'] * 1e7), wp['alt']
                    )
                )

                if wp['action'] == 'loiter' and i > 0:
                    next_wp = self.waypoints[i + 1]
                    prev_wp = self.waypoints[i - 1]
                    loiter_altitude = (0.25*wp['alt'] + 0.75*next_wp['alt'])
                    loiter_center = self.calculate_loiter_center(prev_wp, wp)
                    # Add loiter command
                    mission_items.append(
                        self.master.mav.mission_item_int_encode(
                            self.master.target_system,
                            self.master.target_component,
                            len(mission_items),
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                            0, 1,
                            self.loiter_time,  # Param 1: Loiter time in seconds
                            0,  # Param 2: Empty
                            self.loiter_radius,  # Param 3: Loiter radius in meters
                            1,  # Param 4: Forward moving aircraft this would be center exit, not used here
                            int(loiter_center['lat'] * 1e7), int(loiter_center['lon'] * 1e7), loiter_altitude
                        )
                    )

                    mission_items.append(
                        self.master.mav.mission_item_int_encode(
                            self.master.target_system,
                            self.master.target_component,
                            len(mission_items),
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1,
                            0, 0, 0, float('nan'),
                            int(wp['lat'] * 1e7), int(wp['lon'] * 1e7), loiter_altitude
                        )
                    )
            lat = self.waypoints[-1]['lat']
            lon = self.waypoints[-1]['lon']
            alt = self.waypoints[-1]['alt']
            mission_items.append(
                self.master.mav.mission_item_int_encode(
                    self.master.target_system,        # target system
                    self.master.target_component,     # target component
                    len(mission_items),               # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,                                # current (0: not the current item on upload)
                    1,                                # autocontinue (1로 주면 자동으로 다음으로 넘어갑니다)
                    0.0,                              # param1 (empty)
                    0.0,                              # param2 (empty)
                    0.0,                              # param3 (empty)
                    0.0,                              # param4 (empty)
                    int(lat * 1e7),                   # x: 위도
                    int(lon * 1e7),                   # y: 경도
                    float(alt)                        # z: 착륙 고도 (m)
                )
            )

            self.get_logger().info(f"Sending mission item count: {len(mission_items)}")
            self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(mission_items))

            for i in range(len(mission_items)):
                self.get_logger().info(f"Waiting for mission request {i}...")
                msg = self.master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=10)
                if msg is None:
                    self.get_logger().error(f"No mission request received for item {i}")
                    return False
                self.get_logger().info(f"Sending mission item {msg.seq}")
                self.master.mav.send(mission_items[msg.seq])

            self.get_logger().info("Waiting for mission acknowledgement...")
            msg = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if msg is None:
                self.get_logger().error("No mission acknowledgement received")
                return False
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                self.get_logger().info("Mission uploaded successfully")
                return True
            else:
                self.get_logger().error(f"Mission upload failed: {msg.type}")
                return False

            self.mission_item_count = len(mission_items)
            self.get_logger().info(f"Total mission items: {self.mission_item_count}")


        except Exception as e:
            self.get_logger().error(f"Exception during mission upload: {e}")
            return False

############################## Timer Callback Function #######################
# Functions for Finite State Machine and Sequential Offboard Control
    def timer_callback(self):
        if self.current_position is None or self.vehicle_status is None or self.current_attitude is None:
            self.get_logger().info("Waiting for vehicle data...")
            return

        if self.flight_stage == "INIT":
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            else:
                self.flight_stage = "UPLOAD_MISSION"

        elif self.flight_stage == "UPLOAD_MISSION":
            if self.upload_mission():
                self.flight_stage = "TAKEOFF"
            else:
                self.get_logger().error("Mission upload failed, retrying in next iteration")

        elif self.flight_stage == "TAKEOFF":
            self.takeoff()
            self.flight_stage = "WAIT_FOR_TAKEOFF"

        elif self.flight_stage == "WAIT_FOR_TAKEOFF":
            if self.current_position['alt'] >= self.initial_takeoff_altitude - self.threshold_height:  # 1m tolerance
                self.flight_stage = "ADJUST_YAW_AND_ALTITUDE"
                self.current_target_altitude = self.initial_takeoff_altitude + self.altitude_increment

        elif self.flight_stage == "ADJUST_YAW_AND_ALTITUDE":
            if self.current_target_altitude <= self.final_altitude:
                next_wp = self.waypoints[0]  # First waypoint
                target_yaw = self.calculate_bearing(
                    self.current_position["lat"], self.current_position["lon"],
                    next_wp["lat"], next_wp["lon"]
                )
                self.adjust_yaw_and_altitude(target_yaw, self.current_target_altitude)
                self.yaw_adjustment_start_time = self.get_clock().now()
                self.flight_stage = "WAIT_FOR_YAW_AND_ALTITUDE"
            else:
                self.flight_stage = "START_MISSION"

        elif self.flight_stage == "WAIT_FOR_YAW_AND_ALTITUDE":
            current_time = self.get_clock().now()
            if (current_time - self.yaw_adjustment_start_time).nanoseconds / 1e9 >= self.yaw_adjustment_duration:
                self.current_target_altitude += self.altitude_increment
                self.flight_stage = "ADJUST_YAW_AND_ALTITUDE"

        elif self.flight_stage == "START_MISSION":
            self.start_mission()
            self.flight_stage = "MONITOR_MISSION"

        elif self.flight_stage == "MONITOR_MISSION":
            msg = self.master.recv_match(type='MISSION_ITEM_REACHED', blocking=False)
            if msg:
                self.get_logger().info(f"Reached mission item {msg.seq-self.waypoint_error}")
                if int(msg.seq) == int(self.previous_seq) + 2:
                    self.waypoint_error += 2

                self.previous_seq = msg.seq
                self.current_wp = self.waypoints[msg.seq-self.waypoint_error]
                self.get_logger().info(f"Current waypoint action: {self.current_wp['action']}, transition: {self.current_wp['transition']}")

                # Handle transition
                if self.current_wp["transition"] == "transition_fw":
                    self.transition_fw()
                elif self.current_wp["transition"] == "transition_mc":
                    self.transition_mc()
                    if self.current_wp["action"] == "loiter" and self.point == 0:
                        self.waypoint_error += 1
                        self.point = 1

                    # Handle action
                if self.current_wp["action"] == "loiter":
                    self.get_logger().info(f"Starting loiter maneuver")
                elif self.current_wp["action"] == "land":
                    self.land()
                    self.get_logger().info("Landing. Mission completed.")
                    self.flight_stage = "MISSION_COMPLETE"

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