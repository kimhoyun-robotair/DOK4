#!/usr/bin/env python3
import time
import numpy as np
import math
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint

_MICRO = 1000000
_NSEC_TO_USEC = 1000

def _now_us(clock=None) -> int:
    """
    micro-seconds timestamp.
    • clock: rclpy.clock.Clock
    • None  : wall-clock(time.time())
    """
    if clock is not None:
        return int(clock.now().nanoseconds / _NSEC_TO_USEC)
    return int(time.time() * _MICRO)

def arm(pub, clock):
    """Send an arm command to the vehicle."""
    publish_vehicle_command(pub,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
        clock,
        param1=1.0)
    print('Arm command sent')

def disarm(pub, clock):
    """Send a disarm command to the vehicle."""
    publish_vehicle_command(pub,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
        clock,
        param1=0.0)
    print("Disarm command sent")

def engage_offboard_mode(pub, clock):
    """Send a command to engage offboard mode."""
    publish_vehicle_command(pub,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
        clock,
        param1=1.0, 
        param2=6.0)
    print("Switching to offboard mode")

def takeoff(pub, clock, initial_takeoff_altitude):
    publish_vehicle_command(
        pub,
        VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
        clock,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=float('nan'),  # Yaw
        param5=float('nan'),  # Latitude
        param6=float('nan'),  # Longitude
        param7=initial_takeoff_altitude
    )
    print(f"Takeoff command sent, \
            target altitude: {initial_takeoff_altitude}")

def land(pub, clock):
    """Send a command to land the vehicle."""
    publish_vehicle_command(
        pub, 
        VehicleCommand.VEHICLE_CMD_NAV_LAND,
        clock)
    print("Switching to land mode")

def publish_heartbeat_ob_pos_sp(pub, clock=None):
    """Publish heartbeat message for offboard control."""
    msg = OffboardControlMode()
    msg.position    = True
    msg.velocity    = False
    msg.acceleration= False
    msg.attitude    = False
    msg.body_rate   = False
    msg.timestamp   = _now_us(clock)
    pub.publish(msg)

def publish_position_setpoint(pub, x, y, z, yaw_d: float, clock=None):
    """Publish position setpoint."""
    msg = TrajectorySetpoint()
    msg.position    = [float(x), float(y), float(z)]
    msg.velocity    = [np.nan, np.nan, np.nan]
    msg.acceleration= [np.nan, np.nan, np.nan]
    msg.yaw         = np.clip(np.deg2rad(yaw_d), -np.pi, np.pi)
    msg.timestamp   = _now_us(clock)
    pub.publish(msg)

def publish_vehicle_command(pub, command, clock=None, **params):
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
    msg.timestamp = _now_us(clock)
    pub.publish(msg)

def transition_fw(pub, clock):
    """Publish FW Transition Command"""
    publish_vehicle_command(
        pub, 
        VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
        clock,
        param1=4.0)
    print("Transition to fixed-wing command sent")

def transition_mc(pub, clock):
    """Publish MC Transition Command"""
    publish_vehicle_command(
        pub, 
        VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
        clock,
        param1=3.0)
    print("Transition to multi-copter command sent")

def adjust_yaw_and_altitude(pub, clock, target_yaw, target_altitude, current_heading):
    target_yaw_deg = math.degrees(target_yaw)
    target_yaw_deg += math.degrees(current_heading)
    publish_vehicle_command(
        pub,
        VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
        clock,
        param1=-1.0,  # Default ground speed
        param2=1,  # Bitmask: 0b0001 for custom yaw
        param3=0.0,  # Reserved
        param4=target_yaw_deg,  # Yaw in degrees
        param5=float('nan'),  # Latitude (use current)
        param6=float('nan'),  # Longitude (use current)
        param7=target_altitude
    )
    print(f"Yaw and altitude adjustment command sent, \
            target yaw: {target_yaw_deg} degrees, \
            target altitude: {target_altitude}m")

def main():
    print("Test!")

if __name__=="__main__":
    main()
