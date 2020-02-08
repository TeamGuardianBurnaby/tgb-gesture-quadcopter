#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

vehicle = connect(connection_string, wait_ready=True)

"""
Convenience functions for sending immediate/guided mode commands to control the Copter.

The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.


The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""



def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.


    The direction of movement can be relative to the drone's current heading, or relative to actual compass North, South,
    East, West directions. The frame of reference can be changed within send_ned_velocity:
    MAV_FRAME_BODY_OFFSET_NED - makes velocity component relative to drone's current heading (Currently used)
                                ex. North = front of drone, South = back of drone, East = right, West = left
    MAV_FRAME_LOCAL_NED - velocity component is relative to actual North, South, East, West directions
                            ex. if component is facing south, but velocity component is x m/s East, drone flies East
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


# Start of TGB move function

print("10-4...")


"""
assuming movement will be within boundaries for simulation, vehicle moves at indicated speed
in indicated direction for a a certain number seconds

x > 0 => fly North
x < 0 => fly South
y > 0 => fly East
y < 0 => fly West
z < 0 => ascend
z > 0 => descend
"""


def vmove(x, y, z, flightduration, inboundary=True):
    if inboundary:
        send_ned_velocity(x, y, z, flightduration)
        send_ned_velocity(0, 0, 0, 1)
    else:
        print("Invalid movement")


# example
print("Advance @ 5 m/s for 20 seconds")
vmove(5, 0, 0, 20)

print("Reverse @ 5 m/s for 20 seconds")
vmove(5, 0, 0, 20)

print("Strafe right @ .5 m/s for 30 seconds")
vmove(0, 0.5, 0, 30)

print("Increase Altitude @ .5 m/s for 10 seconds")
vmove(0, 0, 0.5, 10)

print("Reverse @ 10 m/s for 5 seconds")
vmove(-10, 0, 0, 5)

print("Arrived at destination")
