from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

from picamera import PiCamera, Color

import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = '/dev/ttyAMA0'
baud_rate = 57600

vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

camera = PiCamera()
camera.rotation = 0
# max is (2592,1944) for pic / (1920,1080) for vid at 15fps
camera.resolution = (1920, 1080)
camera.framerate = 15


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        Vx, Vy, Vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
# MAIN


try:
    arm_and_takeoff(10)

    vehicle.mode = VehicleMode("GUIDED")

    time.sleep(2)

    camera.start_preview()

    time.sleep(3)
    for x in range(1, 6):
        camera.capture('/home/pi/Pictures/test2/0_0_r' + str(x) + '.jpg')
    camera.stop_preview()

    vehicle.mode = VehicleMode("GUIDED")
    set_velocity_body(0, 1, 0)
    time.sleep(0.5)
    vehicle.mode = VehicleMode("BRAKE")

    camera.start_preview()

    time.sleep(3)
    for x in range(1, 6):
        camera.capture('/home/pi/Pictures/test2/0_0_r' + str(x) + '.jpg')
    camera.stop_preview()

    vehicle.mode = VehicleMode("GUIDED")
    set_velocity_body(0, -1, 0)
    time.sleep(4)
    vehicle.mode = VehicleMode("BRAKE")

    vehicle.mode = VehicleMode("LAND")
    print("End of Script")
except:
    print("Unexpected error: Landing")
    vehicle.mode = VehicleMode("LAND")
