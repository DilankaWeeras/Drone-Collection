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

vehicle = connect(connection_string, baud = baud_rate, wait_ready=True)

def arm_disarm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Vehicle is now Armable")
    
    vehicle.armed=True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(2)
    vehicle.armed = False

    print("Vehicle is not armed.")
    return None

#MAIN

arm_disarm()
vehicle.armed = False()
print("End of Script")