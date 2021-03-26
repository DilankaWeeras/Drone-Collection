from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
baud_rate = 57600

vehicle = connect(connection_string, baud = baud_rate, wait_ready=True)

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Vehicle is now Armable")
    
    vehicle.armed=True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Vehicle is not armed.")
    return None

#MAIN

vehicle = connectToCopter()
arm()
vehicle.armed = False()
print("End of Script")