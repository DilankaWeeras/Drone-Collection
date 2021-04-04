from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

#from picamera import PiCamera, Color

import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = 'udpin:0.0.0.0:14550'
baud_rate = 57600

vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

#Camera Setup --
'''
camera = PiCamera()
camera.rotation = 0
# max is (2592,1944) for pic / (1920,1080) for vid at 15fps
camera.resolution = (1920, 1080)
camera.framerate = 15
'''
#Global Variables --
full_altitude = 0
full_yaw = 0
rows = 0
cols = 0
mission_pts = []

#PRINT METHODS
def print_location():
    """
    prints out the current location of the drone to the terminal
    returns a string of the current location of the drone
    """
    original_location = vehicle.location.global_frame
    printout = "Location: Lat(" + original_location.lat + ") Lon(" + original_location.lon + ") Alt(" + original_location.alt + ")"
    #print('Location: Lat(%s) Lon(%s) Alt(%s)'%(original_location.lat, original_location.lon, original_location.alt))
    print(printout)
    return printout

#LOCATION METHODS
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint(lat, lon, alt):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    print(distancetopoint)
    return distancetopoint

#MISSION
def clear_drone_cmds():
    cmds = vehicle.commands
    print("Removing Automatic commands...")
    cmds.clear()
    cmds.upload()
    print("Commands cleared.")

def read_add_waypoints():
    #clear_drone_cmds()

    print("Opening Locations")
    file_loc = open('locations.txt', 'r')
    
    lines = file_loc.readlines()
    line_split = lines[0].split(',')
    full_altitude = int(line_split[0])
    full_yaw = int(line_split[1])
    print("Altitude = " + str(full_altitude))
    print("Yaw = " + str(full_yaw))
    line = file_loc.readline()
    line_split = lines[1].split(',')
    rows = int(line_split[0])
    cols = (line_split[1])
    print("Rows = " + str(rows))
    print("Colums = " + str(cols))

    for line in lines[2:]:
        line_split = line.split(',')
        print("Adding waypoint at: " + line)
        line_split[0] = float(line_split[0])
        line_split[1] = float(line_split[1])
        mission_pts.append(line_split)
    
    file_loc.close()
    print(mission_pts)

#ARMING
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

#DRONE CONTROL# #MAVLINK#
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

#CAMERA#
def take_pictures(x, y):
    vehicle.mode = VehicleMode("BRAKE")
    time.sleep(3)
    '''
    camera.start_preview()

    time.sleep(3)
    for i in range(1, 6):
        camera.capture('/home/pi/Pictures/test3/' + str(x) +
                       '_' + str(y) + '_l' + str(i) + '.jpg')
    camera.stop_preview()
    '''

    vehicle.mode = VehicleMode("GUIDED")
    set_velocity_body(0, 1, 0)
    time.sleep(0.5)
    vehicle.mode = VehicleMode("BRAKE")
    time.sleep(3)
    '''
    camera.start_preview()

    time.sleep(3)
    for x in range(1, 6):
        camera.capture('/home/pi/Pictures/test3/' + str(x) +
                       '_' + str(y) + '_r' + str(i) + '.jpg')
    camera.stop_preview()
    vehicle.mode = VehicleMode("GUIDED")
    '''
    set_velocity_body(0, 0, 0)
# MAIN


try:
    read_add_waypoints()
    arm_and_takeoff(full_altitude)
    home = vehicle.location.global_frame
    print_location()
    r = 0
    c = 0
    for wp in mission_pts:
        print("Going to Point:" + str(r) + "_" +str(c))
        print("Going to location" + str(wp[0]) + str(wp[1]))
        point = Location(wp[0], wp[1], full_altitude, is_relative=True)
        vehicle.commands.goto(point)
        vehicle.flush()

        while distance_to_current_waypoint(wp[0], wp[1], full_altitude) > 0.5:
            time.sleep(0.5)

        take_pictures()

        c = c+1
        if c = cols:
            r = r+1
            c = 0
        if r = rows:
            break
    time.sleep(1)
    print("Going Home...")
    vehicle.commands.goto(home)
except:
    print("Unexpected error: Landing")
    vehicle.mode = VehicleMode("LAND")
