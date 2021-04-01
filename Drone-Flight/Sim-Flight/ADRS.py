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

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

#MISSION
def clear_drone_cmds():
    cmds = vehicle.commands
    print("Removing Automatic commands...")
    cmds.clear()
    cmds.upload()
    print("Commands cleared.")

def read_add_waypoints():
    clear_drone_cmds()

    file_loc = open('locations.txt', 'r')
    line_split = file_loc[0].split(',')
    full_altitude = line_split[0]
    full_yaw = line_split[1]
    line_split = file_loc[1].split(',')
    rows = line_split[0]
    cols = line_split[1]

    for line in file_loc[2:]:
        mission_pts.append(line)
    
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
    #vehicle.flush()

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
# MAIN


try:
    read_add_waypoints()



    
except:
    print("Unexpected error: Landing")
    vehicle.mode = VehicleMode("LAND")
