#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#Global Variables --
full_altitude = 0
full_yaw = 0

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

def print_next_wp(nextwp):    
    missionitem=vehicle.commands[nextwp-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    print('Waypoint (%s)(%s)(%s)'% (lat,lon,alt))

def get_battery():
    battery_str = str(vehicle.battery)
    percent = battery_str[battery_str.find('level=')+6:len(battery_str)]
    percent_int = int(percent)
    #print(percent_int)
    return percent_int

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

#MISSION METHODS
def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def clear_drone_cmds():
    cmds = vehicle.commands
    print("Removing Automatic commands...")
    cmds.clear()
    cmds.upload()
    print("Commands cleared.")

def add_mission_point(aLatitude, aLongitude, aAltitude):
    '''
    Adds a point that to the mission that the drone will travel to.
    The waypoints are a direct derivative of the location that is sent to the function
    '''

    cmds = vehicle.commands
    
    #Defines the new waypoint to be added.
    print('Adding waypoint @ (%s, %s)' % (aLatitude,aLongitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, aLatitude, aLongitude, aAltitude))
    cmds.upload()

def modify_misson_points(aLatitude, aLongitude, aAltitude):
    cmds = vehicle.commands

    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, aLatitude, aLongitude, aAltitude)

    #Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

def read_add_waypoints():
    clear_drone_cmds()

    file_loc = open("locations.txt", "r")

    numwaypts = 0
    first = True
    for line in file_loc:
        line_split = line.split(',')

        if first:
            full_altitude = float(line_split[0])
            full_yaw = float(line_split[1])
            first = False
        else: 
            aLatitude = float(line_split[0])
            aLongitude = float(line_split[1])
            aAltitude = float(10)
            add_mission_point(aLatitude, aLongitude, full_altitude)
            numwaypts=numwaypts+1

        if 'str' in line:
            break

    print()
    file_loc.close()
    vehicle.commands.upload
    return numwaypts

#TAKEOFF & LAND
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
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def land_vehilce():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    '''
    while not vehicle.location.global_relative_frame.alt==0:
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
    vehicle.armed = False
    vehicle.close()
    '''

#DRONE CONTROL
def condition_yaw(heading = full_yaw, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,0,0,
        Vx,Vy,Vz,
        0,0,0,
        0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

#///MAIN MISSION///

print('CLEAR OLD MISSION')
clear_drone_cmds()

print('MISSION BEGIN')
time.sleep(2)

numwaypts = read_add_waypoints()

arm_and_takeoff(10)

print("Guiding Copter to mission points.")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

currentwaypoint=vehicle.commands.next

while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    print('Battery Level (%s)'%(get_battery()))
    
    if currentwaypoint!=nextwaypoint:
        print("Stablize Motor for Video")
        vehicle.mode = VehicleMode("GUIDED")
        condition_yaw(full_yaw)
        time.sleep(5)
        vehicle.mode = VehicleMode("BRAKE")
        time.sleep(7)
        #This is when RASPI CAMERA WILL TAKE VIDEO---
        vehicle.mode = VehicleMode("GUIDED")
        set_velocity_body(0,-1,0)
        time.sleep(2)
        vehicle.mode = VehicleMode("BRAKE")
        time.sleep(7)
        print("Video recorded")
        currentwaypoint=nextwaypoint
        vehicle.mode = VehicleMode("AUTO")

    if get_battery() <= 40:
        print("Exit! Warning - Low Battery")
    
    if nextwaypoint==(numwaypts-1) or get_battery() < 40: #Dummy waypoint - as soon as we reach final waypoint this is true and we exit.
        print("Exit 'Recon' mission and start heading to HOME location")
        break
    time.sleep(3)


print('Return to launch')
vehicle.mode = VehicleMode("RTL")

while vehicle.location.global_relative_frame.alt > 1:
    print('Battery Level (%s)\nImmediate Land @ (25)'%(get_battery()))
    if get_battery() < 25:
        land_vehilce()
        break
    time.sleep(3)

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
