from __future__ import print_function
import argparse
from pymavlink import mavutil
import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

#from picamera import PiCamera, Color



# PRINT METHODS


def execute_aerial_drone():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = 'udpin:0.0.0.0:14550'
    baud_rate = 57600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    '''
    # Camera Setup --
    camera = PiCamera()
    camera.rotation = 0
    # max is (2592,1944) for pic / (1920,1080) for vid at 15fps
    camera.resolution = (2592,1944)
    camera.framerate = 15
    # Global Variables --
    #global full_altitude
    #full_altitude = 0
    #global full_yaw
    #full_yaw = 0
    '''
    try:
        mission_pts, full_altitude, full_yaw = read_add_waypoints()
        arm_and_takeoff(int(full_altitude), vehicle)
        home = vehicle.location.global_frame
        for wp in mission_pts:
            wp_threshold = 0.65
            print("Going to Point:\t" + str(wp[2]) + "_" + str(wp[3]))
            print("Going to location: " + str(wp[0]) + " " + str(wp[1]))
            point = LocationGlobalRelative(
                float(wp[0]), float(wp[1]), float(full_altitude))
            vehicle.simple_goto(point)
            vehicle.flush()

            wait_time = 0
            while distance_to_current_waypoint(wp[0], wp[1], full_altitude, vehicle) > wp_threshold:
                time.sleep(0.5)
                wait_time = wait_time + 0.5
                if wait_time > 20:
                    wp_threshold = wp_threshold + 0.25

            time.sleep(2)
            camera = 0
            take_pictures(wp[2], wp[3], full_yaw, vehicle, camera)

        time.sleep(1)
        print("Going Home...")
        vehicle.simple_goto(home)
        while distance_to_current_waypoint(home.lat, home.lon, full_altitude, vehicle) > 2:
            time.sleep(0.5)
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
    except:
        print("Unexpected error: Landing")
        vehicle.mode = VehicleMode("LAND")




def print_location():
    """
    prints out the current location of the drone to the terminal
    returns a string of the current location of the drone
    """
    original_location = vehicle.location.global_frame
    printout = "Location: Lat(" + original_location.lat + ") Lon(" + \
        original_location.lon + ") Alt(" + original_location.alt + ")"
    #print('Location: Lat(%s) Lon(%s) Alt(%s)'%(original_location.lat, original_location.lon, original_location.alt))
    print(printout)
    return printout

# LOCATION METHODS


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_current_waypoint(lat, lon, alt, vehicle):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(
        vehicle.location.global_frame, targetWaypointLocation)
    print("Distance to waypoint: " + str(distancetopoint))
    return distancetopoint

# MISSION


def clear_drone_cmds():
    cmds = vehicle.commands
    print("Removing Automatic commands...")
    cmds.clear()
    cmds.upload()
    print("Commands cleared.")


def read_add_waypoints():
    clear_drone_cmds()
    mission_pts = []

    print("Opening Locations")
    file_loc = open('locations.txt', 'r')

    lines = file_loc.readlines()
    line_split = lines[0].split(',')
    full_altitude = int(line_split[0])
    full_yaw = int(line_split[1])
    print("Altitude = " + str(full_altitude))
    print("Yaw = " + str(full_yaw))

    for line in lines[1:]:
        line_split = line.split(',')
        print("Adding waypoint at: " + line)
        line_split[0] = float(line_split[0])
        line_split[1] = float(line_split[1])
        line_split[2] = int(line_split[2])
        line_split[3] = int(line_split[3])
        mission_pts.append(line_split)

    file_loc.close()
    print(mission_pts)

    return mission_pts, full_altitude, full_yaw

# ARMING


def arm_and_takeoff(aTargetAltitude, vehicle):
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


def condition_yaw(vehicle, heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_velocity_body(vehicle, Vx, Vy, Vz):
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


def take_pictures(x, y, full_yaw, vehicle, camera):
    condition_yaw(vehicle, full_yaw)
    time.sleep(7)
    vehicle.mode = VehicleMode("BRAKE")
    time.sleep(1)
    '''
    camera.start_preview()
    time.sleep(3)
    for i in range(1, 6):
        camera.capture('/home/pi/Pictures/test3/' + str(x) +
                       '_' + str(y) + '_l' + str(i) + '.jpg')
    camera.stop_preview()
    '''
    vehicle.mode = VehicleMode("GUIDED")
    set_velocity_body(vehicle, 0, 1, 0)
    time.sleep(0.5)
    vehicle.mode = VehicleMode("BRAKE")
    time.sleep(1)
    '''
    camera.start_preview()
    time.sleep(3)
    for i in range(1, 6):
        camera.capture('/home/pi/Pictures/test3/' + str(x) +
                       '_' + str(y) + '_r' + str(i) + '.jpg')
    camera.stop_preview()
    '''
    vehicle.mode = VehicleMode("GUIDED")
    set_velocity_body(vehicle, 0, 0, 0)
# MAIN



'''
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = '/dev/ttyAMA0'
baud_rate = 57600

vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

# Camera Setup --
camera = PiCamera()
camera.rotation = 0
# max is (2592,1944) for pic / (1920,1080) for vid at 15fps
camera.resolution = (2592,1944)
camera.framerate = 15
# Global Variables --
global full_altitude
full_altitude = 0
global full_yaw
full_yaw = 0
mission_pts = []
'''




'''
try:
    read_add_waypoints()
    arm_and_takeoff(int(full_altitude))
    home = vehicle.location.global_frame
    for wp in mission_pts:
        wp_threshold = 0.65
        print("Going to Point:\t" + str(wp[2]) + "_" + str(wp[3]))
        print("Going to location: " + str(wp[0]) + " " + str(wp[1]))
        point = LocationGlobalRelative(
            float(wp[0]), float(wp[1]), float(full_altitude))
        vehicle.simple_goto(point)
        vehicle.flush()

        wait_time = 0
        while distance_to_current_waypoint(wp[0], wp[1], full_altitude) > wp_threshold:
            time.sleep(0.5)
            wait_time = wait_time + 0.5
            if wait_time > 20:
                wp_threshold = wp_threshold + 0.25

        time.sleep(2)
        take_pictures(wp[2], wp[3])

    time.sleep(1)
    print("Going Home...")
    vehicle.simple_goto(home)
    while distance_to_current_waypoint(home.lat, home.lon, full_altitude) > 2:
        time.sleep(0.5)
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")
except:
    print("Unexpected error: Landing")
    vehicle.mode = VehicleMode("LAND")
'''
execute_aerial_drone()