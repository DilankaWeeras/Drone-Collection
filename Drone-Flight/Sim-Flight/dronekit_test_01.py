from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

#-- Connect to vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, baud=57600, wait_ready = True)

#-- Define the dunction for takeoff
def arm_and_takeoff(tgt_altitude) :
    print("Arming motors")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)

    #--wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt

        if altitude >= tgt_altitude -1:
            print("Altitude Reached")
            break

        time.sleep(1)

#---------MAIN Program---------
arm_and_takeoff(10)

#-- set the default speed
vehicle.airspeed = 7
'''
#--Go to wp1
print("go to wp1")
wp1 = LocationGlobalRelative(35.9872609, -95.8753037, 10)

vehicle.simple_goto(wp1)

#--- Do picture funcitons
time.sleep(30)
'''
#--- Return to initial Location
print("Coming back")
vehicle.mode = VehicleMode("LAND")

time.sleep(20)

#--Close Connection

