# https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf

import serial
import time

saber2x25 = serial.Serial(
    port='/dev/ttyS0',  # Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate=9600
    # parity=serial.PARITY_NONE,
    # stopbits=serial.STOPBITS_ONE,
    # bytesize=serial.EIGHTBITS,
    # timeout=1
)

'''
Sabertooth controls two motors with one 8 byte 
MOTOR1 : 1 is full reverse, 64 is stop and 127 is full forward.
MOTOR2 : 128 is full reverse, 192 is stop and 255 is full forward.
Character 0 (chr 0x00) Sending this character will shut down both motors. 
'''


def m1(speed):
    ''' 

    '''
    if speed > 60 and speed < -60:
        # print('A speed of (%s) is too high',%(speed))
        print("Speed is too high")
    else:
        motor = speed + 64
        send = chr(motor)
        print(send)
        saber2x25.write(send)
    return


def m2(speed):
    ''' 

    '''
    if speed > 60 and speed < -60:
        # print('A speed of (%s) is too high',%(speed))
        print("Speed is too high")
    else:
        motor = speed + 192
        send = chr(motor)
        print(send)
        saber2x25.write(send)
    return


def move(speed):
    ''' 
    Moves both the motors
    '''
    m1(speed)
    m2(speed)
    return


def turnLeft():
    '''

    '''
    m1(30)
    m2(-30)
    time.sleep(0.5)
    stop()
    return


def turnRight():
    '''

    '''
    m1(-30)
    m2(30)
    time.sleep(0.5)
    stop()
    return


def stop():
    '''

    '''
    m1(0)
    m2(0)
    return


def emergencyStop():
    '''
    (chr 0x00) is shut down both motors
    '''
    send = chr(0x00)
    saber2x25.write(send)
    return

#????MAIN_PROGRAM????#

move(40)
time.sleep(0.5)
stop()
turnRight()
turnLeft()
time.sleep(1)
stop()
stop()
emergencyStop()