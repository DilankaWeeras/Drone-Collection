#https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf

import serial
import time

WAIT_TIME = 0.01

saber2x25 = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600
        #parity=serial.PARITY_NONE,
        #stopbits=serial.STOPBITS_ONE,
        #bytesize=serial.EIGHTBITS,
        #timeout=1
)

'''
Sabertooth controls two motors with one 8 byte 
MOTOR1 : 1 is full reverse, 64 is stop and 127 is full forward.
MOTOR2 : 128 is full reverse, 192 is stop and 255 is full forward.
Character 0 (hex 0x00) Sending this character will shut down both motors. 
'''

def m1(speed):
    ''' 
    
    '''
    if speed > 60 and speed < -60:
        #print('A speed of (%s) is too high',%(speed))
	    print("Speed is too high")
    else:
        motor = speed + 64
        send = hex(motor)
        print(send)
        saber2x25.write(send)
    return

def m2(speed):
    ''' 
    
    '''
    if speed > 60 and speed < -60:
        #print('A speed of (%s) is too high',%(speed))
        print("Speed is too high")
    else:
        motor = speed + 192
        send = hex(motor)
        print(send)
        saber2x25.write(send)
    return

def move(speed):
    ''' 
    Moves both the motors
    '''
    m1(speed)
    time.sleep(WAIT_TIME)
    m2(speed)
    return

def turnLeft():
    '''

    '''
    m1(5)
    time.sleep(WAIT_TIME)
    m2(-5)
    time.sleep(0.5)
    stop()
    return

def turnRight():
    '''

    '''
    m1(-5)
    time.sleep(WAIT_TIME)
    m2(5)
    time.sleep(0.5)
    stop()
    return

def stop():
    '''

    '''
    m1(0)
    time.sleep(WAIT_TIME)
    m2(0)
    time.sleep(WAIT_TIME)
    #saber2x25.write(64)
    #saber2x25.write(192)

    return

def emergencyStop():
    '''
    (hex 0x00) is shut down both motors
    '''
    send = hex(64)
    saber2x25.write(send)
    time.sleep(WAIT_TIME)
    send = hex(192)
    saber2x25.write(send)
    return

#????MAIN_PROGRAM????#

#serial.open()

move(5)
time.sleep(1)
stop()
turnRight()
turnLeft()
time.sleep(1)
stop()
