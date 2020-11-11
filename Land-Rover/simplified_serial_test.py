#https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf

import serial
import time

saber2x25 = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
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
    if speed > 100 and speed < -100:
        print('A speed of (%s) is too high',%(speed))
    else
        speed = speed * 0.63
        motor = speed + 64
        motor = int(motor)
        saber2x25.write(motor)
    return

def m2(speed):
    ''' 
    
    '''
    if speed > 100 and speed < -100:
        print('A speed of (%s) is too high',%(speed))
    else
        speed = speed * 0.63
        motor = speed + 192
        motor = int(motor)
        saber2x25.write(motor)
    return

def move(speed):
    ''' 
    
    '''
    if speed > 100 and speed < -100:
        print('A speed of (%s) is too high',%(speed))
    else
        m1(speed)
        m2(speed)
    return

def turnLeft():
    '''

    '''
    m1(20)
    m2(-20)
    time.sleep(1)
    return

def turnRight():
    '''

    '''
    m1(-20)
    m2(20)
    time.sleep(1)
    return

def stop()
    '''

    '''
    m1(0)
    m2(0)
    return

def emergencyStop()
    '''
    (hex 0x00) is shut down both motors
    '''
    send = (hex)0x00
    saber2x25.write(send)
    return

#????MAIN_PROGRAM????#

serial.open()

move(10)
time.sleep(2)
stop()
turnRight()
turnLeft()
time.sleep(1)
emergencyStop()

serial.close()