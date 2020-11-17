# https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf

import serial
import time

class SaberControl:
    def __init__(self):
        print("Initializing Serial Port...")
        time.sleep(1)
        print("Baudrate set to 9600")
        
        self.saber2x25 = serial.Serial(
        port='/dev/ttyS0',  # Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate=9600
        # parity=serial.PARITY_NONE,
        # stopbits=serial.STOPBITS_ONE,
        # bytesize=serial.EIGHTBITS,
        # timeout=1
        )
        self.currentSpeed1 = 0
        self.currentSpeed2 = 0
    
    def m1(self, speed):
        ''' 
        Change the speed of Motor one - left Motor
        Min Speed -60
        Max Speed +60
        '''
        
        if speed > 60 or speed < -60:
            # print('A speed of (%s) is too high',%(speed))
            print("Speed is too high")
        else:
            motor = speed + 64
            send = chr(motor)
            print(send)
            self.saber2x25.write(send)
            self.currentSpeed1 = speed
        return

    def m2(self, speed):
        ''' 
        Change the speed of Motor two - right Motor
        Min Speed -60
        Max Speed +60
        '''
        if speed > 60 and speed < -60:
            # print('A speed of (%s) is too high',%(speed))
            print("Speed is too high")
        else:
            motor = speed + 192
            send = chr(motor)
            print(send)
            self.saber2x25.write(send)
            self.currentSpeed2 = speed
        return

    def instantMove(self, speed):
        ''' 
        Moves both the motors
        '''
        self.m1(speed)
        self.m2(speed)
        return

    def move(self, speed):
        ''' 
        Ramps up both the motors
        not availible while turning
        '''
        if self.currentSpeed1 > speed and self.currentSpeed2 > speed:
            while self.currentSpeed1 >= speed:
                self.instantMove(self.currentSpeed1-1)
                time.sleep(0.1)
        if self.currentSpeed1 < speed and self.currentSpeed2 < speed:
            while self.currentSpeed1 <= speed:
                self.instantMove(self.currentSpeed1+1)
                time.sleep(0.1)        
        return

    def turnLeft(self):
        '''
        Turns the Rover 90 degrees to the left
        '''
        self.instantStop()
        time.sleep(0.5)
        self.m1(30)
        self.m2(-30)
        time.sleep(0.5)
        self.instantStop()
        return

    def turnRight(self):
        '''
        Turns the Rover 90 degrees to the right
        '''
        self.instantStop()
        time.sleep(0.5)
        self.m1(-30)
        self.m2(30)
        time.sleep(0.5)
        self.instantStop()
        return

    def instantStop(self):
        '''
        Sets the speed of both motors to 0
        '''
        self.m1(0)
        self.m2(0)
        return
    
    def stop(self):
        '''
        Ramps down the motors to zero speed
        Not availible while turning
        '''
        self.moveForward(0)     
        return

    def emergencyStop():
        '''
        (chr 0x00) is shut down both motors
        '''
        send = chr(0x00)
        self.saber2x25.write(send)
        return
