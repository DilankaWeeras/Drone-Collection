#Libraries
import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD/BCM)
GPIO.setmode(GPIO.BCM)

#Set GPIO Pins
t1 = 2
t2 = 3
t3 = 4
t4 = 17
t5 = 27

e1 = 5
e2 = 6
e3 = 13
e4 = 19
e5 = 26

#set GPIO direction
GPIO.setup(t1, GPIO.OUT)
GPIO.setup(t2, GPIO.OUT)
GPIO.setup(t3, GPIO.OUT)
GPIO.setup(t4, GPIO.OUT)
GPIO.setup(t5, GPIO.OUT)

GPIO.setup(e1, GPIO.IN)
GPIO.setup(e2, GPIO.IN)
GPIO.setup(e3, GPIO.IN)
GPIO.setup(e4, GPIO.IN)
GPIO.setup(e5, GPIO.IN)

def dist_sense_1(trig_pin, echo_pin):
    # set Trigger to HIGH
    GPIO.output(trig_pin, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(echo_pin) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(echo_pin) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

if __name__ == '__main__':
    try:
        while True:
            dist = dist_sense_1(t1, e1)
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()