import RPi.GPIO as GPIO # importing GPIO
import time # importing time

print("Begin Program:")

print("Test GPIO4")
GPIO.setmode(GPIO.BCM) # importing as broadcom, can also import as .BOARD for boardnames
GPIO.setup(4, GPIO.OUT) # setting pin 18 to GPIO "output"

GPIO.output(4, GPIO.HIGH)
time.sleep(5)
GPIO.output(4, GPIO.LOW)
GPIO.cleanup()

print("End Test GPIO4")
time.sleep(1)

print("Blink GPIO4 rapid")

led = 4

GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)


for x in range(20):
	time.sleep(0.1)
	GPIO.output(led, GPIO.HIGH)
	time.sleep(0.1)
	GPIO.output(led, GPIO.LOW)

print("End Program")

GPIO.cleanup()



