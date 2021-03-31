from picamera import PiCamera, Color
import time

camera = PiCamera()
camera.resolution = (2592, 1944)
count = 0

try:
    time.sleep(3)
    camera.start_preview()
    for x in range(1, 6):
        camera.capture('/home/pi/Pictures/test1/0_0_r' + str(x) + '.jpg')
    camera.stop_preview()
except:
    print("Errors Found")
