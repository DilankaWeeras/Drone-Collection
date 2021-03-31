from picamera import PiCamera, Color
import time

camera = PiCamera()
camera.resolution = (2592,1944)
count = 0

camera.start_preview()

    time.sleep(2)
    for x in range(1,5)
        camera.capture('/home/pi/Pictures/test2/0_0_r' + x +'.jpg')

    camera.stop_preview()
