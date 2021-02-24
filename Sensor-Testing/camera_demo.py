from picamera import PiCamera, Color
import time

camera = PiCamera()

#to start Preview
#camera must preview to adjust for light levels

camera.start_preview()
time.sleep(5)
camera.stop_preview()

#to rotate camera

camera.rotation = 180


# additional functionality
camera.resolution = (1920,1080) #max is (2592,1944) for pic / (1920,1080) for vid at 15fps
camera.framerate = 15
camera.start_preview()
camera.start_recording('/home/pi/Videos/'+time.strftime("%S_%M_%d_%b")+'.h264')
time.sleep(5)
camera.stop_recording()
camera.stop_preview()

