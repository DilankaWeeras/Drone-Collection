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

# to take still pictures

camera.start_preview()
time.sleep(2)
timeStamp = time.strftime("%S_%M_%d_%b")
camera.capture('/home/pi/Pictures/'+timeStamp+'.jpg')
camera.stop_preview()

# to take multiple pictures
# add a for loop

# to record video
camera.start_preview()
camera.start_recording('/home/pi/Videos/'+time.strftime("%S_%M_%d_%b")+'.h264')
time.sleep(4)
camera.stop_recording()
camera.stop_preview()

# additional functionality
camera.resolution = (1920,1080) #max is (2592,1944) for pic / (1920,1080) for vid at 15fps
camera.framerate = 15
camera.start_preview()
camera.start_recording('/home/pi/Videos/'+time.strftime("%S_%M_%d_%b")+'.h264')
time.sleep(4)
camera.stop_recording()
camera.stop_preview()

#annotate text
camera.start_preview()
camera.annotate_text = "Hello World!"
camera.annotate_background = Color('blue')
camera.annotate_foreground = Color('white')
time.sleep(2)
timeStamp = time.strftime("%S_%M_%d_%b")
camera.capture('/home/pi/Pictures/'+timeStamp+'.jpg')
camera.stop_preview()
