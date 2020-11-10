from picamera import PiCamera, Color
import time

camera = PiCamera()

class DroneCamera:

    
    def __init__(self) : 
        #camera = PiCamera()

        camera.resolution = (1920,1080)
        camera.framerate = 15

        camera.annotate_text = "Drone Video"
        camera.annotate_background = Color('blue')
        camera.annotate_foreground = Color('yellow')

    def takeVideo(self, length) :
        camera.start_preview()
        camera.start_recording('/home/pi/Videos/'+time.strftime("%S_%M_%d_%b")+'.h264')
        time.sleep(length)
        camera.stop_recording()
        camera.stop_preview()
    

x = DroneCamera()
x.takeVideo(10)
