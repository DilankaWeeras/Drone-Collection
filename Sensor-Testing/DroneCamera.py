from picamera import PiCamera, Color
import time

class DroneCamera:
    
    def __init__(self) : 
        self.camera = picamera.PiCamera()

        self.camera.resolution = (1920,1080)
        self.camera.framerate = 15

        self.camera.annotate_text = "Drone Video"
        self.camera.annotate_background = Color('blue')
        self.camera.annotate_foreground = Color('yellow')

    def __del__(self):
        self.camera.close()

    def takeVideo(self, length) :
        self.camera.start_preview()
        self.camera.start_recording('/home/pi/Videos/'+time.strftime("%S_%M_%d_%b")+'.h264')
        time.sleep(length)
        self.camera.stop_recording()
        self.camera.stop_preview()
    

x = DroneCamera()
x.takeVideo(4)