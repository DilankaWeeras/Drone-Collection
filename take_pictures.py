from picamera import PiCamera, Color
import time

camera = PiCamera()
count = 0

while count < 30:
    camera.start_preview()
    time.sleep(2)
    timeStamp = time.strftime("%S_%M_%d_%b")
    camera.capture('/home/pi/Pictures/'+ str(count) +'testpix.jpg')
    camera.stop_preview()
    time.sleep(5)
    count = count+1
    print('Taking Picture')
