echo "to run use the promp --connect and then use the connection string"
echo "Serial Port : /dev/ttyAMA0 , also set baud=57600"
echo "SITL UDP : 127.0.0.1:14550"
echo "Windows computer connected useing 3DR Telemetry Radio on COM14: com14 , also set baud=57600"
python mission_main.py --connect udpin:0.0.0.0:14550
