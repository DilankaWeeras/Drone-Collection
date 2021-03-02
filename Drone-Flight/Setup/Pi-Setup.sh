sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip
sudo apt-get install python-dev
sudo pip install future
sudo apt install libxslt1-dev
sudo apt install libxml2-dev
sudo apt-get install screen python-wxgtk4.0 python-lxml
sudo pip install pyserial
sudo pip install dronekit
sudo pip install MAVProxy

echo "
-Set up RPi for UART communication
	raspi-config then disable UART for console, enable it for serial port hardware go into /boot/config.txt and add "dtoverlay=disable-bt" if ttyAMA0 isnt in /dev, enable uart=1 in /boot/config

"
