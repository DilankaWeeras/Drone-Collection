#Installing QGroundControl
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager

wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage