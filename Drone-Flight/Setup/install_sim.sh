#Clone ArduPilot
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

git checkout Copter-4.0.5
git submodule update --init --recursive

#Install dependencies
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect

#Install MAVProxy
sudo pip install future pymavlink MAVProxy

#Add Lines to ~/.bashrc
echo '
Add: to end of file

export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
'
gedit ~./bashrc
    #export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    #export PATH=/usr/lib/ccache:$PATH

./~bashrc

#Run SITL 
echo 'Running Copter SITL'
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
sim_vehicle.py -L (Location Name) --map --console