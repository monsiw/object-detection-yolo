Main node:
mtrackercar mtrackercar

1. Configuration

To enable serial USB for any user make the following:

sudo gedit /etc/udev/rules.d/50-ttyusb.rules

KERNEL=="ttyUSB[0-9]*",NAME="tts/USB%n",SYMLINK+="%k",GROUP="uucp",MODE="0666"


2. Compile the package.

3. Example

a. start the car driver and configure coordinates frames
roslaunch mtrackercar mtracker_demo.launch

b. start demo controller 
rosrun mtrackercar controller

c. observe frames in rviz
rosrun rviz

4. Analyse the controller node to understand in which way the car can be controlled.

