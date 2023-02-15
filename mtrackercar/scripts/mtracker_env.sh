#!/bin/sh

# THIS IS A TEMPLATE FILE THAT SHOULD BE KEPT IN HOME FOLDER
# CHANGE IPS TO PROPER VALUES

export ROS_IP=192.168.1.190
export ROS_HOSTNAME=192.168.1.190

. /opt/ros/hydro/setup.sh
. /home/$(env USER)/catkin_ws/devel/setup.sh
exec "$@"
