#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
# SCRIPT_DIR="~/ownCloud/2021/scripts"
# ROBVW2_DIR="~/ownCloud/2021/rvw2"
# PYTHON_DIR="~/ownCloud/2021/python"
SCRIPT_DIR="~/git/btr2021/jo2021/scripts"
ROBVW2_DIR="~/git/btr2021/jo2021/rvw2"
PYTHON_DIR="~/git/btr2021/jo2021/python"

ROBVIEW="robview"
# ROBVIEW="robview_interpreter"
for PROGNAME in roscore; do
	killall $PROGNAME
done

sudo chmod 777 /dev/ttyUSB?

# gnome-terminal --geometry=105x56 --window\
xterm -e "bash -c roscore" &
xterm -e "sleep 1; cd $SCRIPT_DIR; bash -c $SCRIPT_DIR/rosRcllRefBoxNetwork.sh; bash" &
xterm -e "sleep 1; cd $ROBVW2_DIR; bash -c '$ROBVIEW -f $ROBVW2_DIR/ros.rvw2'; bash" &
xterm -e "sleep 1; cd $PYTHON_DIR; bash -c 'sleep 2; rosrun rcll_btr_msgs robotino.py'; bash" &
xterm -e "sleep 1; cd ~/catkin_ws/src/rplidar_ros/launch; bash -c 'cd ~/catkin_ws/src/rplidar_ros/launch; roslaunch rplidar_a3.launch'; bash" &
xterm -e "sleep 2; cd $PYTHON_DIR; bash -c $PYTHON_DIR/btr_rplidar.py; bash" &
xterm -e "cd $PYTHON_DIR; bash" & # refbox.py

