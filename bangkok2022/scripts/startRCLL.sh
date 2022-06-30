#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
BTR_DIR="~/git/btr2022"
GAME_DIR="bangkok2022"
# TERM="xterm -e"
TERM="gnome-terminal -- bash -c "
 
SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"
ROBVW2_DIR="$BTR_DIR/$GAME_DIR/rvw2"
PYTHON_DIR="$BTR_DIR/$GAME_DIR/python"
CATKIN_DIR="~/catkin_ws/src/rplidar_ros/launch"

ROBVIEW="robview"
# ROBVIEW="robview_interpreter"
for PROGNAME in roscore; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB?

# gnome-terminal --geometry=105x56 --window\
$TERM "bash -c roscore" &
$TERM "sleep 1; cd $SCRIPT_DIR; bash -c ./rosRcllRefBoxNetwork.sh; bash" &
$TERM "sleep 1; cd $SCRIPT_DIR; bash -c ./robview2.sh; bash" &
$TERM "sleep 3; cd $PYTHON_DIR; python  ./robotino.py robotino2 9180 10.42.0.1 9182; bash" &
$TERM "sleep 1; cd $CATKIN_DIR; roslaunch rplidar.launch; bash" &
$TERM "sleep 2; cd $PYTHON_DIR; bash -c ./btr_rplidar.py; bash" &
$TERM "sleep 2; cd $PYTHON_DIR; python  ./aruco.py; bash" &
$TERM "sleep 2; cd $PYTHON_DIR; bash -c ./btr_myCobot_ros.py; bash" &

