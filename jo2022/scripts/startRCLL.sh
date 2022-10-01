#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
BTR_DIR="~/git/btr2022"
GAME_DIR="jo2022"
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
sleep 1
$TERM "echo rosRcllRefBoxNetwork.sh; sleep 1; cd $SCRIPT_DIR; bash -c ./rosRcllRefBoxNetwork.sh; bash" &
# $TERM "echo robview2.sh; sleep 1; cd $SCRIPT_DIR; bash -c ./robview2.sh; bash" &
$TERM "echo robview; sleep 1; cd $ROBVW2_DIR; bash -c '$ROBVIEW -f $ROBVW2_DIR/ros.rvw2'; bash" &
# $TERM "echo robotino.py; sleep 5; cd $PYTHON_DIR; python  ./robotino.py robotino2 9180 10.42.0.1 9182; bash" &
$TERM "echo robotino.py; sleep 5; cd $PYTHON_DIR; bash -c 'rosrun rcll_btr_msgs robotino.py'; bash" &
# $TERM "echo rplidar.launch; sleep 1; cd $CATKIN_DIR; roslaunch rplidar.launch; bash" &
$TERM "echo rplidar.launch; sleep 1; cd $CATKIN_DIR; roslaunch a3_rplidar.launch; bash" &
$TERM "echo btr_rplidar.py; sleep 2; cd $PYTHON_DIR; bash -c ./btr_rplidar.py; bash" &
$TERM "echo btr_camera.py; sleep 1; cd $PYTHON_DIR; python ./btr_camera.py; bash" &
$TERM "echo btr_aruco.py; sleep 2; cd $PYTHON_DIR; python  ./btr_aruco.py; bash" &
# $TERM "echo btr_myCobot_ros.py; sleep 2; cd $PYTHON_DIR; bash -c ./btr_myCobot_ros.py; bash" &
$TERM 'echo please power on and run the script for Cobotta.; bash" &


