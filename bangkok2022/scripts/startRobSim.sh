#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
BTR_DIR="~/git/btr2022"
GAME_DIR="bangkok2022"
TERM="xterm -e"
# TERM="gnome-terminal --"

SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"
# ROBVW2_DIR="$BTR_DIR/$GAME_DIR/rvw2"
ROSBTR_DIR="~/catkin_ws/src/ros-rcll_btr_msgs/scripts/"
PYTHON_DIR="$BTR_DIR/$GAME_DIR/python"

ROBVIEW="robview"
# ROBVIEW="robview_interpreter"
# for PROGNAME in roscore; do
# 	killall $PROGNAME 2> /dev/null
# done

sudo chmod 777 /dev/ttyUSB?

# gnome-terminal --geometry=105x56 --window\
# xterm -e "bash -c roscore" &
# $TERM "sleep 1; cd $SCRIPT_DIR; bash -c $ROSBTR_DIR/rosRcllRefBoxNetwork.sh; bash" &
$TERM "sleep1; cd ~/catkin_ws/src/ros-rcll_refbox_peer/launch; roslaunch rcll_refbox_peer.launch num_robots:=1 team_name:=BabyTigers robot_name:=kotora robot_number:=1 crypto_key:=randomkey peer_address:=127.0.1.1 peer_public_send_port:=4444 peer_public_recv_port:=4444 peer_cyan_send_port:=4441 peer_cyan_recv_port:=4441 peer_magenta_send_port:=4442 peer_magenta_recv_port:=4442; bash" &
$TERM "sleep 1; cd $ROBVW2_DIR; bash -c $ROSBTR_DIR/robview.sh; bash" &
$TERM "sleep 3; cd $PYTHON_DIR; python $ROSBTR_DIR/robotino.py; bash" &
# xterm -e "sleep 2; cd ~/catkin_ws/src/rplidar_ros/launch; bash -c 'cd ~/catkin_ws/src/rplidar_ros/launch; roslaunch rplidar_a3.launch'; bash" &
# xterm -e "sleep 3; cd $PYTHON_DIR; bash -c $PYTHON_DIR/btr_rplidar.py; bash" &
$TERM "cd $PYTHON_DIR; bash" & # refbox.py
$TERM "cd $REFBOX_DIR; bash -c $REFBOX_DIR/llsf-refbox; bash" &
$TERM "cd $REFBOX_DIR; bash -c $REFBOX_DIR/llsf-refbox-shell; bash" &
$TERM "cd $REFBOX_DIR; bash -c ~/robotinoSimDemo.sh; bash" &

