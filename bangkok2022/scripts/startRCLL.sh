#!/bin/bash
##
BTR_DIR="~/git/btr2022"
GAME_DIR="bangkok2022"
SCRIPT_DIR="$BTR_DIR/$GAME_DIR/scripts"

for PROGNAME in roscore; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB?

cd ~/$SCRIPT_DIR
roslaunch btr2022.launch
