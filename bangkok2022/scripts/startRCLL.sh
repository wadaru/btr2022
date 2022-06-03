#!/bin/bash
BTR_DIR="~/git/btr2022"
GAME_DIR="bangkok2022"
LAUNCH_DIR="$BTR_DIR/$GAME_DIR/launch"

for PROGNAME in roscore; do
	killall $PROGNAME 2> /dev/null
done

sudo chmod 777 /dev/ttyUSB?

eval cd $LAUNCH_DIR
roslaunch btr2022.launch
