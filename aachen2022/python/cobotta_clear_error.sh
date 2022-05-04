#!/bin/bash
rosservice call /cobotta/clear_error
rosservice call /cobotta/set_motor_state "state: True"
