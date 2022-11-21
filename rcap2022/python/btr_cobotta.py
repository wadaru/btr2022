#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import time
import threading
import os
import sys
import textwrap
import serial
import serial.tools.list_ports

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

serialPort = "/dev/ttyAMA0"
# serialBaud = "115200"
serialBaud = "1000000"

class MycobotBTR(object):
    def __init__(self):
        self.mycobot = None

    def run(self, command):
        self.connect_mycobot()
        self.mycobot.power_on()
        self.send_color("green")
        if command == "init":
            self.gripperClose()
            self.mycobot.send_angles([0, 0, 0, 0, 0, 0], 30)
            time.sleep(1)
        if command == "status":
            self.getStatus()
        if command == "teach":
            self.teaching()
        if command == "open":
            self.gripperOpen()
        if command == "close":
            self.gripperClose()
        if command == "gripper":
            print(self.mycobot.get_gripper_value(), self.mycobot.is_gripper_moving())
        if command == "move_g":
            self.getWork()
        if command == "move_r":
            self.releaseWork()
        self.disconnect_mycobot()

    def getWork(self):
        self.gripperGoInit()
        self.gripperGoStart()
        self.gripperOpen()
        self.gripperGoAbove()
        self.gripperGoWork()
        self.gripperClose()
        time.sleep(1)
        self.gripperGoAbove()
        self.gripperGoStart()
        self.gripperGoInit()

    def releaseWork(self):
        self.gripperGoInit()
        self.gripperGoStart()
        self.gripperClose()
        self.gripperGoAbove()
        self.gripperGoWork()
        self.gripperOpen()
        time.sleep(1)
        self.gripperGoAbove()
        self.gripperClose()
        self.gripperGoStart()
        self.gripperGoInit()

    def gripperGoInit(self):
        # self.mycobot.send_angles([10.01, 17.4, 18.19, -115.04, -76.02, 54.14], 30)
        self.mycobot.send_angles([95.62, 32.78, -129.46, 103.79, -89.47, -50.97], 30)
    def gripperGoStart(self):
        self.mycobot.send_angles([10.01, 17.4, 18.19, -115.04, -76.02, 54.14], 100)
        time.sleep(1)
    def gripperGoAbove(self):
        self.mycobot.send_angles([25.75, -42.71, 8.34, -79.54, -77.51, 68.2], 100)
    def gripperGoWork(self):
        self.mycobot.send_angles([32.87, -63.89, 8.26, -113.11, -67.14, 138.07], 100)

    def gripperOpen(self):
        # self.mycobot.set_gripper_value(2047, 10)
        self.mycobot.set_gripper_state(0, 100)
        time.sleep(0.5)

    def gripperClose(self):
        # self.mycobot.set_gripper_value(1400, 10)
        self.mycobot.set_gripper_state(1, 100)
        time.sleep(0.5)

    def teaching(self):
        self.send_color("blue")
        self.mycobot.release_all_servos()
        self.count_10s()
        for i in range(1, 6):
            self.mycobot.focus_servo(i)
        time.sleep(1)
        self.getStatus()

    def getStatus(self):
        print("Angles: ", self.mycobot.get_angles())
        print("Radians: ", self.mycobot.get_radians())

    # ============================================================
    # Connect method
    # ============================================================
    def connect_mycobot(self):
        global serialPort, serialBaud
        self.port = port = serialPort
        self.baud = baud = serialBaud
        baud = int(baud)

        # self.mycobot = MyCobot(PI_PORT, PI_BAUD)
        self.mycobot = MyCobot(port, baud)
        # self.mycobot = MyCobot("/dev/cu.usbserial-0213245D", 115200)

    def disconnect_mycobot(self):
        if not self.has_mycobot():
            return

        del self.mycobot
        self.mycobot = None

    # ============================================================
    #  Function method
    # ============================================================
    def count_10s(self):
        for i in range(8):
            time.sleep(0.5)
            self.send_color("green")
            time.sleep(0.5)
            self.send_color("blue")
        for i in range(4):
            time.sleep(0.25)
            self.send_color("green")
            time.sleep(0.25)
            self.send_color("red")

    def release_mycobot(self):
        if not self.has_mycobot():
            return
        self.mycobot.release_all_servos()

    def send_color(self, color: str):
        if not self.has_mycobot():
            return

        color_dict = {
            "red": [255, 0, 0],
            "green": [0, 255, 0],
            "blue": [0, 0, 255],
        }
        self.mycobot.set_color(*color_dict[color])
        # print("send color", color)

    # ============================================================
    # Utils method
    # ============================================================
    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.mycobot:
            print("no connection to myCobot")
            return False
        return True

    def get_current_time(self):
        """Get current time with format."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        return current_time

if __name__ == "__main__":
    command = sys.argv[1]
    MycobotBTR().run(command)
