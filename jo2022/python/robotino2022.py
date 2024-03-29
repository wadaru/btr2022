#!/usr/bin/python

import struct
import time
import math
import sys
import rospy
import numpy
from numpy import linalg
from scipy import interpolate

from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
# import rcll_btr_msgs
from rcll_btr_msgs.msg import TagInfoResponse, TagLocationResponse
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance, TagInfo,     TagLocation

turn_angle    = numpy.array([-999, -20, -10,  -5, -2, -0.2, 0.1,  2,  5,  10,  20, 999])
turn_velocity = numpy.array([  30,  20,  10,   3,  3,    0,   0, -3, -3, -10, -20, -30])

go_distance = numpy.array([-999, -50, -20, -15, -10, 10, 15, 20,  50, 999])
go_velocity = numpy.array([ -50, -50,- 10, -10,   0,  0, 10, 10,  50,  50])

go_distance_fast = numpy.array([-999, -20, -10,   -1, -0.9, 0,   1, 1.1, 5, 10,  20, 999])
go_velocity_fast = numpy.array([ -50, -50, -50,  -15,    0, 0,   0,  15, 15, 50, 100, 100])

def tangent_angle(u, v):
    i = numpy.inner([u.y, u.x], [v.y, v.x])
    n = linalg.norm([u.y, u.x]) * linalg.norm([v.y, v.x])
    if (n == 0 ):
        return 90
    else:
        c = i / n
    return numpy.rad2deg(numpy.arccos(numpy.clip(c, -1.0, 1.0)))

def MPS_angle(u, v):
    print(u, v)
    p0 = Point()
    p1 = Point()
    p0.x = 1.0
    p0.y = 0.0
    p1.x = v.x - u.x
    p1.y = v.y - u.y
    return tangent_angle(p1, p0)

class robotino2022(object):
    def __init__(self):
        self.btrOdometry = Odometry()
        self.btrVelocity = Float32MultiArray()

        # rospy.init_node('robotino2022')
        self.sub1 = rospy.Subscriber("robotino/odometry", Odometry, self.robotinoOdometry)
        self.sub2 = rospy.Subscriber("robotino/getVelocity", Float32MultiArray, self.robotinoVelocity)
        self.sub3 = rospy.Subscriber("/btr/centerPoint", Point, self.centerPoint)
        self.sub4 = rospy.Subscriber("/btr/leftPoint", Point, self.leftPoint)
        self.sub5 = rospy.Subscriber("/btr/rightPoint", Point, self.rightPoint)
        self.rate = rospy.Rate(10)

        # self.pose = Pose2D()
        # self.pose.x = -2500
        # self.pose.y = 500
        # self.pose.theta = 90
        # print("init", self.pose.x, self.pose.y, self.pose.theta)
        # self.setOdometry(self.pose)
        
    def run(self):
        print("run")

    def setOdometry(self, data):
        odometry = SetOdometry()
        pose = Pose2D()
        rospy.wait_for_service('/rvw2/setOdometry')
        setOdometry = rospy.ServiceProxy('/rvw2/setOdometry', SetOdometry)
        odometry.header = Header()
        pose = data
        odometry.pose = pose
        resp = setOdometry(odometry.header, odometry.pose)

    def setVelocity(self, data):
        velocity = SetVelocity()
        pose = Pose2D()
        rospy.wait_for_service('/rvw2/setVelocity')
        setVelocity = rospy.ServiceProxy('/rvw2/setVelocity', SetVelocity)
        velocity.header = Header()
        pose = data
        velocity.pose = pose
        print("send")
        resp = setVelocity(velocity.header, velocity.pose)
        print("setVelocity")

    def goToPoint(self, x, y, theta):
        self.position = SetPosition()
        self.pose = Pose2D()
        rospy.wait_for_service('/rvw2/positionDriver')
        self.setPosition = rospy.ServiceProxy('/rvw2/positionDriver', SetPosition)
        self.position.header = Header()
        self.pose.x = x
        self.pose.y = y
        self.pose.theta  = theta
        self.position.pose = pose
        print("send")
        self.resp = self.setPosition(self.position.header, self.position.pose)
        print("goToPoint")

    def moveRobotino(self, x, y, theta):
        self.position = SetPosition()
        self.pose = Pose2D()
        rospy.wait_for_service('/rvw2/move')
        self.setPosition = rospy.ServiceProxy('/rvw2/move', SetPosition)
        self.position.header = Header()
        self.pose.x = x
        self.pose.y = y
        self.pose.theta  = theta
        self.position.pose = pose
        print("send")
        self.resp = self.setPosition(self.position.header, self.position.pose)
        print("goToPoint")

    def goToInputVelt(self):    # 375mm from left side(= 25 + 50*7)
        self.goToMPSCenter()
        self.move(0, 20, 0, 4)
        self.goToWall(15)

    def goToOutputVelt(self):   # 325mm from left side (= 25 + 50*6)
        self.goToMPSCenter()
        self.move(0, 15, 0, 1)
        self.goToWall(15)

    def move(self, x, y, theta, number):
        v = Pose2D()
        v.x = x
        v.y = y
        v.theta = theta
        for i in range(number):
            self.setVelocity(v)
            # rospy.sleep(1)
        v.x = 0
        v.y = 0
        v.theta = 0
        self.setVelocity(v)

    def robotinoTurn(self, turnAngle):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            nowAngle = self.btrOdometry
            print(nowAngle)
            if (nowAngle.header.seq != 0):
                break

        targetAngle = nowAngle.pose.pose.position.z + turnAngle

        v = Pose2D()
        v.x = 0
        v.y = 0
        while True:
            diff = (targetAngle - self.btrOdometry.pose.pose.position.z)
            if (diff > 180):
                diff -= 180
            if (diff < -180):
                diff += 180
            v.theta = -velocity1(diff)
            self.setVelocity(v)
            print(diff, v)
            if ((-3 < diff) and (diff < 3)):
                break
        v.theta = 0
        self.setVelocity(v)
        print("finish")

    def goToMPSCenter(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        version = 1 
        if (version == 1):
            for i in range(2):
                # turn parallel for the face of MPS.
                self.parallelMPS()
                # goTo at the front of the MPS with 50cm.
                self.goToWall(50)
                # go to the front of the MPS.
                self.goToMPSCenter1()
            self.goToWall(17)
            self.parallelMPS()
        elif (version == 2):
            rospy.wait_for_service('/btr_aruco/TagLocation')
            tagInfo = rospy.ServiceProxy('/btr_aruco/TagLocation', TagLocation)
            tag = tagInfo()
            print(tag)
            if (tag.ok == True):
                degree = math.atan(tag.tag_location.y / tag.tag_location.x)
                if (tag.tag_location.y < 0):
                    degree = -degree
                self.robotinoTurn(degree)
                for i in range(2):
                    # turn parallel for the face of MPS.
                    self.parallelMPS()
                    # goTo at the front of the MPS with 50cm.
                    self.goToWall(50)
                    # go to the front of the MPS.
                    self.goToMPSCenter1()
                self.goToWall(17)
                self.parallelMPS()

    def goToMPSCenter1(self):
        global go_distance, go_velocity
                
        velocityY = interpolate.interp1d(go_distance, go_velocity)
        while True:
            dist = self.leftPoint.y * 1000 + self.rightPoint.y * 1000
            v = Pose2D()
            v.x = 0
            if (math.isnan(dist) or math.isinf(dist)):
                v.y = 0
            else:
                v.y = velocityY(dist)
            v.theta = 0
            print("MPSCenter:", dist, v.y)
            if ((-1 < v.y) and (v.y < 1)):
                v.y = 0
            if (not(math.isnan(dist))):
                self.setVelocity(v)
            if (v.y == 0):
                break

    def goToWall(self, distance):
        global go_distance_fast, go_velocity_fast
        velocityX = interpolate.interp1d(go_distance_fast, go_velocity_fast)
        while True:
            sensor = self.centerPoint.x * 100
            v = Pose2D()
            if (math.isnan(sensor) or math.isinf(sensor)):
                if (distance < 17):
                    v.x = 0
                else:
                    v.x = -15
            else:
                v.x = velocityX(sensor - distance)
            v.y = 0
            v.theta = 0
            print("Wall ", distance, "cm:", sensor, v.x)
            if ((-1 < v.x) and (v.x < 1)):
                v.x = 0
            if (not(math.isnan(sensor))):
                self.setVelocity(v)
            if (v.x == 0):
                break


    def parallelMPS(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            angle1 = MPS_angle(self.leftPoint, self.rightPoint)
            angle2 = MPS_angle(self.leftPoint, self.centerPoint)
            angle3 = MPS_angle(self.centerPoint, self.rightPoint)
            angle = (angle1 + angle2 + angle3) / 3.0
            #
            v = Pose2D()
            v.x = 0
            v.y = 0
            if (math.isnan(angle)):
                angle = 90
            else:
                v.theta = velocity1(angle - 90)  

            print("parallelMPS:", angle, v.theta)
            if ((-1 < v.theta) and (v.theta < 1)):
                v.theta = 0
            if (not(math.isnan(angle))):
                    self.setVelocity(v)
            if (v.theta == 0):
                break

    def turnClockwise(self):
        rospy.wait_for_service('/rvw2/turnClockwiseMPS')
        self.turnClockwise = self.rospy.ServiceProxy('/rvw2/turnClockwiseMPS', Empty)
        print("turnClockwiseMPS")
        self.resp = self.turnClockwise()

    def turnCounterClockwise(self):
        rospy.wait_for_service('/rvw2/turnCounterClockwiseMPS')
        self.turnCounterClockwise = self.rospy.ServiceProxy('/rvw2/turnCounterClockwiseMPS', Empty)
        print("turnCounterClockwiseMPS")
        self.resp = self.turnCounterClockwise()
    
    def getWork(self):
        rospy.wait_for_service('/btr/move_g')
        self.getWork = self.rospy.ServiceProxy('/btr/move_g', Empty)
        print("getWork")
        self.resp = self.getWork()
        print("finish")

    def putWork(self):
        rospy.wait_for_service('btr/move_r')
        self.putWork = self.rospy.ServiceProxy('/btr/move_r', Empty)
        print("putWork")
        self.resp = self.putWork()
        print("finish")

    def robotinoOdometry(self, data):
        # global self.btrOdometry
        self.btrOdometry = data

    def robotinoVelocity(self, data):
        # global btrVelocity
        self.btrVelocity = data

    def centerPoint(self, data):
        self.centerPoint = data

    def leftPoint(self, data):
        self.leftPoint = data

    def rightPoint(self, data):
        self.rightPoint = data

#    robot.x = btrOdometry.pose.pose.position.x
#    robot.y = btrOdometry.pose.pose.position.y

# main
#
if __name__ == '__main__':
  args = sys.argv
  challenge = "test"
  if (len(args) == 2):
    challenge = args[1]

  print(challenge)
  challengeFlag = True

  agent = robotino2022()
  # while True:
  while not rospy.is_shutdown():

    if (challenge == "test" and challengeFlag):
        agent.run()
        # agent.goToOutputVelt()
        turnClockwise()
        goToInputVelt()
        # turnCounterClockwise()
        challengeFlag = False

    agent.rate.sleep()


