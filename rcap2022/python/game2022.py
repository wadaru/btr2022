#!/usr/bin/python
import struct
import time
import math
import sys
import rospy
# import robotino2022
import btr2022
import quaternion
import tf
import rcll_ros_msgs
import rcll_btr_msgs
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PointStamped, Point, \
                              Quaternion, Vector3
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

TEAMNAME = "BabyTigers"

FIELDMINX = -5
FIELDMAXX = -1
FIELDMINY =  1
FIELDMAXY =  5
FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999

zoneX = { "S11" :  -500, "S21" : -1500, "S31" : -2500, "S41" : -3500, "S51" : -4500,
          "S12" :  -500, "S22" : -1500, "S32" : -2500, "S42" : -3500, "S52" : -4500,
          "S13" :  -500, "S23" : -1500, "S33" : -2500, "S43" : -3500, "S53" : -4500,
          "S14" :  -500, "S24" : -1500, "S34" : -2500, "S44" : -3500, "S54" : -4500,
          "S15" :  -500, "S25" : -1500, "S35" : -2500, "S45" : -3500, "S55" : -4500 }
zoneY = { "S11" :  500, "S21" :  500, "S31" :  500, "S41" :  500, "S51" :  500,
          "S12" : 1500, "S22" : 1500, "S32" : 1500, "S42" : 1500, "S52" : 1500,
          "S13" : 2500, "S23" : 2500, "S33" : 2500, "S43" : 2500, "S53" : 2500,
          "S14" : 3500, "S24" : 3500, "S34" : 3500, "S44" : 3500, "S54" : 3500,
          "S15" : 4500, "S25" : 4500, "S35" : 4500, "S45" : 4500, "S55" : 4500 }
inputX = { 0: 1000, 45: 500, 90:     0, 135:  -500, 180: -1000, 225:  -500, 270:     0, 315: 500, 360: 1000}
inputY = { 0:     0, 45: -500, 90: -1000, 135: -500, 180:    0, 225:  500, 270:  1000, 315:  500, 360:     0}
outputX = {  0: inputX[180],  45: inputX[225],  90: inputX[270], 135: inputX[315],
           180: inputX[  0], 225: inputX[ 45], 270: inputX[ 90], 315: inputX[135]}
outputY = {  0: inputY[180],  45: inputY[225],  90: inputY[270], 135: inputY[315],
           180: inputY[  0], 225: inputY[ 45], 270: inputY[ 90], 315: inputY[135]}

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    # print("ExplorationInfo: ", data)

def gameState(data):
    global refboxTime, refboxGameState, refboxGamePhase, \
           refboxPointsMagenta, refboxTeamMagenta, \
           refboxPointCyan, refboxTeamCyan
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)
    sendBeacon()

def machineInfo(data):
    global refboxMachineInfo, refboxMachineInfoFlag
    refboxMachineInfo = data
    refboxMachineInfoFlag = True
    # print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    # print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    # print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    # print("RingInfo: ", data)

def navigationRoutes(data):
   global refboxNavigationRoutes, refboxNavigationRoutesFlag
   refboxNavigationRoutes = data
   refboxNavigationRoutesFlag = True
   # print("NavigaionRoutes: ", data)

#
# send information to RefBox
#
def sendBeacon():
    global btrOdometry
    beacon = SendBeaconSignal()
    beacon.header = Header()

    # for poseStamped()
    beacon.pose = PoseStamped()
    beacon.pose.pose.position.x = btrOdometry.pose.pose.position.x
    beacon.pose.pose.position.y = btrOdometry.pose.pose.position.y
    beacon.pose.pose.position.z = 0
    beacon.pose.pose.orientation.x = btrOdometry.pose.pose.orientation.x
    beacon.pose.pose.orientation.y = btrOdometry.pose.pose.orientation.y
    beacon.pose.pose.orientation.z = btrOdometry.pose.pose.orientation.z
    beacon.pose.pose.orientation.w = btrOdometry.pose.pose.orientation.w
    beacon.header.seq = 1
    beacon.header.stamp = rospy.Time.now()
    beacon.header.frame_id = TEAMNAME
    beacon.pose.header.seq = 1
    beacon.pose.header.stamp = rospy.Time.now()
    beacon.pose.header.frame_id = "robot1"

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        # print("/rcll/send_beacon")
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        # print("sendBeacon: ", beacon.header, beacon.pose)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        # resp1 = refboxSendBeacon(beacon.header, beacon.point)
        # print("sendBeacon: ", beacon.header, beacon.pose)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendMachineReport(report):
    sendReport = SendMachineReport()
    machineReport = MachineReportEntryBTR()
    machineReport.name = report.name
    machineReport.type = report.type
    machineReport.zone = report.zone
    machineReport.rotation = report.rotation
    if (refboxTeamCyan == TEAMNAME):
        sendReport.team_color = 1
    else:
        sendReport.team_color = 2
    machineReportEntryBTR = [machineReport]
    sendReport.machines = machineReportEntryBTR
    print("machineReport: ", machineReport)

    rospy.wait_for_service('/rcll/send_machine_report')
    try:
        refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        resp1 = refboxMachineReport(sendReport.team_color, sendReport.machines)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendPrepareMachine(data):
    prepare = SendPrepareMachine()
    # prepare.machine = data.machine
    prepare.machine = data.machine
    prepare.bs_side = 0
    prepare.bs_base_color = 0
    prepare.ds_order_id = 0
    prepare.cs_operation = 0
    prepare.rs_ring_color =0
    
    machineType = prepare.machine[2:4]
    print(machineType)
    if (machineType == "BS"):
        prepare.bs_side = data.bs_side
        prepare.bs_base_color =data.bs_base_color
    if (machineType == "DS"):
        prepare.ds_order_id = data.ds_order_id
    if (machineType == "CS"):
        prepare.cs_operation = data.cs_operation
    if (machineType == "RS"):
        prepare.rs_ring_color = data.rs_ring_color
    prepare.wait = data.wait
    rospy.wait_for_service('/rcll/send_prepare_machine')
    try:
        refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        resp1 = refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def robotinoOdometry(data):
    global btrOdometry, btrBeaconCounter
    quat = quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
    btrOdometry = data
    ##btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
    btrOdometry.pose.pose.position.z = btrOdometry.pose.pose.position.z # / math.pi * 180
    # btrOdometry = data
    btrBeaconCounter +=1
    if (btrBeaconCounter > 5):
      sendBeacon()
      btrBeaconCounter = 0

# 
# challenge program
#

def startGrasping():
    for j in range(3):
        print(j)
        btrRobotino.w_goToOutputVelt()
        btrRobotino.w_getWork()
        #
        if (robotNum != 2):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()
        btrRobotino.w_goToInputVelt()
        btrRobotino.w_putWork()
        print("turn")
        if (robotNum != 2):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()
        # finish?

def initField():
    global btrField
    btrField = [[0 for y in range(5)] for x in range(5)]
    #
    # this field is [y][x]
    # but game field is from -5 to -1
    # so when you use this variable, please shift argment + 5.
    #   (-5, 5) => (0, 4)
    #   (-5 ,1) => (0, 0)

def setField(x, y, number):
    global FIELDMINX, FIELDMINY
    # print(x, FIELDMINX, y, FIELDMINX, number)
    btrField[y - FIELDMINY][x - FIELDMINX] = number

def getField(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if (int(x) < FIELDMINX or FIELDMAXX < int(x) or int(y) < FIELDMINY or FIELDMAXY < int(y)):
        # print("getField range over: ", x, y)
        return MAXSTEP 
    return btrField[int(y) - FIELDMINY][int(x) - FIELDMINX]

def zoneToPose2D(zone):
    point = Pose2D()
    if zone < 0:        
        zone = zone + 255
    point.y = zone % 10
    point.x = (zone % 100) // 10
    if (zone > 100):
        point.x = -point.x
    return point

def setMPStoField():
    global btrField
    point = Pose2D()
    for machine in refboxMachineInfo.machines:
        point = zoneToPose2D(machine.zone)
        print("setMPS: ", machine.name, machine.zone, point.x, point.y)
        if (point.x == 0 and point.y == 0):
            print("received NULL data for MPS", machine.name)
        else:
            setField(point.x, point.y, MAXSTEP)

def getStep(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if ((x < FIELDMINX or FIELDMAXX < x) or (y < FIELDMINY or FIELDMAXY < y)):
        return MAXSTEP

    step = getField(x, y)
    if (step == 0):
        step = MAXSTEP
    return step

def wallCheck(x, y, dx, dy):
    notWallFlag = True
    if ((x == -5 and y == 1) and (dx ==  0 and dy ==  1)):
        notWallFlag = False
    if ((x == -4 and y == 1) and (dx ==  0 and dy ==  1)):
        notWallFlag = False
    if ((x == -3 and y == 1) and (dx ==  1 and dy ==  0)):
        notWallFlag = False
    if ((x == -2 and y == 1) and (dx == -1 and dy ==  0)):
        notWallFlag = False
    if ((x == -5 and y == 2) and (dx ==  0 and dy == -1)):
        notWallFlag = False
    if ((x == -4 and y == 2) and (dx ==  0 and dy == -1)):
        notWallFlag = False
    return notWallFlag

def getNextDirection(x, y):
    minStep = getField(x, y)
    nextD = Pose2D()
    nextD.x = nextD.y = 0
    for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
        notWallFlag = wallCheck(x, y, dx, dy)

        if ((minStep > getField(x + dx, y + dy)) and notWallFlag):
            minStep = getField(x + dx, y + dy)
            nextD.x = dx
            nextD.y = dy
    return nextD

def makeNextPoint(destination):
    global btrField, btrOdometry, FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    tmpField = btrField
    point = zoneToPose2D(destination)
    print("destination is ", destination, point.x, point.y)
    setField(point.x, point.y, 1)
    for i in range(FIELDSIZE):
        for x in range(FIELDMINX, FIELDMAXX + 1):
            for y in range(FIELDMINY, FIELDMAXY + 1):
                if (x == point.x and y == point.y):
                    setField(x, y, 1)
                elif (getField(x, y) != MAXSTEP):
                    setField(x, y, min(getStep(x - 1, y), getStep(x, y - 1), \
                                       getStep(x + 1, y), getStep(x, y + 1)) \
                                   + 1)
                    # wall information
                    if (x == -5 and y == 1): # M_Z51 = M_Z41 + 1
                        setField(x, y, getStep(x + 1, y) + 1)
                    if (x == -4 and y == 1): # M_Z41 = M_Z31 + 1
                        setField(x, y, getStep(x + 1, y) + 1)
                    if (x == -3 and y == 1): # M_Z31 = M_Z32 + 1
                        setField(x, y, getStep(x, y + 1) + 1)
                    if (x == -2 and y == 1): # M_Z21 <= min(M_Z22, MZ_11) + 1
                        setField(x, y, min(getStep(x, y + 1), getStep(x + 1, y)) + 1)

    # get optimized route
    if (False):
        for y in range(FIELDMAXY, FIELDMINY  - 1, -1):
            for x in range(FIELDMINX, FIELDMAXX + 1):
                if (getField(x,y) == MAXSTEP):
                    print("*",)
                else:
                    print(getField(x, y),)
            print()

    robotReal = Pose2D()
    robotZone = Pose2D()
    point = Pose2D()
    robotReal.x = btrOdometry.pose.pose.position.x
    robotReal.y = btrOdometry.pose.pose.position.y

    if (robotReal.x > 0):
        robotZone.x = int((robotReal.x) / 1000) + 1
    else:
        robotZone.x = int((robotReal.x) / 1000) - 1
    robotZone.y = int((robotReal.y) / 1000) + 1

    x = int(robotZone.x)
    y = int(robotZone.y)
    # which direction?
    nextD = getNextDirection(x, y)
    # where is the turning point?
    point.x = robotZone.x + nextD.x
    point.y = robotZone.y + nextD.y
    # print("direction: ",nextD.x, nextD.y)
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]):
        if (nextD.x == dx and nextD.y == dy):
            theta = phi

    goToPoint(robotReal.x, robotReal.y, theta) # turn for the next point.

    while True:
        # print(point)
        if getField(point.x, point.y) == 0:
            print("Next Point is goal")
            break
        if (getField(point.x + nextD.x, point.y + nextD.y) == getField(point.x, point.y) - 1):
            notWallFlag = wallCheck(point.x, point.y, nextD.x, nextD.y)
            if (notWallFlag):
                point.x = point.x + nextD.x
                point.y = point.y + nextD.y
            else:
                # there is a wall.
                break
        else:
            print("Next Point is the turning point")
            break

    # for the nextStep
    nextDD = getNextDirection(point.x, point.y)
    theta = -360
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]): 
        if (nextDD.x == dx and nextDD.y == dy):
           theta = phi

    print("nextPosition: ", point.x, point.y, theta)
    if (point.x == robotZone.x and point.y == robotZone.y):
        theta = 360
    point.theta = theta
    return point

def getNextPoint(pointNumber):
    point = Pose2D()
    route = refboxNavigationRoutes.route
    # zone = route[pointNumber].zone
    zone = route[0].zone

    print(zone)
    point = makeNextPoint(zone)
    return point

def startNavigation():
    global btrField
    initField()
    print("----")
    setMPStoField()
    print("====")
    oldTheta = 90
    for pointNumber in range(12):
        print(pointNumber)
        route = refboxNavigationRoutes.route
        if (len(route) == 0):
            print("finished")
        else:
            point = getNextPoint(pointNumber)
            robot = btrOdometry.pose.pose.position
            while True:
                point = getNextPoint(pointNumber)
                if (point.x > 0):
                    point.x = point.x * 1000 - 500
                else:
                    point.x = point.x * 1000 + 500
                point.y = point.y * 1000 - 500
                if (point.theta == 360):
                    goToPoint(point.x, point.y, oldTheta)
                    break
                else:
                    goToPoint(point.x, point.y, point.theta)
                oldTheta = point.theta

            print("arrived #", pointNumber + 1, ": point")
            for i in range(4):
                sendBeacon()
                rospy.sleep(2)

    print("****")
    # print(refboxNavigationRoutes)
    # print(refboxMachineInfo)


# main
#
if __name__ == '__main__':
  args = sys.argv
  if (len(args) >= 2):
    challenge = args[1]
    if (len(args) >= 3):
      robotNum = int(args[2])
    else:
      robotNum = 1

  # valiables for refbox
  refboxBeaconSignal = BeaconSignal()
  refboxExplorationInfo = ExplorationInfo()
  refboxExplorationSignal = ExplorationSignal()
  refboxExplorationZone = ExplorationZone()
  refboxGameState = Int8()
  refboxGamePhase = Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxLightSpec = LightSpec()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxMachineReportEntry = MachineReportEntryBTR()
  refboxMachineReportInfo = MachineReportInfo()
  refboxOrderInfo = OrderInfo()
  refboxOrder = Order()
  refboxProductColor = ProductColor()
  refboxRingInfo = RingInfo()
  refboxRing = Ring()
  refboxTeam = Team()
  refboxTime = Time()
  refboxNavigationRoutes = NavigationRoutes()
  refboxMachineInfoFlag = False
  refboxNavigationRoutesFlag = False

  btrOdometry = Odometry()
  btrBeaconCounter = 0
  btrVelocity = Float32MultiArray()

  btrField = [[0 for y in range(5)] for x in range(5)]

  rospy.init_node('btr2022')
  rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
  rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
  rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
  rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
  rospy.Subscriber("/odom", Odometry, robotinoOdometry)
  rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
  rate = rospy.Rate(10)

  machineReport = MachineReportEntryBTR()
  prepareMachine = SendPrepareMachine() 

  btrRobotino = btr2022.btr2022()

  pose = Pose2D()
  pose.x = -1000 * robotNum - 1500
  pose.y = 500
  pose.theta = 90
  if (challenge == "grasping"):
      startX =     [ -500, -4500, -500]
      startY =     [  500,  1500, 4500]
      startTheta = [   90,    90,  180]
      pose.x = startX[robotNum - 1]
      pose.y = startY[robotNum - 1]
      pose.theta = startTheta[robotNum - 1]
  if (challenge == "driving" or challenge == "positioning"):
      pose.x = zoneX["S31"]
      pose.y = zoneY["S31"]
      pose.theta = 90
  print(pose.x, pose.y, pose.theta)
  if (challenge == "reset"):
      # goToPoint(-800, -1500, 90)
      goToPoint(pose.x, pose.y, pose.theta)
      exit()
  btrRobotino.w_resetOdometry(pose)

  print(challenge)
  challengeFlag = True
  # while True:
  while not rospy.is_shutdown():
    # sendBeacon()
    # print("sendBeacon")
    
    if (challenge == "test" and challengeFlag):
        challengeFlag = False
        btrRobotino.w_goToOutputVelt()
        btrRobotino.w_getWork()
        btrRobotino.w_turnClockwise()
        btrRobotino.w_goToInputVelt()
        btrRobotino.w_putWork()
        btrRobotino.w_turnCounterClockwise()
        break

    if (challenge == "testMPS" and challengeFlag):
        if (True):
            btrRobotino.w_goToOutputVelt()
            btrRobotino.w_getWork()
            btrRobotino.w_turnClockwise()
            btrRobotino.w_goToInputVelt()
            btrRobotino.w_putWork()
            btrRobotino.w_turnCounterClockwise()
        else:
            startGrasping()
        challengeFlag = False

    if (challenge == "driving" and challengeFlag):
        print("startDriving for JapanOpen2020")
        targetZone =  ["S31", "S32", "S22", "S22", "S32", "S35", "S35", "S31"]
        #                            Target1               Target2
        targetAngle = [   90,     0,    90,   180,    90,    45,   270,    90]
        sleepTime   = [    0,     0,     5,     0,     0,     5,     0,     1]
        for number in range(len(targetZone)):
            print(targetZone[number])
            x = zoneX[targetZone[number]]
            y = zoneY[targetZone[number]]
            theta = targetAngle[number]
            goToPoint(x, y, theta)
            time.sleep(sleepTime[number])

        challengeFlag = False
        break

    if (challenge == "positioning" and challengeFlag):
        print("startPositioning for JapanOpen2020")
        #
        # MPSZone, MPSAngle, firstSide, turn
        #
        MPSZone = "S53" # Input !!!
        MPSAngle = 0  # Input !!!
        firstSide = "input"
        turn = "counterClock"

        # goTo S322
        goToPoint(zoneX["S32"], zoneY["S32"], 90)

        if (firstSide == "input"):
            MPSx = zoneX[MPSZone] + inputX[MPSAngle]
            MPSy = zoneY[MPSZone] + inputY[MPSAngle]
            theta = MPSAngle + 180
        else:
            MPSx = zoneX[MPSZone] + outputX[MPSAngle]
            MPSy = zoneY[MPSZone] + outputY[MPSAngle]
            theta = MPSAngle
        
        goToPoint(MPSx, MPSy, theta)
        btrRobotino.w_goToMPSCenter(350)
        if (firstSide == "input"):
            time.sleep(10)

        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            turnClockwise()
        else:
            turnCounterClockwise()
        btrRobotino.w_goToMPSCenter(350)
        time.sleep(10)
        
        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            turnCounterClockwise()
        else:
            turnClockwise()
        if (firstSide == "output"):
            btrRobotino.w_goToMPSCenter(350) 
            time.sleep(10)
        
        theta = 270
        goToPoint(MPSx, MPSy, theta)

        # goTo S32 & S31
        goToPoint(zoneX["S32"], zoneY["S32"], 270)
        goToPoint(zoneX["S31"], zoneY["S31"], 90)
        challengeFlag = False
        break

    if (refboxGamePhase == 30 and challenge == "grasping" and challengeFlag):
        startGrasping()
        challengeFlag = False
        break

    if (refboxGamePhase == 30 and challenge == "navigation" and challengeFlag):
        if (refboxMachineInfoFlag and refboxNavigationRoutesFlag):
            startNavigation()
            challengeFlag = False
            break

    # send machine report for Exploration Phase
    if (refboxGamePhase == 20):
        if (refboxTime.sec == 10):
            machineReport.name = "C-CS1"
            machineReport.type = "CS"
            machineReport.zone = -53 
            machineReport.rotation = 210
            sendMachineReport(machineReport)

    # send machine prepare command
    if (refboxGamePhase == 30 and challenge == "" ):
        # make C0
        # which requires get base with cap from shelf at C-CS1, 
        #                Retrieve cap at C-CS1,
        #                bring base without cap to C-RS1,
        #                get base at C-BS,
        #                bring base to C-CS1,
        #                Mount cap at C-CS1,
        #                bring it to C-DS corresponded by order it.

        if (refboxTime.sec ==   5):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 1 # CS_OP_RETRIEVE_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  30):
            prepareMachine.machine = "C-BS"
            prepareMachine.bs_side = 1  # INPUT or OUTPUT side
            prepareMachine.bs_base_color = 1 # BASE COLOR
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  60):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 0 # CS_OP_MOUNT_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  90):
            prepareMachine.machine = "C-DS"
            prepareMachine.ds_order_id = 1 # ORDER ID
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)

    rate.sleep()


