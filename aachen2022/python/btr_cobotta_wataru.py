# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#
# usage: python ./packing_pose.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
import math
import moveit_commander
import rosservice
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState

from std_srvs.srv import Empty, EmptyResponse

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]

#
# Poses
#
joints_packing_old = [90, -60, 125, 90, -95, 0]
joints_packing_new = [90, -30, 120, -170, -94, 0]
joints_home = [0, 30, 100, 0, 50, 0]
joints_demo = [90, -30, 120, -170, -94, 50]
joint_1 = [1.663,0.447,1.437,0.083,1.027,0.321]
joint_2 = [1.081,0.475,1.639,0.301,1.028,0.321]
joint_3 = [1.575,0.252,1.372,0.789,1.028,0.321]

joint_first = [0.047683078589843726, -0.594701243274222, 1.4382421074734248, -0.046362160889901166, 0.5794516350371134, -0.01091915142670699]
joint_top1 = [0.24370694351808536, -0.08731717585263447, 1.3805230704407934, -0.01980721069814855, 0.21872966826197177, -0.03342906875279541]
# joint_grip1 = [0.2330647189103087, -0.05588865215603098, 1.6651671946252877, 0.1174937162901705, 0.06885131594062463, -0.043394960927964486]
joint_grip1 = [0.2856254837787167, -0.10940734248012898, 1.8093436461862942, -0.01887290830672645, -0.21725644941868588, -0.019173509945705724]
joint_grip2 = [0.2658191213142435, -0.15051910575123467, 1.8993234688513068, 0.054293350079306565, -0.28463454651602466, -0.0735092872833667]
# joint_grip2 =[0.21364265900111631, 0.14835253616607208, 1.485021178494382, 0.06423847997911071, 0.03392736336155386, -0.08780817605643537]
joint_top2 = [0.7162,0.294,0.8543,-0.016,0.3278,0.001]
joint_release = [0.6868,0.1569,1.4562,0.1121,-0.0499,-0.1313]
# joint_release2 = [0.26162135494117605, 0.22162680385038738, 1.4755721788897505, -0.05263236805011171, -0.09309443190587288, 0.07844890340497224]
joint_release2 = joint_grip1
joint_run = [-0.08705930963861734, -0.848481708396655, 2.173530828186504, -0.020720750814205716, 1.3772429686423875, -0.0658398833050844]

#
#
#PLAY EREA
#
#

def test_move(data):
    
    print "cobotta now moving..."
    
    move_group.go(joint_1,wait = True)


def grab_Arm(data):
    global gripper_client, gripper_parallel_open, gripper_parallel_close, gripper_parallel_speed, gripper_parallel_effort
    print "cobotta grabbing objects..."
    print gripper_parallel_open, gripper_parallel_close

    # move_group.go(joint_first,wait = True)
    gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
    move_group.go(joint_first,wait = True)
    # rospy.sleep(5)

    move_group.go(joint_top1,wait = True)
    move_group.go(joint_grip1,wait = True)
    gripper_move(gripper_client,gripper_parallel_close, gripper_parallel_speed, gripper_parallel_effort)
    rospy.sleep(5)
    move_group.go(joint_grip2,wait = True)
    gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
    rospy.sleep(3)

    move_group.go(joint_top1,wait = True)
    rospy.sleep(1)
    move_group.go(joint_run,wait = True)

    print"robotino moving next position..."

    return EmptyResponse()


def release_Arm(data):
    global gripper_client, gripper_parallel_open, gripper_parallel_close, gripper_parallel_speed, gripper_parallel_effort  
    print "cobotta releacing objects..."
    print gripper_parallel_open, gripper_parallel_close
    move_group.go(joint_first,wait = True)
    # rospy.sleep(5)

    move_group.go(joint_top1,wait = True)
    # move_group.go(joint_grip1,wait = True)
    move_group.go(joint_release2,wait = True)
    # move_group.go(joint_grip2,wait = True)
    gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
    rospy.sleep(5)

    move_group.go(joint_grip1,wait = True)
    # move_group.go(joint_grip2,wait = True)
    move_group.go(joint_top1,wait = True)
    rospy.sleep(1)
    move_group.go(joint_run,wait = True)
    print"robotino moving next position..."

    return EmptyResponse()


#
#
#
#
#


#
# Parallel gripper
#
gripper_parallel_open   =  0.015
gripper_parallel_mid    =  0.0010
gripper_parallel_close  =  0.0001
gripper_parallel_speed  = 10.0
gripper_parallel_effort = 10.0

def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = effort
    gripper_client.send_goal(goal)

def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state',
                                             GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

def is_simulation():
    service_list = rosservice.get_service_list()
    if '/cobotta/get_motor_state' in service_list:
        return False
    return True

def arm_move_left():
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',GripperMoveAction)
    gripper_width = 0.0

    arm = moveit_commander.MoveGroupCommander("arm")
    print("Reference frame: %s" % arm.get_planning_frame())
    print("Reference frame: %s" % arm.get_end_effector_link())

    target_pose_arm = arm.get_current_pose().pose
    print(target_pose_arm)

    gripper_width = gripper_parallel_open
    gripper_move(gripper_client, gripper_width,gripper_parallel_speed, gripper_parallel_effort)

#
#Test erea
#




if __name__ == '__main__':
    rospy.init_node("packing_pose")
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper_client=actionlib.SimpleActionClient('/cobotta/gripper_move',GripperMoveAction)
    
    print(os.path.basename(__file__) + " sets pose goal and moves COBOTTA.")
    print("start")
    src00 = rospy.Service('/btr/move_g',Empty,grab_Arm)
    src01 = rospy.Service('/btr/move_r',Empty,release_Arm)

    print"0: End"
    
    """while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)
            if input == 0:
                break"""
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

        """ while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)

        joints = []
        gripper_width = 0.0

        if input == 0:
            #joints = joints_packing_old

            arm_move_left()

            gripper_width = gripper_parallel_open

        elif input == 1:
            move_group.go(joint_1,wait = True)
            gripper_move(gripper_client, gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_2,wait = True)

            move_group.go(joint_3,wait = True)
            gripper_move(gripper_client, gripper_parallel_open, gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 2:
            #joints = joints_home
            gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 3:
            #joints = joints_demo
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 4:
            move_group.go(joint_first,wait = True)
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top,wait = True)
            move_group.go(joint_grip,wait = True)
            gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top,wait = True)
            rospy.sleep(1)
            move_group.go(joint_top2,wait = True)
            move_group.go(joint_release,wait = True)
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top2,wait = True)
            move_group.go(joint_top,wait = True)
            move_group.go(joint_first,wait = True)
            
        elif input == 5
            
            print("True")
        else:
            break

        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

        gripper_move(gripper_client, gripper_width,
                     gripper_parallel_speed, gripper_parallel_effort)
        #arm_move(move_group, joints)"""

    print("Bye...")
