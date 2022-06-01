#!/usr/bin/env python
import os
import rospy
from std_srvs.srv import Empty, EmptyResponse
CMD = "ssh -i id_rsa_myCobot pi@pi python3 btr_myCobot.py"

def grab_Arm(data):
    cmd = CMD + " move_g"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

def release_Arm(data):
    cmd = CMD + " move_r"
    print(cmd)
    os.system(cmd)
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node("BTR_myCobot-ros")
    src00 = rospy.Service('/btr/move_g',Empty,grab_Arm)
    src01 = rospy.Service('/btr/move_r',Empty,release_Arm)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

    print("Bye...")
