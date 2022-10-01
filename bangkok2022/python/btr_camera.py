import cv2
from cv2 import aruco
import numpy as np
import time
import rospy
import rosservice
import os
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from rcll_btr_msgs.msg import PictureInfoResponse
from rcll_btr_msgs.srv import PictureInfo

def initCamera():
    global cap
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

def getPicture(data):
    global cap, bath_path, n, ext
    ret, frame1 = cap.read()
    frame2 = cv2.resize(frame1, (640, 480))
    pictureInfo = PictureInfoResponse()
    pictureInfo.ok = ret
    pictureInfo.filename.data = String()
    pictureInfo.filename.data = '{}_{:0>4}.{}'.format(base_path, n, ext)
    cv2.imwrite(pictureInfo.filename.data, frame2)
    n += 1
    print(pictureInfo.filename)
    return pictureInfo

def finishCamera():
    global cap, cameraFlag
    cameraFlag = False
    
if __name__ == "__main__":
    initCamera()
    rospy.init_node('btr_camera')
    cameraFlag = True
    srv01 = rospy.Service('btr_camera/picture', PictureInfo, getPicture)
    rate = rospy.Rate(10)
    base_path = os.getcwd() + "/../pictures/btr-camera"
    n = 0
    ext = "jpg"

    while not rospy.is_shutdown():
        if cameraFlag == False:
            break
        rate.sleep()
