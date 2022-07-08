import cv2
from cv2 import aruco
import numpy as np
import time
import rospy
import rosservice
from std_srvs.srv import Empty, EmptyResponse
from rcll_btr_msgs.msg import Corners, TagInfoResponse, PictureInfoResponse
from rcll_btr_msgs.srv import TagInfo, PictureInfo

def initAruco():
    global dict_aruco, parameters
    # cap = cv2.VideoCapture(0)
    dict_aruco = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

def getAruco(data):
    global cap, dict_aruco, parameters
    pictureInfo = PictureInfo()
    rospy.wait_for_service('/btr_camera/picture')
    videoCapture = rospy.ServiceProxy('/btr_camera/picture', PictureInfo)
    picture = videoCapture()
    print(picture.filename.data)
    
    frame = cv2.imread(picture.filename.data)
    # ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    tagInfo = TagInfoResponse()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
    print(ids,corners)

    if (len(ids) == 0):
        tagInfo.tag_id.data = 0
        tagInfo.ok = False
    else:
        tagInfo.tag_id.data = int(ids[0])
        tagInfo.UpLeft.x,      tagInfo.UpLeft.y      = corners[0][0][0][0], corners[0][0][0][1]
        tagInfo.UpRight.x,     tagInfo.UpRight.y     = corners[0][0][1][0], corners[0][0][1][1]
        tagInfo.BottomRight.x, tagInfo.BottomRight.y = corners[0][0][2][0], corners[0][0][2][1]
        tagInfo.BottomLeft.x,  tagInfo.BottomLeft.y  = corners[0][0][3][0], corners[0][0][3][1]
        tagInfo.ok = True

    print(tagInfo)
    return tagInfo

if __name__ == "__main__":
    initAruco()
    rospy.init_node('btr_aruco')
    arucoFlag = False
    srv01 = rospy.Service('btr_aruco/TagInfo', TagInfo, getAruco)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
