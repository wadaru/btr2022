import cv2
from cv2 import aruco
import numpy as np
import time
import math

cap = cv2.VideoCapture(0)
dict_aruco = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters = aruco.DetectorParameters_create()

while True:
    ret, frame1 = cap.read()
    frame = cv2.resize(frame1, (640,480))
    gray = frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
    aruco.drawDetectedMarkers(gray, corners, ids, (0,255,255))
    cv2.imwrite("../pictures/test.jpg", gray)
    # print(ids,corners)

    if not(ids is None):
        marker_length = 0.13
        camera_matrix = np.array([[614.72443761,   0.        , 329.52316472],
                                  [  0.        , 613.30068366, 199.92578538],
                                  [  0.        ,   0.        ,   1.        ]])
        distortion_coeff = np.array([ 0.13505291, -0.29420201, -0.00645303,  0.00196495, -0.01754147])

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, distortion_coeff)
        tvec = np.squeeze(tvec)
        rvec = np.squeeze(rvec)
        rvec_matrix = cv2.Rodrigues(rvec)
        rvec_matrix = rvec_matrix[0] # rodoriguesから抜き出し
        transpose_tvec = tvec[np.newaxis, :].T
        proj_matrix = np.hstack((rvec_matrix, transpose_tvec))
 
        # オイラー角への変換
        euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
        print("x : " + str(tvec[0]))
        print("y : " + str(tvec[1]))
        print("z : " + str(tvec[2]))

        #カメラ座標系におけるx軸の回転角度　
        print("roll : " + str(euler_angle[0]))
        #カメラ座標系におけるy軸の回転角度
        print("pitch: " + str(euler_angle[1]))
        #カメラ座標系におけるz軸の回転角度
        print("yaw  : " + str(euler_angle[2]))

        # draw
        img = frame
        draw_pole_length = marker_length / 2
        aruco.drawAxis(img, camera_matrix, distortion_coeff, rvec, tvec, draw_pole_length)
        cv2.imwrite("../pictures/axis.jpg", img)
        cv2.imshow("aruco", img)
    else:
        cv2.imshow("aruco", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

