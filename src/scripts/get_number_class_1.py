#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from predict_1 import predict_1
from get_roi_one import get_roi_1
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


def cv_show(img):
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows


def get_number_class_1():
    bridge = CvBridge()
    data = rospy.wait_for_message('/camera/color/image_raw', Image)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as err:
        print(err)

    img = cv_image
    # img = cv2.imread(
    #     '/home/xcy/kevin_ws/ros_ws/src/detect_grasp_place_service/scripts/1.png')
    roi, pos = get_roi_1(img, 'number')
    cc = predict_1(roi).numpy()
    index = np.argmax(cc)
    if cc[index] < 0.2:
        index = 99
    return index+1
