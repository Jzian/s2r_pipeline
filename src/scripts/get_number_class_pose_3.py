#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from predict import predict
from get_roi_3 import get_roi_3
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


def cv_show(img):
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows


def get_number_class_pose_3(method):
    bridge = CvBridge()
    data = rospy.wait_for_message('/camera/color/image_raw', Image)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as err:
        print(err)
    # cv_image = cv2.imread(
    #     '/home/xcy/kevin_ws/ros_ws/src/detect_grasp_place_service/scripts/total_scene.png')
    img = cv_image
    rois, pos = get_roi_3(img, method)
    numbers_sort_x = []
    numbers_sort_index = []
    for p in pos:
        numbers_sort_x.append(p[0][0][0])
    numbers_sort_index = sorted(
        range(len(numbers_sort_x)), key=lambda k: numbers_sort_x[k])
    cls = []
    cls_np = []
    index = []
    for roi in rois:
        cc = predict(roi)
        cls.append(cc)
        cls_np.append(cc.numpy())
    for cls_n in cls_np:
        index.append(np.argmax(cls_n))
    index = [index[i]+1 for i in numbers_sort_index]
    pos = [pos[i] for i in numbers_sort_index]
    # if max(cls_np) < 0.2:
    #     index = [9, 9, 9]
    #     pos = []
    return index, pos
