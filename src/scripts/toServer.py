#!/usr/bin/python3
# -*- coding: utf-8 -*-
from get_number_pose_1 import get_number_pose_ddd
from get_number_pose_1 import get_number_pose_1
from get_number_pose_1 import get_box_pose_ddd ,get_box_pose_ddd_pre
from get_number_class_1 import get_number_class_1
from get_number_class_pose_3 import get_number_class_pose_3

import numpy as np
import cv2
from geometry_msgs.msg import Pose
import rospy
import inspect
import ctypes
from grasp_cube_ddd import graspAruco
from place_cube_ddd import placeAruco

class toServer():
    def __init__(self):
        self.case1_target_numbers = get_number_class_pose_3
        self.case2_number_class = get_number_class_1
        # self.case2_number_pose = get_number_pose_ddd
        self.case2_number_pose = get_number_pose_1
        self.case3_box_pose_ddd = get_box_pose_ddd
        self.case3_box_pose_pre = get_box_pose_ddd_pre

        self.case3_box_class_pose = get_number_class_pose_3
        self.pub_pose = rospy.Publisher('aruco_pose', Pose, queue_size=1)
        self.grasp = graspAruco()
        self.place = placeAruco()
        self.r = 0.045
        self.model_object = np.array([(0-0.5*self.r, 0-0.5*self.r, 0.0),
                                      (self.r-0.5*self.r, 0-0.5*self.r, 0.0),
                                      (self.r-0.5*self.r, self.r-0.5*self.r, 0.0),
                                      (0-0.5*self.r, self.r-0.5*self.r, 0.0)])
        self.camera_matrix = np.array([
            (617.3054000792732, 0.0, 424.0),
            (0.0, 617.3054000792732, 240.0),
            (0, 0, 1)],
            dtype="double")
        self.dist_coeffs = np.array([[0, 0, 0, 0]], dtype="double")
        self.pose_msg = Pose()

    def point2xyz(self, points):
        points_f = points.astype('float')
        quads_prj = []
        rvec_list = []
        tvec_list = []
        ret, rvec, tvec = cv2.solvePnP(
            self.model_object, points_f, self.camera_matrix, self.dist_coeffs)
        projectedPoints, _ = cv2.projectPoints(
            self.model_object, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        err = 0
        for t in range(len(projectedPoints)):
            err += np.linalg.norm(projectedPoints[t]-points_f[t])
        area = cv2.contourArea(points)
        if err/area < 0.005:
            quads_prj.append(projectedPoints.astype(int))
            rvec_list.append(rvec)
            tvec_list.append(tvec)
        return tvec_list, rvec_list

    def tvec2msg(self, msg, tvec_list):
        msg.position.x = tvec_list[0][0]
        msg.position.y = tvec_list[0][1]
        msg.position.z = tvec_list[0][2]
        msg.orientation.w = 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        return msg

    def pub_point2msg(self, points):
        tvec_list, _ = self.point2xyz(points)
        self.pose_msg = self.tvec2msg(self.pose_msg, tvec_list)
        self.pub_pose.publish(self.pose_msg)
        return self.pose_msg
