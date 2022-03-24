#!/usr/bin/python3
# -*- coding: utf-8 -*-
from get_number_pose_1 import get_number_pose_1
from get_number_pose_1 import get_box_pose_ddd, get_box_pose_ddd_pre
from get_number_class_1 import get_number_class_1
from get_number_class_pose_3 import get_number_class_pose_3

import numpy as np
import cv2
from geometry_msgs.msg import Pose
import rospy
from place_cube_ddd import placeAruco
from grasp_cube_kevin import grasp_cube_kevin


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
        self.place = placeAruco()
        self.grasp_place = grasp_cube_kevin()
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

