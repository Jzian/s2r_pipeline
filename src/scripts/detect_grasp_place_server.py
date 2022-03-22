#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from s2r_pipeline.srv import TargetNumber, TargetNumberResponse
import rospy
import threading
from toServer import toServer
# from target_number_pose_service.msg import pose
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8
import numpy as np
import cv2
from grasp_cube_ddd import grasp
from get_number_pose_1 import get_number_pose_1


class detect_grasp_place_server():
    def __init__(self):
        rospy.init_node('detect_grasp_place_server')
        self.service = rospy.Service('/detect_grasp_place_service',
                                     TargetNumber, self.targetCallback)

        self.target_number_pose = np.array([])
        self.target_numbers_pose = np.array([])
        self.target_number = 0
        self.target_numbers = []
        self.toServer = toServer()
        self.pub_grasp_pose_thread = threading.Thread(
            target=self.pub_grasp_pose)

    def targetCallback(self, req):
        print(req.work_case)
        if req.work_case == 1:
            self.target_numbers = self.toServer.case1_target_numbers()[0]
            if self.target_numbers == [9, 9, 9]:
                return TargetNumberResponse(False, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])
            else:
                print(self.target_numbers)
                return TargetNumberResponse(True, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])
        elif req.work_case == 2 and req.number == self.toServer.case2_number_class():
            # elif req.work_case == 2:
            self.target_number = self.toServer.case2_number_class()
            self.target_number_pose = self.toServer.case2_number_pose()[
                :, 0, :]
            if self.target_number_pose.shape[0] == 4:
                print('Ready to grasp')
                self.pub_grasp_pose_thread.start()
                grasp_flag = grasp()
                if grasp_flag:
                    # self.toServer.stop_thread(self.pub_grasp_pose_thread)
                    return TargetNumberResponse(True, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])

        elif req.work_case == 3:
            pass
        else:
            print('work_case is not fit')
            return TargetNumberResponse(False, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])

    def thread_job(self):
        print("Ready to work by case")
        rospy.spin()

    def pub_grasp_pose(self):
        print('start pub grasp_pose')
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.target_number_pose = self.toServer.case2_number_pose()[
                :, 0, :]
            print(self.target_number_pose)
            self.toServer.pub_point2msg(self.target_number_pose)
            rate.sleep()


if __name__ == "__main__":
    detect = detect_grasp_place_server()
    add_thread = threading.Thread(target=detect.thread_job)
    add_thread.start()
    rate = rospy.Rate(100)
    pub_message = Pose()
    while not rospy.is_shutdown():
        # grasp()
        rate.sleep()
