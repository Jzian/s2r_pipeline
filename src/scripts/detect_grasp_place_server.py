#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
from s2r_pipeline.srv import TargetNumber,TargetNumberResponse
import rospy
import threading
from toServer import toServer
# from target_number_pose_service.msg import pose
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8
import numpy as np
import cv2
from pose_detect import *

class detect_grasp_place_server():
    def __init__(self):
        rospy.init_node('detect_grasp_place_server')
        self.service = rospy.Service('/detect_grasp_place_service',
                                     TargetNumber, self.targetCallback)

        self.target_number_pose = np.array([])
        self.target_numbers_pose = np.array([])
        self.target_number = 0
        self.target_numbers = [9,9,9]
        self.grasp_flag = False
        self.place_flag = False
        self.toServer = toServer()
        self.request_nubmer=0
        # self.pub_grasp_pose_thread = threading.Thread(
        #     target=self.pub_grasp_pose)
        # self.pub_grasp_pose_thread.start()

    def targetCallback(self, req):
        print(req.work_case)
        if req.work_case == 1:
            self.target_numbers = self.toServer.case1_target_numbers('target_number')[
                0]
            if self.target_numbers == [9, 9, 9]:
                return TargetNumberResponse(False, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])
            else:
                print(self.target_numbers)
                return TargetNumberResponse(True, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])
        elif req.work_case == 2 and req.number == self.toServer.case2_number_class():
            # elif req.work_case == 2:
            self.grasp_flag = False
            # self.target_number_pose = self.toServer.case2_number_pose()
            # if self.target_number_pose.shape[0] == 4:
            print('Ready to grasp')
            self.start_grasp()
            if self.grasp_flag:
                self.grasp_flag = False
                print('Finish grasp')
                return TargetNumberResponse(True, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])

        elif req.work_case == 3:
            self.place_flag = False
            print('req.number:',req.number)
            self.request_nubmer=req.number-1
            # self.target_number = self.toServer.case2_number_class()
            # self.target_number_pose = self.toServer.case2_number_pose()
            # self.target_numbers,self.target_numbers_pose=self.toServer.case3_box_class_pose('box')
            # if self.target_number_pose.shape[0] == 4:
            print('Ready to place')
            self.start_place()
            if self.place_flag:
                self.place_flag = False
                print('send place success')
                return TargetNumberResponse(True, self.target_numbers[0],self.target_numbers[1],self.target_numbers[2])

    def thread_job(self):
        print("Ready to work by case")
        rospy.spin()

    def start_grasp(self):
        print('start place')
        # rate = rospy.Rate(30)
        print('self.grasp_flag:', self.grasp_flag)
        rate = rospy.Rate(100)
        print("init")
        self.toServer.grasp.reset_arm()
        print("reset arm at beginning")
        self.toServer.grasp.open_gripper()
        print("open gripper at beginning")
        while not self.grasp_flag:
            self.target_number_pose = self.toServer.case2_number_pose()[  #read four cornor point 
                :, 0, :]
            self.toServer.pub_point2msg(self.target_number_pose) # based on camera xyz
            self.toServer.grasp.graspCallback(self.toServer.pose_msg)
            if self.toServer.grasp.grasp_success: #grasp success 
                print('finish grasp')
                self.toServer.grasp.grasp_success = False
                self.grasp_flag = True
        rate.sleep()

        
        # while not self.grasp_flag:
        #     self.toServer.pose_msg = self.toServer.case2_number_pose()
        #     self.toServer.grasp.graspCallback(self.toServer.pose_msg)
        #     if self.toServer.grasp.grasp_success:
        #         print('finish grasp')
        #         self.toServer.grasp.grasp_success = False
        #         self.grasp_flag = True
        # rate.sleep()

    def start_place(self):
        number_of_box=self.request_nubmer
        print('start_place_number_of_box:',number_of_box)
        print('start pub grasp_pose')
        # rate = rospy.Rate(30)
        # print('self.grasp_flag:', self.grasp_flag)
        # while not rospy.is_shutdown():
        #     if not self.grasp_flag:
        #         self.target_number_pose = self.toServer.case2_number_pose()[
        #             :, 0, :]
        #         self.toServer.pub_point2msg(self.target_number_pose)
        #     else:
        #         pass
        #     rate.sleep()
        rate = rospy.Rate(100)
        print("=====init=====")
        print("=====reset arm at beginning=====")
        print("=====open gripper at beginning=====")
        while not self.grasp_flag:
            self.toServer.pose_msg = self.toServer.case3_box_pose_ddd(number_of_box)
            self.toServer.place.sinkCallback(self.toServer.pose_msg)
            if self.toServer.place.place_success:
                print('finish place')
                self.toServer.place.place_success = False
                self.place_flag = True
        rate.sleep()



if __name__ == "__main__":
    load_template()
    detect = detect_grasp_place_server()
    add_thread = threading.Thread(target=detect.thread_job)
    add_thread.start()
    add_thread.setName('server_loop')
    
    rate = rospy.Rate(100)
    pub_message = Pose()
    while not rospy.is_shutdown():
        # grasp()
        rate.sleep()
