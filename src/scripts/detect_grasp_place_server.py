#!/usr/bin/python3
# -*- coding: utf-8 -*-
# kevin_node
from s2r_pipeline.srv import TargetNumber, TargetNumberResponse
import rospy
import threading
from toServer import toServer
# from target_number_pose_service.msg import pose
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8
import numpy as np
import cv2
from pose_detect import *
from grasp_ddd import get_number_pose_ddd
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class detect_grasp_place_server():
    def __init__(self):
        rospy.init_node('detect_grasp_place_server')
        self.service = rospy.Service('/detect_grasp_place_service',
                                     TargetNumber, self.targetCallback)

        self.target_number_pose = np.array([])
        self.target_numbers_pose = np.array([])
        self.target_number = 0
        self.target_numbers = [9, 9, 9]
        self.grasp_flag = False
        self.place_flag = False
        self.toServer = toServer()
        self.request_nubmer = 0
        self.place_box_number = 0
        self.judge_state_distance = 15
        self.judge_state_center = -1
        self.grasp_position_fit_flag = False

    def targetCallback(self, req):
        print(req.work_case)
        if req.work_case == 1:
            self.target_numbers = self.toServer.case1_target_numbers('target_number')[
                0]
            if self.target_numbers == [9, 9, 9]:
                return TargetNumberResponse(False, self.target_numbers[0], self.target_numbers[1], self.target_numbers[2])
            else:
                print(self.target_numbers)
                self.toServer.grasp_place.move_right_by_distance(0.2)
                # return TargetNumberResponse(True, 2, 1, 4)
                return TargetNumberResponse(True, self.target_numbers[0], self.target_numbers[1], self.target_numbers[2])
        elif req.work_case == 7 and req.number == self.toServer.case2_number_class():
            # elif req.work_case == 2:
            self.grasp_flag = False
            # self.target_number_pose = self.toServer.case2_number_pose()
            # if self.target_number_pose.shape[0] == 4:
            print('Ready to grasp')
            self.start_grasp()
            if self.grasp_flag:
                self.grasp_flag = False
                print('Finish grasp')
                return TargetNumberResponse(True, self.target_numbers[0], self.target_numbers[1], self.target_numbers[2])
        elif req.work_case == 6:
            try:
                self.place_flag = False
                print('req.number:', req.number)
                self.request_nubmer = req.number
                print('Ready to place')
                self.start_place()
            except Exception:
                print('zhou postion is not right')
                self.toServer.grasp_place.move_function_xy(-0.1, 0)
                self.prepare_place_debug()
                self.start_place()
            if self.place_flag:
                self.place_flag = False
                print('send place success')
                return TargetNumberResponse(True, 9, 9, 9)

        elif req.work_case == 2:
            # local_flag = True
            # while local_flag:
            #     try:
            #         self.target_number_pose = self.toServer.case2_number_pose()
            #         local_flag = False
            #     except Exception:
            #         self.toServer.grasp_place.move_forward_by_distance(-0.2)
            self.grasp_kevin(req.number)
            self.judge_state_distance = 0
            self.judge_state_center = 0
            if self.grasp_position_fit_flag:
                self.grasp_position_fit_flag = False
                return TargetNumberResponse(True, 7, 7, 7)
            return TargetNumberResponse(True, 9, 9, 9)
        elif req.work_case == 3:
            place_number = req.number
            self.place_flag = False
            rate = rospy.Rate(30)
            while not self.place_flag:
                try:
                    self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
                        'box')
                    for i in range(len(self.target_numbers)):
                        self.target_numbers[i] -= 5
                    for j in range(len(self.target_numbers_pose)):
                        self.target_numbers_pose[j] = self.target_numbers_pose[j][:, 0, :]
                    box_pose_dict = dict(
                        zip(self.target_numbers, self.target_numbers_pose))
                    place_box_number_pose = box_pose_dict[place_number]
                    self.toServer.grasp_place.pose_msg = self.toServer.grasp_place.point2msg(
                        place_box_number_pose)
                    center = self.toServer.grasp_place.pose_msg.position.x
                    distance = self.toServer.grasp_place.pose_msg.position.z
                    print(self.toServer.grasp_place.pose_msg.orientation)
                    print(center, distance)
                    self.toServer.grasp_place.forward_to_box(
                        center, distance)
                    # self.toServer.grasp_place.place_cube()
                    print(place_box_number_pose)
                except Exception:
                    self.toServer.grasp_place.move_forward_by_distance(0.1)
                if self.toServer.grasp_place.place_success:
                    self.place_flag = True
                    self.toServer.grasp_place.place_success = False
                rate.sleep()
        elif req.work_case == 5:
            self.toServer.grasp_place.drop_cube()

    def thread_job(self):
        print("Ready to work by case")
        rospy.spin()

    def start_place(self):
        number_of_box = self.request_nubmer
        print('start_place_number_of_box:', number_of_box)
        print('start pub grasp_pose')
        rate = rospy.Rate(50)
        print("=====init=====")
        print("=====reset arm at beginning=====")
        print("=====open gripper at beginning=====")

        self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
            'box')
        # while(len(self.target_numbers) <= 2):
        #     print('correct_by_kevin')
        #     self.toServer.grasp_place.move_function_z(0.2)
        #     self.toServer.grasp_place.move_function_xy_ddd(0, 3)
        #     self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
        #         'box')

        self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
            'box')
        # if len(self.target_numbers)<=2:
        #     self.toServer.grasp_place.move_left_by_distance(0.11)
        #     rospy.sleep(1.5)

        while(self.toServer.case3_box_pose_ddd
              (number_of_box).position.z > 0.6):
            print('correct_by_ddd')
            self.toServer.grasp_place.move_function_xy_ddd(3, 0)

        while not self.place_flag:
            self.toServer.pose_msg = self.toServer.case3_box_pose_ddd(
                number_of_box)
            self.toServer.place.sinkCallback(self.toServer.pose_msg)
            if self.toServer.place.place_success:
                print('finish place')
                self.toServer.place.place_success = False
                self.place_flag = True
        rate.sleep()

    def point_list_max_top(self, point_list):
        area_size_max = 0  # 面积最大区域中间量
        area_size_list = []  # 所有区域的面积列表
        for cnt in point_list:
            current_area_size = cv2.contourArea(cnt)
            area_size_list.append(current_area_size)
            if current_area_size > area_size_max:
                area_size_max = current_area_size
        top_index = sorted(
            range(len(area_size_list)), key=lambda k: area_size_list[k])
        top_index.reverse()
        roi_box = []
        for i, num in enumerate(top_index):
            if area_size_list[num] > 400:
                roi_box_temp = cv2.approxPolyDP(
                    point_list[num], 3, True)
                roi_box.append(roi_box_temp)
        numbers_sort = []
        roi_size = []
        for p in roi_box:
            points = np.squeeze(p)
            x, y = np.split(points, 2, axis=1)
            x, y = np.squeeze(x), np.squeeze(y)
            numbers_sort.append(max(y))
            roi_size.append(cv2.contourArea(p))
        numbers_sort_index = sorted(
            range(len(numbers_sort)), key=lambda k: numbers_sort[k])
        roi_size_index = sorted(
            range(len(roi_size)), key=lambda k: roi_size[k], reverse=True)
        for i, sub_roi_size_index in enumerate(roi_size_index):
            for sub_numbers_sort_index in numbers_sort_index:
                if sub_numbers_sort_index == sub_roi_size_index:
                    return roi_box[roi_size_index[i]]

    def grasp_kevin(self, number):
        self.grasp_flag = False
        rate = rospy.Rate(100)
        self.toServer.grasp_place.set_arm()
        while not self.grasp_flag:
            try:
                # if self.toServer.case3_box_class_pose('number2')[0][0] == 2:
                #     self.target_number_pose = self.toServer.case3_box_class_pose('number2')[
                #         1][0]
                # else:

                # #-----------------KAIWEN___________________
                # self.target_number_pose = self.toServer.case2_number_pose()
                # pose = self.target_number_pose
                # self.toServer.grasp_place.pose_msg = self.toServer.grasp_place.point2msg(
                #     pose)
                if number == 2:
                    point_list = get_number_pose_ddd()
                    self.target_number_pose = self.point_list_max_top(
                        point_list)
                    pose = self.target_number_pose
                else:
                    self.target_number_pose = self.toServer.case2_number_pose()
                    pose = self.target_number_pose
                self.toServer.grasp_place.pose_msg = self.toServer.grasp_place.point2msg(
                    pose)
                center = self.toServer.grasp_place.pose_msg.position.x
                distance = self.toServer.grasp_place.pose_msg.position.z
                self.judge_state_distance = distance
                self.judge_state_center = center
                # print(center, distance)
                self.toServer.grasp_place.forward_to_cube(
                    center, distance, self.toServer.grasp_place.pose_msg.orientation)
            except Exception as e:
                print('the img is wrong')
                self.toServer.grasp_place.move_forward_by_distance(-0.2)
                rospy.sleep(0.5)
                if self.toServer.case2_number_pose()[:, 0, :].shape[0] != 4 and self.judge_state_distance > 0.20 and self.toServer.case2_number_class() != 2:
                    self.toServer.grasp_place.move_forward_by_distance(0.1)
                    rospy.sleep(1)
                    print('not ready to grasp,but one frame is lost')
                    self.grasp_position_fit_flag = True
                elif self.toServer.case2_number_pose()[:, 0, :].shape[0] != 4 and self.judge_state_center < 0 and self.judge_state_distance < 0.20:
                    print('ready to grasp cube,but can not see the cube')
                    self.toServer.grasp_place.move_function_z(-1)
                    self.toServer.grasp_place.move_forward_by_distance(-0.1)
                    self.grasp_position_fit_flag = True
                elif self.toServer.case2_number_pose()[:, 0, :].shape[0] != 4 and self.judge_state_center > 0 and self.judge_state_distance < 0.20:
                    print('ready to grasp cube,but can not see the cube')
                    self.toServer.grasp_place.move_function_z(1)
                    self.toServer.grasp_place.move_forward_by_distance(-0.1)
                    self.grasp_position_fit_flag = True
                if self.toServer.case2_number_class() == 2:
                    print('the number is 2,fkfkfkfk')
                    self.toServer.grasp_place.move_forward_by_distance(0.11)
            finally:
                if self.toServer.grasp_place.grasp_success:
                    self.grasp_flag = True
                    self.toServer.grasp_place.grasp_success = False
            rate.sleep()
        if self.grasp_flag:
            self.grasp_flag = False
            return True

    def prepare_place_debug(self):
        numbers_pose = []
        numbers_distance = []
        self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
            'box')
        while len(self.target_numbers) != 3:
            numbers_pose = []
            numbers_distance = []
            numbers_center = []
            print('box is not 3,is:', len(self.target_numbers))
            self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
                'box')
            for i in range(len(self.target_numbers)):
                self.target_numbers[i] -= 5
            for pose in self.target_numbers_pose:
                self.toServer.pose_msg = self.toServer.grasp_place.point2msg(
                    pose)
                numbers_pose.append(self.toServer.pose_msg.orientation)
                numbers_distance.append(
                    self.toServer.pose_msg.position.z)
                numbers_center.append(
                    self.toServer.pose_msg.position.x)
            box_pose_dict = dict(
                zip(self.target_numbers, numbers_pose))
            average_distance = np.mean(numbers_distance)
            print('average_distance:', average_distance)
            box_center_dict = dict(
                zip(self.target_numbers, numbers_center))

            if not 1 in box_pose_dict:
                self.toServer.grasp_place.move_left_by_distance(0.1)
                print('can not see b,move left')
                rospy.sleep(0.1)
            if not 3 in box_pose_dict:
                self.toServer.grasp_place.move_right_by_distance(0.1)
                print('can not see x,move right')
                rospy.sleep(0.1)
            if 2 in box_pose_dict:
                print('o center:', box_center_dict[2])
            if average_distance < 0.3:
                print('im too close to the box')
                self.toServer.grasp_place.move_function_xy(
                    -1-average_distance, 0)
                rospy.sleep(1)
                if 2 in box_center_dict:
                    if box_center_dict[2] > 0:
                        self.toServer.grasp_place.move_right_by_distance(
                            0.1)
                        rospy.sleep(0.1)
                        print('o is on my right ,move right')
                    elif box_center_dict[2] < 0:
                        self.toServer.grasp_place.move_left_by_distance(
                            0.1)
                        rospy.sleep(0.1)
                        print('o is on my left ,move left')
            self.target_numbers, self.target_numbers_pose = self.toServer.case3_box_class_pose(
                'box')


if __name__ == "__main__":
    load_template()
    detect = detect_grasp_place_server()
    add_thread = threading.Thread(target=detect.thread_job)
    add_thread.start()
    rate = rospy.Rate(100)
    pub_message = Pose()
    while not rospy.is_shutdown():
        # grasp()
        rate.sleep()
