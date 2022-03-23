#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import rospy
from s2r_pipeline.srv import TargetNumber,TargetNumberRequest

def detect_grasp_place_client():
    rospy.init_node('detect_grasp_place_client')
    rospy.wait_for_service('/detect_grasp_place_service')
    try:
        detect_grasp_place_request = rospy.ServiceProxy(
            '/detect_grasp_place_service', TargetNumber)
        req = TargetNumberRequest()
        req.work_case = 3
        req.number = 4
        response = detect_grasp_place_request(req)
        return response
    except(rospy.ServiceException):
        print("Service call failed")


if __name__ == "__main__":
    print(detect_grasp_place_client())
