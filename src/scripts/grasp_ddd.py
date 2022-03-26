import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from pose_detect import *    # image processing by cython
from grasp_cube_kevin import grasp_cube_kevin
from pyquaternion import Quaternion


def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0]
    aruco_pose_msg.position.y = tvec[1]
    aruco_pose_msg.position.z = tvec[2]
    aruco_pose_msg.orientation.x = rvec[0]
    aruco_pose_msg.orientation.y = rvec[1]
    aruco_pose_msg.orientation.z = rvec[2]
    aruco_pose_msg.orientation.w = rvec[3]
    return aruco_pose_msg


def tvec2msg(rvec):
    r_matrix = cv2.Rodrigues(rvec)[0]
    quat = list(Quaternion(matrix=np.array(r_matrix)))
    return quat


def get_number_pose_ddd():
    bridge = CvBridge()
    data = rospy.wait_for_message('/camera/color/image_raw', Image)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as err:
        print(err)
    # cv_image = cv2.imread(
    #     '/home/xcy/kevin_ws/ros_ws/src/detect_grasp_place_service/scripts/1.png')
    img = cv_image

    seg_papram = np.array([0, 15, 125, 180, 46, 80], dtype="uint8")

    id_list = []
    # id_list,tvec_list,rvec_list= marker_detection(cv_image,seg_papram)
    id_list, tvec_list, rvec_list, point_list = pose_detection(
        cv_image, seg_papram)
    return point_list

    # cv2.imshow('frame', cv_image)
    # cv2.waitKey(1)

    # publish pose to rostopic
    # target_detected = False
    # aruco_pose_msg_list = [0, 0, 0]
    # field_top = -0.15
    # msg_list = []
    # for i in range(len(id_list)):
    #     rvec = tvec2msg(rvec=rvec_list[i])
    #     aruco_pose_msg = pose_aruco_2_ros(rvec, tvec_list[i])
    #     if (aruco_pose_msg.position.y > field_top) and (aruco_pose_msg.position.y < 0.2):
    #         msg_list.append(aruco_pose_msg)
    # if(len(id_list) == 1):
    #     return(msg_list[0])
    # elif(len(id_list) == 2):
    #     if area_list[0] > area_list[1]:
    #         return msg_list[0]
    #     else:
    #         return msg_list[1]
    # else:
    #     print(len(id_list))
    #     print("ffffffffffffffffff")
    #     return msg_list[1]
