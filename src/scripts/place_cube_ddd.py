#!/usr/bin/python3
# Software License Agreement (BSD License)
import re
import rospy
import numpy as np
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import sys
import cv2
from pyquaternion import Quaternion


class placeAruco:
    def __init__(self):
        self.base_move_position_pub = rospy.Publisher(
            "cmd_position", Twist, queue_size=10)
        self.base_move_vel_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=10)
        self.arm_gripper_pub = rospy.Publisher(
            "arm_gripper", Point, queue_size=10)
        self.arm_position_pub = rospy.Publisher(
            "arm_position", Pose, queue_size=10)
        self.image_sub = rospy.Subscriber(
            "/aruco_sink2", Pose, self.sinkCallback, queue_size=1)

        self.place_success = False
        self.base_vel = 0.3

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        print("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        print("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self, t_vector):
        print("move_arm", t_vector)
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.2      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def move_arm0(self, t_vector):
        print("move_arm", t_vector)
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.90      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.09
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        print("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)

    def move_base_x(self, t_vector):
        move_base_msg_x = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        # goal = [0.03, 0.0, 0.115]
        # distance_x = t_vector[2]-goal[2]
        # # x_move = 0.07
        # if distance_x <= 0.05:
        #     x_move = 0
        # else:
        x_move = 0.05
        # x_move = 0.7 * distance_x
        # print("x_movement", x_move)
        move_base_msg_x.linear.x = x_move
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in x direction")
        self.base_move_position_pub.publish(move_base_msg_x)

    def forward_zero(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)

    def forward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.11
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def backward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = -0.11
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_right(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = -0.11
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_left(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.11
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def move_function_xy(self, x, y):
        vel_cmd = Twist()
        vel_cmd.linear.x = x * self.base_vel
        vel_cmd.linear.y = y * self.base_vel
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

        rospy.sleep(0.01)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def move_base_velocity_x(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_x()
            # for tt in range(int(n_forward_zero)):
            #     self.forward_zero()

    def move_forward_by_distance(self, distance):
        move_base_msg_x = Twist()
        move_base_msg_x.linear.x = distance
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in forward")
        self.base_move_position_pub.publish(move_base_msg_x)

    def move_base_velocity_back(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.backward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_right(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_right()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_left(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_left()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_y(self):
        move_base_msg_y = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        # goal = [0.03, 0.0, 0.115]
        y_move = 0.05
        move_base_msg_y.linear.x = 0.0
        # move_base_msg_y.linear.y = goal[0] - t_vector[0]
        move_base_msg_y.linear.y = y_move
        move_base_msg_y.linear.z = 0.0
        move_base_msg_y.angular.x = 0.0
        move_base_msg_y.angular.y = 0.0
        move_base_msg_y.angular.z = 0.0
        print("move_base_y 5.0 cm")
        self.base_move_position_pub.publish(move_base_msg_y)

# ---- The code below is written by myself
    def distance_funtion(self, distance):
        d = distance*1
        if (d > 0) and (d < 1):
            d = 1
        if (d < 0) and (d > -1):
            d = -1
        return d

    def my_move_function(self, dx, dy):
        vel_cmd = Twist()
        vel_cmd.linear.x = dx * self.base_vel
        vel_cmd.linear.y = -dy * self.base_vel
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

        rospy.sleep(0.01)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def q2e(self, quat):
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        r = r / math.pi * 180
        p = math.asin(2 * (w * y - z * x))
        p = p / math.pi * 180
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        y = y / math.pi * 180
        return r, p, y

    # def tvec2msg(self,rvec):
    #     r_matrix = cv2.Rodrigues(rvec)[0]
    #     quat = list(Quaternion(matrix=np.array(r_matrix)))
    #     return quat

    def move_function_z(self, z):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = z
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.01)
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

# ------------------------------------------------

    def sinkCallback(self, data):

        if self.place_success == True:
            return

        gama_x = 0.003
        gama_y = 0.009

        tvec = [0, 0, 0]
        tvec[0] = data.position.x
        tvec[1] = data.position.y
        tvec[2] = data.position.z

        rvec = [0, 0, 0, 0]
        rvec[0] = data.orientation.x
        rvec[1] = data.orientation.y
        rvec[2] = data.orientation.z
        rvec[3] = data.orientation.w

        _, rotate_goal, _ = self.q2e(rvec)

        goal = [0.0305, 0.0, 0.2]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = tvec[0] - goal[0]
        # print("distance in x", distance_in_x)
        # print("distance in y", distance_in_y)

        if (distance_in_x <= gama_x) and (abs(distance_in_y) <= gama_y) and (abs(rotate_goal) < 1.3):
            # step forward
            self.move_base_velocity_x(b_vector=0.2, duration=13)

            print("===== start placing ====")
            # self.reset_arm()
            # rospy.sleep(0.2)
            # self.move_arm0(tvec)
            # rospy.sleep(0.5)
            self.move_arm(tvec)
            rospy.sleep(0.5)
            self.open_gripper()
            rospy.sleep(0.5)
            self.forward_zero()
            # rospy.sleep(1)
            print("===== finish ====")
            # self.move_base_velocity_back(b_vector=0.4, duration=8)
            self.move_forward_by_distance(-0.17)
            self.reset_arm()
            rospy.sleep(1)
            self.close_gripper()
            self.place_success = True

        else:
            if distance_in_x <= gama_x:
                dx = 0
            else:
                dx = self.distance_funtion(distance_in_x*0.7)
            if abs(distance_in_y) <= gama_y:
                dy = 0
            else:
                dy = self.distance_funtion(distance_in_y*4)
            self.my_move_function(dx=dx, dy=dy)
            # if abs(distance_in_y) <= gama_y and distance_in_y < 0:
            #     self.move_function_xy(0, 1)
            # elif abs(distance_in_y) <= gama_y and distance_in_y > 0:
            #     self.move_function_xy(0, -1)
            print("rotate_goal", rotate_goal)
            if rotate_goal > 1.3:
                self.move_function_z(-0.5)
                pass
            elif rotate_goal < -1.3:
                self.move_function_z(0.5)
                pass


def main():
    rospy.init_node('grasp_aruco_node', anonymous=True)
    ap = placeAruco()
    print("=====init=====")
    # ap.reset_arm()
    rospy.sleep(1)
    print("=====reset arm at beginning=====")
    # ap.open_gripper()
    rospy.sleep(1)
    print("=====open gripper at beginning=====")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ap.forward_zero()


        # ap.reset_arm()
if __name__ == '__main__':
    main()
