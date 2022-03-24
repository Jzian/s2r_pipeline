
import math
import rospy
from geometry_msgs.msg import Twist, Point, Pose
import numpy as np
import cv2
from pyquaternion import Quaternion


class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class grasp_cube_kevin():
    def __init__(self):
        self.base_move_position_pub = rospy.Publisher(
            "cmd_position", Twist, queue_size=10)
        self.base_move_vel_pub = rospy.Publisher(
            "cmd_vel", Twist, queue_size=10)
        self.arm_gripper_pub = rospy.Publisher(
            "arm_gripper", Point, queue_size=10)
        self.arm_position_pub = rospy.Publisher(
            "arm_position", Pose, queue_size=10)
        self.grasp_success = False
        self.place_success = False
        self.target_pose = np.array([])
        self.pose_msg = Pose()
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
        self.close_camera = False
        self.base_vel = 0.3
        self.rpy = []
    def q2e(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        r = r / math.pi * 180
        p = math.asin(2 * (w * y - z * x))
        p = p / math.pi * 180
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        y = y / math.pi * 180
        return r, p, y
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

    def move_arm(self):
        move_arm_msg = Pose()
        move_arm_msg.position.x = 0.2
        move_arm_msg.position.y = -0.02
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def move_arm0(self):
        move_arm_msg = Pose()
        move_arm_msg.position.x = 0.90
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
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        print("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)
        rospy.sleep(0.1)
        self.arm_position_pub.publish(reset_arm_msg)

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

    def move_left_by_distance(self, distance):
        move_base_msg_y = Twist()
        move_base_msg_y.linear.x = 0.0
        move_base_msg_y.linear.y = distance
        move_base_msg_y.linear.z = 0.0
        move_base_msg_y.angular.x = 0.0
        move_base_msg_y.angular.y = 0.0
        move_base_msg_y.angular.z = 0.0
        print("move the base in left")
        self.base_move_position_pub.publish(move_base_msg_y)

    def move_right_by_distance(self, distance):
        move_base_msg_y = Twist()
        move_base_msg_y.linear.x = 0.0
        move_base_msg_y.linear.y = -distance
        move_base_msg_y.linear.z = 0.0
        move_base_msg_y.angular.x = 0.0
        move_base_msg_y.angular.y = 0.0
        move_base_msg_y.angular.z = 0.0
        print("move the base in right")
        self.base_move_position_pub.publish(move_base_msg_y)

    def set_vel2zero(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)

    def point2xyz(self, points):
        points_f = points.astype('float')
        model_image = np.array([(points_f[0, 0, 0], points_f[0, 0, 1]),
                                (points_f[1, 0, 0], points_f[1, 0, 1]),
                                (points_f[2, 0, 0], points_f[2, 0, 1]),
                                (points_f[3, 0, 0], points_f[3, 0, 1])])
        quads_prj = []
        rvec_list = []
        tvec_list = []
        ret, rvec, tvec = cv2.solvePnP(
            self.model_object, model_image, self.camera_matrix, self.dist_coeffs)
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

    def tvec2msg(self, msg, tvec_list, rvec_list):
        msg.position.x = tvec_list[0][0]
        msg.position.y = tvec_list[0][1]
        msg.position.z = tvec_list[0][2]
        r_matrix = cv2.Rodrigues(rvec_list[0])[0]
        q = list(Quaternion(matrix=np.array(r_matrix)))
        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        return msg

    def point2msg(self, points):
        # print('before', points)
        points_after = self.sort_points(points)
        # print('after', points_after)
        tvec_list, rvec_list = self.point2xyz(points_after)
        self.pose_msg = self.tvec2msg(self.pose_msg, tvec_list, rvec_list)
        return self.pose_msg

    def sort_points(self, cnt):
        def cross(x1, y1, x2, y2):
            return x1*y2-x2*y1

        def compare(a=point(0, 0), b=point(0, 0), c=point(0, 0)):
            return cross((b.x-a.x), (b.y-a.y), (c.x-a.x), (c.y-a.y))

        def cmp2(a, b):
            c = point(0, 0)
            if(compare(c, a, b) == 0):
                return a.x < b.x
            else:
                return compare(c, a, b) > 0
        if not len(cnt) == 4:
            return None

        cx = (cnt[0, 0, 0]+cnt[1, 0, 0]+cnt[2, 0, 0]+cnt[3, 0, 0])/4
        cy = (cnt[0, 0, 1]+cnt[1, 0, 1]+cnt[2, 0, 1]+cnt[3, 0, 1])/4

        cnt_norm = cnt.copy()
        for i in range(4):
            cnt_norm[i, 0, 0] = cnt[i, 0, 0] - cx
            cnt_norm[i, 0, 1] = cnt[i, 0, 1] - cy
        cnt_sort = cnt_norm.copy()
        for i in range(4):
            if cnt_norm[i, 0, 0] < 0 and cnt_norm[i, 0, 1] < 0:
                cnt_sort[0, 0, 0], cnt_sort[0, 0,
                                            1] = cnt_norm[i, 0, 0], cnt_norm[i, 0, 1]
            elif cnt_norm[i, 0, 0] > 0 and cnt_norm[i, 0, 1] < 0:
                cnt_sort[1, 0, 0], cnt_sort[1, 0,
                                            1] = cnt_norm[i, 0, 0], cnt_norm[i, 0, 1]
            elif cnt_norm[i, 0, 0] > 0 and cnt_norm[i, 0, 1] > 0:
                cnt_sort[2, 0, 0], cnt_sort[2, 0,
                                            1] = cnt_norm[i, 0, 0], cnt_norm[i, 0, 1]
            elif cnt_norm[i, 0, 0] < 0 and cnt_norm[i, 0, 1] > 0:
                cnt_sort[3, 0, 0], cnt_sort[3, 0,
                                            1] = cnt_norm[i, 0, 0], cnt_norm[i, 0, 1]

        # for t in range(10):
        #     for i in range(3):
        #         p1 = point(cnt_norm[i, 0, 0], cnt_norm[i, 0, 1])
        #         p2 = point(cnt_norm[i+1, 0, 0], cnt_norm[i+1, 0, 1])
        #         if cmp2(p1, p2):
        #             cnt_norm[i, 0, 0], cnt_norm[i+1, 0,
        #                                         0] = cnt_norm[i+1, 0, 0], cnt_norm[i, 0, 0]
        #             cnt_norm[i, 0, 1], cnt_norm[i+1, 0,
        #                                         1] = cnt_norm[i+1, 0, 1], cnt_norm[i, 0, 1]

        for i in range(4):
            cnt_sort[i, 0, 0] = cnt_sort[i, 0, 0] + cx
            cnt_sort[i, 0, 1] = cnt_sort[i, 0, 1] + cy

        return cnt_sort

    def point2line_lenth(self, points):
        points = np.squeeze(points)
        x, y = np.split(points, 2, axis=1)
        x, y = np.squeeze(x), np.squeeze(y)
        x_index = sorted(
            range(len(x)), key=lambda k: x[k])
        y_sort = [y[i] for i in x_index]
        left = abs(y_sort[1]-y_sort[0])
        right = abs(y_sort[3]-y_sort[2])
        return left, right

    def set_arm(self):
        self.reset_arm()
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(0.5)

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
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

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

    def grasp_one_step(self):
        self.close_camera = True
        self.move_arm()
        rospy.sleep(1)
        self.close_gripper()
        rospy.sleep(1)
        self.reset_arm()
        rospy.sleep(1)
        self.forward_zero()
        rospy.sleep(1)
        self.move_forward_by_distance(-0.3)

    def drop_cube(self):
        self.reset_arm()
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(0.5)

    def grasp_cube(self):
        self.move_arm()
        rospy.sleep(1)
        self.close_gripper()
        rospy.sleep(1)
        self.reset_arm()
        rospy.sleep(1)
        self.forward_zero()
        rospy.sleep(1)
        self.move_forward_by_distance(-0.3)
        self.grasp_success = True

    def forward_to_cube(self, center, distance, orientation):
        _,r,_=self.q2e(orientation)
        mid =  0.037
        print(r)
        if center > mid +0.03:
            self.move_function_xy(0,-1)
        elif center < mid -0.03:
            self.move_function_xy(0,1)
        elif distance > 0.35:
            self.move_function_xy(1,0)
        else :
            if r > 5:
                self.move_function_z(-0.2)
            elif r < -5:
                self.move_function_z(0.2)
            else:
                if distance > 0.12:
                    self.move_function_xy(1,0)
                elif center > mid +0.01:
                    self.move_function_xy(0,-1)
                elif center < mid - 0.01:
                    self.move_function_xy(0,1)
                else:
                    self.move_function_xy(0.8,0)
                    self.grasp_cube()

        # if distance > 0.18:
        #     self.move_function_xy(1, 0)
        # elif 0.14 < distance < 0.18:
        #     if center > 0.003:
        #         self.move_function_z(-0.5)
        #     elif center < -0.003:
        #         self.move_function_z(0.5)
        #     else:
        #         self.move_forward_by_distance(0.1)

        # elif distance < 0.14:
        #     self.place_cube()

    def forward_to_box(self, center, distance):
        if distance > 0.45:
            self.move_function_xy(1, 0)
        elif 0.35 < distance < 0.45:
            if center > 0.12:
                self.move_right_by_distance(0.1)
            elif center < -0.12:
                self.move_left_by_distance(0.1)
            else:
                self.move_forward_by_distance(0.1)

        elif distance < 0.15:
            self.grasp_cube()

    def place_cube(self):
        print("===== start placing ====")
        self.reset_arm()
        rospy.sleep(1)
        self.move_arm0()
        rospy.sleep(1)
        self.move_arm()
        rospy.sleep(1)
        self.open_gripper()
        rospy.sleep(1)
        self.reset_arm()
        rospy.sleep(1)
        self.close_gripper()
        self.forward_zero()
        rospy.sleep(1)
        self.place_success = True
        print("===== finish ====")
