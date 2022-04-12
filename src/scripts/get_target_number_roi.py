#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


def points2area(points: np.ndarray):
    points = np.squeeze(points)
    x, y = np.split(points, 2, axis=1)
    x, y = np.squeeze(x), np.squeeze(y)
    y_min = np.where(y == np.min(y))
    y_max = np.where(y == np.max(y))
    x_min = np.where(x == np.min(x))
    x_max = np.where(x == np.max(x))
    first = [int(x[x_min][0]), int(y[x_min][0])]
    second = [int(x[y_min][0]), int(y[y_min][0])]
    third = [int(x[x_max][0]), int(y[x_max][0])]
    forth = [int(x[y_max][0]), int(y[y_max][0])]
    # print([first, second, third, forth])
    return [first, second, third, forth]


def extract_roi(img, lsPointsChoose):
    mask = np.zeros(img.shape, np.uint8)
    pts = np.array(lsPointsChoose, np.int32)  # pts是多边形的顶点列表（顶点集）
    col0 = pts[:, 0]
    col1 = pts[:, 1]
    x1 = np.min(col0)
    y1 = np.min(col1)
    x2 = np.max(col0)
    y2 = np.max(col1)
    pts = pts.reshape((-1, 1, 2))
    # 这里 reshape 的第一个参数为-1, 表明这一维的长度是根据后面的维度的计算出来的。
    # OpenCV中需要先将多边形的顶点坐标变成顶点数×1×2维的矩阵，再来绘制

    # --------------画多边形---------------------
    mask = cv2.polylines(mask, [pts], True, (255, 255, 255))
    # -------------填充多边形---------------------
    mask2 = cv2.fillPoly(mask, [pts], (255, 255, 255))
    ROI = cv2.bitwise_and(mask2, img)
    return ROI[y1:y2, x1:x2]


def get_roi_3(img, method_flag='target_number'):
    method = {'Traditional': True, 'Color': False}
    flag = method['Traditional']
    h, w, c = img.shape
    if method_flag == 'target_number':
        flag = method['Color']
        img = img[0:h // 3, 0:w, 0:c]
        img = cv2.resize(img, (3 * w, h))
    elif method_flag == 'box':
        flag = method['Color']
    if flag:
        # 传统方法start
        greyPic = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转化为灰度图
        # cv2.imwrite('./output/01 灰度.png', greyPic)
        greyPic = cv2.medianBlur(greyPic, 3)  # 滤波
        # cv2.imwrite('./output/02 灰度滤波.png', greyPic)
        binPic = cv2.threshold(greyPic, 80, 255, cv2.THRESH_BINARY)[1]  # 二值化
        # cv2.imwrite('./output/03 二值化.png', binPic)
        cannyPic = cv2.Canny(binPic, 300, 500)  # canny
        # cv2.imwrite('./output/04 canny.png', cannyPic)
        # 传统方法end
    ###################-----------sim color method------------##################
    # if not flag:
    #     gs_frame = cv2.GaussianBlur(img, (3, 3), 1)
    #     hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    #     erode_hsv = cv2.erode(
    #         hsv, kernel=np.ones((2, 2), np.uint8), iterations=1)
    #     inRange_hsv = cv2.inRange(
    #         erode_hsv, np.array([0, 180, 70]), np.array([180, 230, 100])
    #     cannyPic=cv2.Canny(inRange_hsv, 600, 800)
    # contours=cv2.findContours(
    #     cannyPic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 找轮廓
    # pic_contours=cv2.drawContours(
    #     img.copy(), contours, -1, (255, 0, 0), 1)
    # area_size_max=0
    # area_size_list=[]
    # for cnt in contours:
    #     current_area_size=cv2.contourArea(cnt)
    #     area_size_list.append(current_area_size)
    #     if current_area_size > area_size_max:
    #         area_size_max=current_area_size
    ###################-----------sim color method------------##################
    if not flag:
        gs_frame = cv2.GaussianBlur(img, (3, 3), 1)
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
        inRange_hsv = cv2.inRange(
            hsv, np.array([156, 110, 90]), np.array([180, 200, 255]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilate_hsv = cv2.dilate(
            inRange_hsv, kernel=kernel, iterations=2)
        cannyPic = cv2.Canny(dilate_hsv, 600, 800)
    contours_pre = cv2.findContours(
        cannyPic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    area_size_max = 0
    area_size_list = []
    contours = []
    for cnt in contours_pre:
        cnt = cv2.approxPolyDP(cnt, 3, True)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        contours.append(box)
        current_area_size = cv2.contourArea(box)
        area_size_list.append(current_area_size)
        if current_area_size > area_size_max:
            area_size_max = current_area_size
    top_index = sorted(
        range(len(area_size_list)), key=lambda k: area_size_list[k])
    top_index.reverse()

    roi = []
    roi_box = []
    for i, num in enumerate(top_index):
        if area_size_list[num] > 2000:
            roi_box_temp = cv2.approxPolyDP(
                contours[num], 3, True)
            if len(roi_box_temp) == 4:
                roi_box.append(roi_box_temp)  # 多边形拟合
                roi_temp = extract_roi(img.copy(), contours[num].squeeze())
                roi_temp_gray = cv2.cvtColor(roi_temp, cv2.COLOR_BGR2GRAY)
                roi.append(roi_temp_gray)

    return roi, roi_box
