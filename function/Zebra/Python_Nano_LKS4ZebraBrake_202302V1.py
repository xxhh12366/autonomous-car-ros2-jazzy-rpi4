# from ctypes.wintypes import ULARGE_INTEGER
import sys
import os
# Add Hardware directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import numpy as np
import cv2 as cv
import time
from Hardware import servo_driver as Servo


# -------------------------------------------------------------------------------#
#                                 图像处理部分                                        #
# -------------------------------------------------------------------------------#
def nothing(x):
    pass


cv.namedWindow('adjust')
cv.createTrackbar('binary_value', 'adjust', 90, 255, nothing)
cv.createTrackbar('canny_low_threshold', 'adjust', 68, 255, nothing)
cv.createTrackbar('hof_threshold', 'adjust', 40, 255, nothing)
cv.createTrackbar('hof_min_line_len', 'adjust', 20, 255, nothing)
cv.createTrackbar('hof_max_line_gap', 'adjust', 10, 255, nothing)
cv.moveWindow('adjust', 0, 0)


# 灰度图转换
def grayscale(image):
    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)  # 将图片变灰
    cv.imshow('Gray image', gray)  # 根据图像灰度 进行缩放
    cv.moveWindow('Gray image', 390, 0)  # 改变窗口的位置和尺寸  390，0窗口左上角的坐标
    return gray


# 二值化图像
def img_binary(img):
    binary_value = cv.getTrackbarPos('binary_value', 'adjust')  # 产生调节板名字 binary_value          #
    ret, binary = cv.threshold(img, binary_value, 255,
                               cv.THRESH_BINARY_INV)  # 原图像(必须为灰度图)，阈值，高于（低于）阈值赋予的新的像素，选择性参数：得到 阈值值和处理后的图像
    # cv.imshow('Binary image', binary)
    # cv.moveWindow('Binary image',390,0)
    return binary


# Canny边缘检测
def canny(image):
    low_threshold = cv.getTrackbarPos('canny_low_threshold', 'adjust')  # 产生调节板名字 low_threshold
    cannyedge = cv.Canny(image, low_threshold, low_threshold * 3)  # 处理过程的第一阈值，第二个阈值
    cv.imshow('Canny image', cannyedge)
    cv.moveWindow('Canny image', 1050, 0)
    return cannyedge


# 高斯滤波，去除噪声
def gaussian_blur(image):
    kernel_size = 3  # 高斯滤波器大小size,奇数
    blur = cv.GaussianBlur(image, (kernel_size, kernel_size), 0)
    # kernel_size = cv.getTrackbarPos('kernel', 'adjust')
    # blur = cv.GaussianBlur(image, (3*kernel_size, 4*kernel_size), 0)
    cv.imshow('Blur image', blur)
    cv.moveWindow('Blur image', 720, 0)
    return blur


# 生成感兴趣区域即Mask掩模
def region_of_interest(image):
    imshape = image.shape  # 获取图像大小
    # 设置梯形感兴趣区
    vertices = np.array([[(0, imshape[0]), (imshape[1] * 5 / 34, imshape[0] * 2 / 3),
                          (imshape[1] * 29 / 34, imshape[0] * 2 / 3), (imshape[1], imshape[0])]],
                        dtype=np.int32)
    # 设置矩形感兴趣区
    # vertices = np.array([[(0,imshape[0]), (imshape[1], imshape[0]),
    #                       (imshape[1], int(imshape[0] * 2 / 3)),(0 ,int(imshape[0] * 2 / 3)) ]],
    #                     dtype=np.int32)
    mask = np.zeros_like(image)  # 生成图像大小一致的zeros矩

    # 填充顶点vertices中间区域
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # 填充函数
    cv.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv.bitwise_and(image, mask)
    cv.imshow('ROI image', masked_image)
    cv.moveWindow('ROI image', 390, 300)
    return masked_image


# 原图像与车道线图像按照a:b比例融合
def weighted_img(img, initial_img):
    alpha = 0.8  # 原图像权重
    beta = 1.  # 车道线图像权重
    lambda_ = 0.
    return cv.addWeighted(initial_img, alpha, img, beta, lambda_)


# 角度滤波
def bypass_angle_filter(lines):
    low_thres = 20  # 低阈值
    high_thres = 80  # 高阈值
    filtered_lines = []
    if lines is None:
        return filtered_lines
    for line in lines:
        for x1, y1, x2, y2 in line:
            # 过滤掉角度0或90度的直线
            if x1 == x2 or y1 == y2:
                continue
            # 保留角度在low_thres到high_thres之间的直线,角度按360度的标度来算
            angle = abs(np.arctan((y2 - y1) / (x2 - x1)) * 180 / np.pi)
            if low_thres < angle < high_thres:
                filtered_lines.append([[x1, y1, x2, y2]])
    return filtered_lines  # 得到过滤后的直线端点对集合


# 霍夫线条检测
def hough_lines(image):
    rho = 2  # 霍夫像素单位,线段以像素为单位的距离精度   距离分辨率
    theta = np.pi / 180  # 霍夫角度移动步长,像素以弧度为单位的角度精度(np.pi/180较为合适)    角度分辨率
    threshold = cv.getTrackbarPos('hof_threshold', 'adjust')  # 霍夫平面累加的阈值，超过设定阈值才被检测出线段，值越大，基本上意味着检出的线段越长，检出的线段个数越少
    min_line_len = cv.getTrackbarPos('hof_min_line_len', 'adjust')  # 线段最小长度(像素级),比这个短的都被忽略.
    max_line_gap = cv.getTrackbarPos('hof_max_line_gap', 'adjust')  # 最大允许断裂长度,两条直线之间的最大间隔，小于此值，认为是一条直线
    lines = cv.HoughLinesP(image, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)
    # drawing = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    return lines


# -------------------------------------------------------------------------------#
#                                 舵机转化                                        #
# -------------------------------------------------------------------------------#


def err_generator(err):
    err_range = 80  # 视觉处理的误差范围（-80，80）
    uart_range = 2000  # uart转角的范围（1000 2000）
    k = float((uart_range - 1500) / err_range)  # 换算比例
    b = uart_range - k * err_range
    uart = k * err + b
    return uart


# ------------------------------------------------------------------------------- #
#                                 计算                                             #
# ------------------------------------------------------------------------------- #
error, right_x_err, left_x_err = 0, 0, 0


def draw_lines(image, lines):
    global error
    right_y_set = []
    right_x_set = []
    right_slope_set = []
    right_intercept_set = []

    left_y_set = []
    left_x_set = []
    left_slope_set = []
    left_intercept_set = []

    middle_x = image.shape[1] // 2  # 图像中线x坐标（实际）
    cv.circle(image, (middle_x, image.shape[0] - 50), 10, (0, 0, 255), -1)
    max_y = image.shape[0]  # 最大y坐标

    for line in lines:
        global right_x_err, left_x_err
        right_x_err = left_x_err = 0
        for x1, y1, x2, y2 in line:
            fit = np.polyfit((x1, x2), (y1, y2), 1)  # 拟合成直线
            slope = fit[0]  # 斜率
            intercept = fit[1]  # 截距
            # 将斜率大于0的点存为右边车道线
            if slope > 0:
                right_y_set.append(y1)
                right_y_set.append(y2)
                right_x_set.append(x1)
                right_x_set.append(x2)
                right_slope_set.append(slope)
                right_intercept_set.append(intercept)

            # 将斜率小于0的点存为左边车道线
            elif slope < 0:
                left_y_set.append(y1)
                left_y_set.append(y2)
                left_x_set.append(x1)
                left_x_set.append(x2)
                left_slope_set.append(slope)
                left_intercept_set.append(intercept)
    # 绘制左车道线
    if left_y_set:
        lindex = left_y_set.index(min(left_y_set))  # 最高点索引
        left_x_top = left_x_set[lindex]  # 最高点横坐标
        left_y_top = left_y_set[lindex]  # 最高点纵坐标
        lslope = np.median(left_slope_set)  # 计算斜率平均值
        lintercept = np.median(left_intercept_set)  # 计算截距平均值

        # 根据斜率计算车道线与图片下方交点作为起点
        left_x_bottom = int(left_x_top + (max_y - left_y_top) / lslope)
        left_x_err = (max_y - 50 - lintercept) / lslope
        # 绘制线段
        cv.line(image, (left_x_bottom, max_y), (left_x_top, left_y_top), (0, 255, 0), 10)
        cv.circle(image, (int(left_x_err), int(max_y - 50)), 10, (0, 255, 255), -1)
    else:
        left_x_err = 0

    # 绘制右车道线
    if right_y_set:
        rindex = right_y_set.index(min(right_y_set))  # 最高点
        right_x_top = right_x_set[rindex]
        right_y_top = right_y_set[rindex]
        rslope = np.median(right_slope_set)
        rintercept = np.median(right_intercept_set)

        # 根据斜率计算车道线与图片下方交点作为起点
        right_x_bottom = int(right_x_top + (max_y - right_y_top) / rslope)
        right_x_err = (max_y - 50 - rintercept) / rslope
        # 绘制线段
        cv.line(image, (right_x_top, right_y_top), (right_x_bottom, max_y), (0, 255, 0), 10)
        cv.circle(image, (int(right_x_err), int(max_y - 50)), 10, (0, 255, 255), -1)
    else:
        right_x_err = 0

    if right_y_set and left_y_set:
        miderr_x = (right_x_err + left_x_err) / 2
        cv.circle(image, (int(miderr_x), int(max_y - 50)), 10, (0, 255, 255), -1)  # 理论中点
        # print('小车直行')
        error = middle_x - miderr_x
        if error > 40:
            error = 40
        if error < -40:
            error = -40
    elif left_y_set and not right_y_set:
        # print('小车右转')
        error = -80
        '''
        lslope = np.median(left_slope_set)
        radian=math.atan(lslope)
        #print('radian = ', radian)
        error=math.degrees(radian)
        print('err = ', error)
        '''
    elif not left_y_set and right_y_set:
        # print('小车左转')
        error = 80
    # print('err = ', error)
    return error


# 图像进行畸变矫正
def undistort(img_path, K, D, DIM, scale=0.82, imshow=False):
    # img = cv2.imread(img_path)
    img = img_path
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if dim1[0] != DIM[0]:
        img = cv.resize(img, DIM, interpolation=cv.INTER_AREA)
    Knew = K.copy()
    if scale:  # change fov
        Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
    map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, DIM, cv.CV_16SC2)
    undistorted_img = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    if imshow:
        cv.imshow('undistorted', undistorted_img)
        # cv2.moveWindow('undistorted',390,0)
    return undistorted_img


def process_image(image):
    DIM = (320, 240)
    K = np.array(
        [[179.36453813267272, 0.0, 160.26827442603204], [0.0, 179.37624139841472, 116.4523671147297], [0.0, 0.0, 1.0]])
    D = np.array([[0.007468616264710591], [-0.31203242809114046], [0.6296849644146987], [-0.42864593218106467]])
    img_undistort = undistort(image, K, D, DIM)  # 图像畸变矫正
    # cv.imshow('img_undistort',img_undistort)
    # cv.moveWindow('img_undistort',400,0)
    # 灰度图转换
    gray = grayscale(img_undistort)
    # binary_img=img_binary(gray)#二值化图像
    # 高斯滤波
    blur_gray = gaussian_blur(gray)
    # Canny边缘检测
    edge_image = canny(blur_gray)
    # 生成Mask掩模
    masked_edges = region_of_interest(edge_image)
    # 基于霍夫变换的直线检测
    lines = hough_lines(masked_edges)
    # 角度滤波
    filtered_lines = bypass_angle_filter(lines)

    line_image = np.zeros_like(img_undistort)  # 输出全部为0的与image大小一致的矩阵
    # 绘制车道线线段
    err = draw_lines(line_image, filtered_lines)

    # 图像融合
    lines_edges = weighted_img(img_undistort, line_image)
    return lines_edges, err


def main():
    # cap = cv.VideoCapture("anticlockwise_rpm500.mp4")
    # cap = cv.VideoCapture("/home/relaxingtech/Videos/MP4Resize/anticlockwise_rpm700.mp4")
    cap = cv.VideoCapture(0)  # 参数是0，表示打开笔记本的内置摄像头
    width = 320
    height = 240
    cap.set(cv.CAP_PROP_FRAME_WIDTH, width)  # 设置图像宽度
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)  # 设置图像高度
    cap.set(cv.CAP_PROP_BUFFERSIZE, 0)  # 限制缓冲区
    fps = cap.get(cv.CAP_PROP_FPS)  # 给出每秒帧数
    while cap.isOpened():
        start_time = time.time()
        counter = 0
        _, frame = cap.read()
        [processed, err] = process_image(frame)

        uart = int(err_generator(-err))
        print("Servo position: ", uart)
        Servo.servo_angle_write(uart)

        # processed = process_image(frame)
        # cv.imshow("image", processed)
        # cv.moveWindow('image',720,300)

        # 用来播放视频
        counter += 1  # 计算帧数
        if (time.time() - start_time) != 0:  # 实时显示帧数
            cv.putText(processed, "FPS {0}".format(float('%.1f' % (counter / (time.time() - start_time)))), (0, 50),
                       cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            cv.imshow('image', processed)
            cv.moveWindow('image', 720, 300)
        # print("FPS: ", counter / (time.time() - start_time))

        time.sleep(1 / fps)  # 按原帧率播放
        cv.waitKey(1)


if __name__ == '__main__':
    main()
