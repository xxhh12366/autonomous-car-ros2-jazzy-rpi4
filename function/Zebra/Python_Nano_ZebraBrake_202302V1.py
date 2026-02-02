import time
import cv2
import numpy as np
from pyvesc.VESC.messages import GetValues, SetDutyCycle, SetRPM
import Python_Nano_Motor_202302V2 as Motor
import Python_Nano_Servo_202302V2 as Servo
import Python_Nano_LKS4ZebraBrake_202302V1 as LKS


# 图像进行畸变矫正
def undistort(img_path, K, D, DIM, scale=0.82, imshow=False):
    # img = cv2.imread(img_path)
    img = img_path
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if dim1[0] != DIM[0]:
        img = cv2.resize(img, DIM, interpolation=cv2.INTER_AREA)
    Knew = K.copy()
    if scale:  # change fov
        Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    if imshow:
        cv2.imshow('undistorted', undistorted_img)
        # cv2.moveWindow('undistorted',390,0)
    return undistorted_img


# 感兴趣区域截取
def ROI(img, left_down_x, left_down_y, left_up_x, left_up_y, right_up_x, right_up_y, right_down_x, right_down_y):
    mask = np.zeros_like(img)  # 输入为矩阵img，输出为形状和img一致的矩阵，其元素全部为0

    # print(left_down_x, left_down_y, left_up_x, left_up_y, right_up_x, right_up_y, right_down_x, right_down_y)
    vertices = np.array(
        [[(left_down_x, left_down_y), (left_up_x, left_up_y), (right_up_x, right_up_y), (right_down_x, right_down_y)]],
        dtype=np.int32)  # 截取图像的四点坐标
    # 创建掩膜
    ignore_mask_color = (255, 255, 255)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    # cv2.imshow('mask',mask)
    # cv2.moveWindow('ROI image',390,300)
    return masked_image


# 透视变换
def birdeye(img, ldx, ldy, lux, luy, rux, ruy, rdx, rdy):
    h, w = img.shape[:2]  # 取img图像的高（行）、宽（列）
    # src：源图像中待测矩形的四点坐标

    src = np.float32([[rdx, rdy],  # br
                      [ldx, ldy],  # bl
                      [lux, luy],  # tl
                      [rux, ruy]])  # tr

    # dst：目标图像中矩形的四点坐标
    dst = np.float32([[w, h],  # br
                      [0, h],  # bl
                      [0, 0],  # tl
                      [w, 0]])  # tr

    M = cv2.getPerspectiveTransform(src, dst)  # 计算得到转换矩阵
    minv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)  # 实现透视变换转换
    # cv2.imshow('birdeye image',warped)
    # cv2.moveWindow('birdeye image', 390, 0)
    return warped, minv


# 灰度处理
def preprocessing(img):
    """
    取原始图像的蓝色通道并平滑过滤    
    """
    # 图像腐蚀与膨胀
    kernel1 = np.ones((3, 3), np.uint8)
    kernel2 = np.ones((5, 5), np.uint8)
    # gray = img[ :, :, 0]
    gray1 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    gray2 = cv2.medianBlur(gray1, 1)  # 中值滤波法是一种非线性平滑技术，它将每一像素点的灰度值设置为该点某邻域窗口内的所有像素点灰度值的中值。5为方框的尺寸，必须是奇数
    gray3 = cv2.morphologyEx(gray2, cv2.MORPH_OPEN, kernel1,
                             iterations=3)  # 开运算(open) ,先腐蚀后膨胀的过程。开运算可以用来消除小黑点，在纤细点处分离物体、平滑较大物体的边界的 同时并不明显改变其面积。迭代4次
    gray4 = cv2.morphologyEx(gray3, cv2.MORPH_CLOSE, kernel2, iterations=2)  # 闭运算(close)，先膨胀后腐蚀的过程。闭运算可以用来排除小黑洞。迭代3次

    # cv2.imshow('gray1', gray1)

    # cv2.moveWindow('gray',720,0)
    return gray4


# canny边缘检测
def img_canny(img):
    cannyedge = cv2.Canny(img, 30, 90)  # 边缘检测参数
    # cv2.imshow('Canny image', cannyedge)
    # cv2.moveWindow('canny image',880,0)
    return cannyedge


def getGD(canny):
    """
    返回梯度模式和方向
    """
    sobelx = cv2.Sobel(canny, cv2.CV_32F, 1, 0,
                       ksize=3)  # 用Sobel算子进行图像梯度计算：图像深度是指存储每个像素值所用的位数CV_32F（32位浮点数），当组合为dx=1,
    # dy=0时求x方向的一阶导数，ksize:（可选参数）Sobel算子的大小，必须是1,3,5或者7,默认为3。
    sobely = cv2.Sobel(canny, cv2.CV_32F, 0, 1, ksize=3)  # 当组合为dx=0,dy=1时求y方向的一阶导数
    theta = np.arctan(np.abs(sobely / (sobelx + 1e-10))) * 180 / np.pi  # 方向
    # cv2.imshow("theta", theta)
    # cv2.moveWindow('theta',1200,400)
    Amplitude = np.sqrt(sobelx ** 2 + sobely ** 2)

    mask = (Amplitude > 30).astype(np.float32)  # 转化数据类型为浮点型

    Amplitude = Amplitude * mask
    # cv2.imshow("Amplitude", Amplitude)
    # cv2.moveWindow('Amplitude',800,400)
    return Amplitude, theta


def sliding_window(img1, img2, patch_size, istep=20, jstep=10):  # istep为斑马线识别步长
    """
    get patches and thier upper left corner coordinates获取补丁及其左上角坐标
    The size of the sliding window is currently fixed.滑动窗口的大小目前是固定的
    patch_size: sliding_window's size'补丁大小：滑动窗格的大小
    istep: Row stride
    """
    Ni, Nj = (int(s) for s in patch_size)
    # print('ni=',Ni,'nj=',Nj)
    height1, width1 = img1.shape
    height2, width2 = img2.shape
    for i in range(0, img1.shape[0] - Ni + 1, istep):
        # for j in range(0, img1.shape[1] - Nj, jstep):
        #     patch = (img1[i:i + Ni, j:j + Nj], img2[i:i + Ni, j:j + Nj])
        patch = (img1[i:i + Ni, 0:width1], img2[i:i + Ni, 0:width2])
        yield (i, 0), patch


def predict(patches, DEBUG):
    """
    predict zebra crossing for every patches 1 is zc 0 is background
    """
    labels = np.zeros(len(patches))
    index = 0
    for Amplitude, theta in patches:
        mask = (Amplitude > 25).astype(np.float32)
        h, b = np.histogram(theta[mask.astype(np.bool)], bins=range(0, 80, 5))
        low, high = b[h.argmax()], b[h.argmax() + 1]
        newmask = ((Amplitude > 25) * (theta <= high) * (theta >= low)).astype(
            np.float32)  # newmask没搞懂？？？？？？？？？？？？？？？？？？？？？？？
        value = ((Amplitude * newmask) > 0).sum()

        if value > 900:  # 判断有无斑马线点的阈值
            labels[index] = 1
        index += 1
        # cv2.imshow("newAmplitude", newmask)
        # cv2.moveWindow('newAmplitude',800,0)
        if DEBUG:
            cv2.waitKey(0)

    return labels


def getlocation(indices, labels, Ni, Nj, A):
    """
    return if there is a zebra cossing如果有斑马线请返回
    if true, Combine all the rectangular boxes as its position如果为真，则将所有矩形框合并为其位置
    assume a picture has only one zebra crossing假设一张图片只有一条斑马线
    """
    zc = indices[labels == 1]

    if len(zc) == 0:
        Motor.get_values_example(SetDutyCycle(0.08))
        A = 0
        print('A1=', A)
        return 0, None, A
    else:
        A = A + 1
        print('A2=', A)
        if A == 1:
            Motor.get_values_example(SetDutyCycle(0))
            time.sleep(3)
            # exit(0)   # 退出程序，禁止下一次斑马线制动

        xmin = int(min(zc[:, 1]))
        ymin = int(min(zc[:, 0]))
        xmax = int(xmin + Nj)
        ymax = int(max(zc[:, 0]) + Ni)
        # print('xmin=',xmin,'ymin=',ymin,'xmax=',xmax,'ymax=',ymax)

        Motor.get_values_example(SetDutyCycle(0.08))
        print('A3=', A)
        # GetValueV2.nice(1000)
        return 1, ((xmin, ymin), (xmax, ymax)), A


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # 参数是0，表示打开笔记本的内置摄像头

    DIM = (320, 240)
    K = np.array(
        [[179.36453813267272, 0.0, 160.26827442603204], [0.0, 179.37624139841472, 116.4523671147297], [0.0, 0.0, 1.0]])
    D = np.array([[0.007468616264710591], [-0.31203242809114046], [0.6296849644146987], [-0.42864593218106467]])

    # 截取位置 global left_down_x, left_down_y, left_up_x, left_up_y, right_up_x, right_up_y, right_down_x, right_down_y,
    # cut_trapezoid_up
    cut_trapezoid_up = 180  # 梯形上边长
    cut_trapezoid_down = 240  # 梯形下边长
    cut_trapezoid_high = 40  # 梯形高
    cut_trapezoid_middle_point_x = 320 / 2  # 梯形底中点X
    cut_trapezoid_middle_point_y = 240 - 20  # 梯形底中点Y
    left_down_x = int(cut_trapezoid_middle_point_x - cut_trapezoid_down / 2)
    left_down_y = right_down_y = int(cut_trapezoid_middle_point_y)
    left_up_x = int(cut_trapezoid_middle_point_x - cut_trapezoid_up / 2)
    left_up_y = right_up_y = int(cut_trapezoid_middle_point_y - cut_trapezoid_high)
    right_up_x = int(cut_trapezoid_middle_point_x + cut_trapezoid_up / 2)
    right_down_x = int(cut_trapezoid_middle_point_x + cut_trapezoid_down / 2)

    Ni, Nj = (50, 320)  # 滑动窗格尺寸Nj为x方向长度，Ni为Y方向长度

    DEBUG = False  # if False, won't draw all step
    A = 0
    while True:

        ret, frame = cap.read()
        img = cv2.resize(frame, (320, 240))
        img_undistort = undistort(img, K, D, DIM)  # 图像畸变矫正
        # cv2.imshow('img_undistort',img_undistort)
        # cv2.moveWindow('img_undistort',400,0)

        hfgray = LKS.grayscale(img_undistort)
        blur_gray = LKS.gaussian_blur(hfgray)
        edge_image = LKS.canny(blur_gray)
        masked_edges = LKS.region_of_interest(edge_image)
        lines = LKS.hough_lines(masked_edges)
        filtered_lines = LKS.bypass_angle_filter(lines)
        line_image = np.zeros_like(img_undistort)
        err = LKS.draw_lines(line_image, filtered_lines)
        uart = int(LKS.err_generator(-err))
        print('err=', uart)
        Servo.servo_angle_write(uart)
        lines_edges = LKS.weighted_img(img_undistort, line_image)
        cv2.imshow('hf_img', lines_edges)
        cv2.moveWindow('hf_img', 720, 300)

        # cv2.imshow('hfgray',hfgray)
        # cv2.moveWindow('img_undistort',800,0)
        ROI_img = ROI(img_undistort, left_down_x, left_down_y, left_up_x, left_up_y, right_up_x, right_up_y,
                      right_down_x, right_down_y)  # 截取感兴趣区域
        # cv2.imshow('ROI_img',ROI_img)
        # cv2.moveWindow('ROI_img',800,0)

        birdeye_img, Minv = birdeye(ROI_img, left_down_x, left_down_y, left_up_x, left_up_y, right_up_x, right_up_y,
                                    right_down_x, right_down_y)  # 进行透视变换
        # cv2.imshow('birdeye_img',birdeye_img)
        # cv2.moveWindow('birdeye_img',1200,0)

        gray = preprocessing(birdeye_img)  # 灰度化图像
        # cv2.imshow('gray', gray)
        # cv2.moveWindow('gray',0,400)

        canny = cv2.Canny(gray, 30, 90, apertureSize=3)

        out_img = np.dstack((canny, canny, canny)) * 255  # 创建输出图像以绘制和可视化结果

        # cv2.imshow("canny",canny)
        # cv2.moveWindow('canny',400,400)
        # cannyedge_img=img_canny(gray)#边缘提取

        Amplitude, theta = getGD(canny)
        indices, patches = zip(*sliding_window(Amplitude, theta, patch_size=(Ni, Nj)))  # use sliding_window get
        # indices and patches使用滑动窗口获取索引和补丁

        labels = predict(patches, DEBUG)  # 预测每一个补丁的斑马线，1是斑马线，0是背景
        # print(labels)
        indices = np.array(indices)  # 斑马线判断间距指标，仅参考项
        # print(indices)
        ret, location, A = getlocation(indices, labels, Ni, Nj, A)  # 判断有无斑马线，如有斑马线获取其坐标并将其框选
        for i, j in indices[labels == 1]:
            cv2.rectangle(out_img, (j, i), (j + Nj, i + Ni), (0, 0, 255), 10)
            # print(j,i)
            # print(j+Nj, i+Ni)
        if ret:
            cv2.rectangle(out_img, location[0], location[1], (0, 255, 255), 10)

        # cv2.imshow('out_img',out_img)
        # cv2.moveWindow('out_img',800,800)
        dewarped_out_img = cv2.warpPerspective(out_img, Minv, (320, 240))
        # cv2.imshow('dewarped_out_img',dewarped_out_img)
        # cv2.moveWindow('dewarped_out_img',0,800)
        blend_im = cv2.addWeighted(src1=dewarped_out_img, alpha=1, src2=img_undistort, beta=1, gamma=0)
        cv2.imshow('blend_im', blend_im)
        cv2.moveWindow('blend_im', 1050, 300)

        # 画梯形
        line_color = (255, 0, 0)
        line_weight = 2
        cv2.line(img, (left_down_x, left_down_y), (left_up_x, left_up_y), line_color, line_weight)
        cv2.line(img, (left_up_x, left_up_y), (right_up_x, right_up_y), line_color, line_weight)
        cv2.line(img, (right_up_x, right_up_y), (right_down_x, right_down_y), line_color, line_weight)
        cv2.line(img, (right_down_x, right_down_y), (left_down_x, left_down_y), line_color, line_weight)
        cv2.imshow("img", img)
        cv2.moveWindow('img', 0, 0)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
