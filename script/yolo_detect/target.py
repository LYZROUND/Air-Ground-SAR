import rospy
import torch
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import math

# 初始化 ROS 节点
rospy.init_node('road_and_target_angle_publisher')

# 图像转换工具
bridge = CvBridge()

# 加载 YOLO 模型
road_model = YOLO('/home/mzh/Air_Ground_ws/script/yolo_detect/roadnew.pt').to('cuda')  # 路面分割模型
target_model = YOLO('/home/mzh/runs/detect/train/weights/best.pt').to('cuda')   # 目标检测模型

# 定义 ROS 发布者
angle_pub = rospy.Publisher('/detected_angles', PointStamped, queue_size=10)
image_pub = rospy.Publisher('/detected/image_raw', Image, queue_size=10)

# 相机参数（根据实际调整）
CAMERA_CENTER_X = 320.5  # 图像水平中心（无人机正前方）
CAMERA_FOCAL_LENGTH = 381.36  # 相机焦距（单位像素）

def calculate_angle(x):
    """计算像素点 x 相对于相机中心的角度（弧度）"""
    return math.atan((x - CAMERA_CENTER_X)/CAMERA_FOCAL_LENGTH)

def process_image(data):
    try:
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    # 路面分割推理
    road_results = road_model(cv_image)
    road_masks = road_results[0].masks

    road_center = None
    if road_masks is not None:
        # 初始化一个全零的掩码，与输入图像大小一致
        combined_road_mask = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8)
        
        # 遍历所有路面掩码
        for mask in road_masks.data:
            road_mask = (mask.cpu().numpy() * 255).astype(np.uint8)  # 单个掩码从 GPU 转到 CPU
            # 调整掩码大小以匹配输入图像
            road_mask_resized = cv2.resize(road_mask, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
            # 累加掩码
            combined_road_mask = cv2.bitwise_or(combined_road_mask, road_mask_resized)

        # 将判断为路面的像素显示为蓝色
        blue_channel = cv_image[:, :, 0]
        blue_channel[combined_road_mask > 0] = 255
        cv_image[:, :, 0] = blue_channel

        # 获取所有非零像素的坐标
        road_pixels = np.column_stack(np.where(combined_road_mask > 0))

        if len(road_pixels) > 0:
            # 计算路面像素点的平均值（质心坐标）
            cy, cx = np.mean(road_pixels, axis=0).astype(int)
            road_center = (cx, cy)

            # 绘制路面中心点
            # cv2.circle(cv_image, road_center, 5, (0, 255, 0), -1)

        # 目标检测推理
        target_results = target_model(cv_image)
        target_boxes = target_results[0].boxes.xywh.cpu().numpy() if target_results[0].boxes is not None else []  # 从 GPU 转到 CPU
        target_center = None
        target_reached = False
        if len(target_boxes) > 0:
            # 处理所有目标框
            for box in target_boxes:
                x_center, y_center, w, h = map(int, box)
                x_min, x_max = x_center - w // 2, x_center + w // 2
                y_min, y_max = y_center - h // 2, y_center + h // 2

                # 绘制边框
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

                # 假设只关注第一个目标
                if target_center is None:
                    target_center = (x_center, y_center)

                    # 绘制目标中心点
                    #cv2.circle(cv_image, target_center, 5, (0, 0, 255), -1)

                    # 统计目标框内的像素数
                    target_pixels = w * h
                    total_pixels = cv_image.shape[0] * cv_image.shape[1]
                    rospy.loginfo("target_pixels=%d",target_pixels)
                    # 判断是否到达目标点
                    if target_pixels > 0.09 * total_pixels:
                        target_reached = 1

    # 计算并发布角度
    if road_center:
        angle_msg = PointStamped()
        angle_msg.header.stamp = rospy.Time.now()
        angle_msg.header.frame_id = "camera"

        # 计算路面中心点的角度
        road_angle = calculate_angle(road_center[0])
        angle_msg.point.y = road_angle

        # 计算路面点和目标点连线中点的角度
        if target_center:
            # midpoint_x = (road_center[0] + target_center[0]) // 2
            midpoint_angle = calculate_angle(target_center[0])
            angle_msg.point.x = midpoint_angle
        else:
            angle_msg.point.x = 0.0  # 如果没有目标，设置中点角度为 0

        # 设置 Z 轴数据
        angle_msg.point.z = 1.0 if target_reached else 0.0

        # 发布消息
        angle_pub.publish(angle_msg)

        rospy.loginfo("target_reached=%d",int(target_reached))

    # 发布标记后的图像
    try:
        marked_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_pub.publish(marked_image_msg)
    except Exception as e:
        rospy.logerr(f"Failed to publish image: {e}")

def main():
    # 订阅相机图像话题
    rospy.Subscriber('/camera/color/image_raw', Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    main()

