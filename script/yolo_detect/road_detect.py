import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from ultralytics import YOLO

# 初始化 ROS 节点
rospy.init_node('road_segmentation_and_center_pub')

# 图像转换工具
bridge = CvBridge()

# 加载 YOLOv8 模型
model = YOLO('/home/mzh/Air_Ground_ws/src/yolo_detect/road.pt')  # 替换为你的模型路径

# 定义 ROS 发布者
image_pub = rospy.Publisher('/segmented_road_image', Image, queue_size=10)

def process_image(data):
    try:
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    # 使用模型进行分割推理
    results = model(cv_image)
    masks = results[0].masks  # 获取分割掩码

    if masks is not None:
        # 假设路面的类别为0
        road_mask = (masks.data[0].cpu().numpy() * 255).astype(np.uint8)

        # 调整掩码大小以匹配输入图像
        road_mask_resized = cv2.resize(road_mask, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        # 获取所有非零像素的坐标
        road_pixels = np.column_stack(np.where(road_mask_resized > 0))

        if len(road_pixels) > 0:
            # 计算像素点的平均值（质心坐标）
            cy, cx = np.mean(road_pixels, axis=0).astype(int)  # 翻转为 (x, y)
            center_point = (cx, cy)

            # 可视化检查
            rospy.loginfo(f"Calculated center (mean of pixels): {center_point}")

            # 在图像上绘制中心点和掩码
            marked_image = cv_image.copy()
            cv2.drawContours(marked_image, [road_pixels[:, ::-1]], -1, (0, 255, 0), 2)  # 调整为 (x, y)
            cv2.circle(marked_image, center_point, 5, (0, 0, 255), -1)

            # 发布标记后的图像
            image_pub.publish(bridge.cv2_to_imgmsg(marked_image, encoding='bgr8'))
        else:
            rospy.loginfo("No road pixels detected.")
    else:
        rospy.loginfo("No masks detected.")


def main():
    # 订阅 Gazebo 摄像头话题
    rospy.Subscriber('/camera/color/image_raw', Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    main()

