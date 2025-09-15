import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import math

# 初始化 YOLO 模型
model = YOLO("/home/mzh/runs/detect/train/weights/best.pt")  # 替换为你的模型路径

# 初始化 CvBridge
bridge = CvBridge()

# 初始化发布者
image_output_topic = "/detected/image_raw"
output_pub = rospy.Publisher(image_output_topic, Image, queue_size=10)

waypoint_topic = "/detected_angle"
waypoint_pub = rospy.Publisher(waypoint_topic, PointStamped, queue_size=10)

# 相机参数（根据你的实际相机设置调整）
CAMERA_FOCAL_LENGTH = 381.36  # 焦距
CAMERA_CENTER_X = 320.5       # 图像中心 X 坐标

# 回调函数
def image_callback(msg):
    try:
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 使用 YOLO 模型推理
        results = model(cv_image)

        # 检测到的目标框信息
        if results and results[0].boxes is not None:
            boxes_xywh = results[0].boxes.xywh.cpu().numpy()  # 获取目标框 (x_center, y_center, width, height)
            confidences = results[0].boxes.conf.cpu().numpy()  # 获取置信度值

            for box, conf in zip(boxes_xywh, confidences):
                if conf > 0.5:  # 仅在置信度 > 0.5 时处理
                    if len(box) == 4:  # 确保框格式正确
                        x_center, y_center, w, h = map(int, box)  # 转换为整数
                        # 画目标中心点
                        cv2.circle(cv_image, (x_center, y_center), radius=3, color=(0, 255, 0), thickness=-1)

                        # 计算相对角度
                        theta = math.atan((x_center - CAMERA_CENTER_X) / CAMERA_FOCAL_LENGTH)

                        # 发布目标点
                        waypoint_msg = PointStamped()
                        waypoint_msg.header.stamp = rospy.Time.now()
                        waypoint_msg.header.frame_id = "kinect_self"  # 相机坐标系
                        waypoint_msg.point.x = theta  # 以角度形式发布（可以根据需求调整为弧度）
                        waypoint_msg.point.y = 0.0  # 保持 Y 轴为 0（或根据需求更改）
                        waypoint_msg.point.z = 0.0  # 保持 Z 轴为 0（或根据需求更改）
                        waypoint_pub.publish(waypoint_msg)

                        rospy.loginfo("Published waypoint: [theta: %f degrees]", math.degrees(theta))
                    else:
                        rospy.logwarn("Box format incorrect: %s", box)
                else:
                    rospy.loginfo("Skipped detection with confidence: %f", conf)

        # 将处理后的图像转换回 ROS 消息并发布
        output_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        output_pub.publish(output_msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge error: %s", e)
    except Exception as e:
        rospy.logerr("Error in image callback: %s", e)


# 初始化 ROS 节点
rospy.init_node('yolo_angle_waypoint_publisher', anonymous=True)

# 订阅输入图像话题
image_topic = "/camera/color/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)

# 保持节点运行
rospy.spin()

