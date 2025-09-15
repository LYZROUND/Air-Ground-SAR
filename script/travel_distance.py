#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
import math

class UAVDistanceCalculator:
    def __init__(self):
        rospy.init_node("uav_distance_calculator", anonymous=True)
        self.prev_pos = None
        self.total_distance = 0.0

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.loginfo("UAV Distance Calculator Started.")
        rospy.spin()

    def pose_callback(self, msg):
        # 当前坐标
        current_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        step_dist = 0.0  # 默认设为0，防止第一次没有值
        if self.prev_pos is not None:
            dx = current_pos[0] - self.prev_pos[0]
            dy = current_pos[1] - self.prev_pos[1]
            dz = current_pos[2] - self.prev_pos[2]
            step_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            self.total_distance += step_dist

        self.prev_pos = current_pos

        rospy.loginfo("Step Distance: %.2f m, Total Distance: %.2f m",
                      step_dist, self.total_distance)


if __name__ == "__main__":
    try:
        UAVDistanceCalculator()
    except rospy.ROSInterruptException:
        pass

