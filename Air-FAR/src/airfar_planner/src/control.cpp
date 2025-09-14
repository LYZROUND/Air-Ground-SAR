#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mutex>

class ControlNode {
public:
    ControlNode() : rate_(10.0) { // 设置发布频率为10 Hz
        // 订阅目标点话题
        waypoint_sub_ = nh_.subscribe("/way_point", 10, &ControlNode::waypointCallback, this);
        
        // 发布到 MAVROS 的目标位置话题
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

        // 订阅本地位置，转发到视觉位置
        local_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &ControlNode::poseCallback, this);
        vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

        // 使用定时器发布消息
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &ControlNode::timerCallback, this);

        // 初始化目标点，避免发布空消息
        current_waypoint_.pose.position.x = 0.0;
        current_waypoint_.pose.position.y = 0.0;
        current_waypoint_.pose.position.z = 0.0;
        current_waypoint_.pose.orientation.w = 1.0; // 默认水平姿态
        current_waypoint_.header.frame_id = "map";
    }

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 更新目标点信息并发布
        current_waypoint_.header.stamp = ros::Time::now(); // 更新时间戳
        setpoint_pub_.publish(current_waypoint_);
        ROS_INFO_ONCE("Started publishing setpoints to /mavros/setpoint_position/local");
    }

    void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        double target_yaw;
        double target_pitch;
        double dx = msg->point.x - current_x;
        double dy = msg->point.y - current_y;
        double dz = msg->point.z - current_z;
        target_yaw = std::atan2(dy, dx);
        double horizontal_distance = std::sqrt(dx * dx + dy * dy); // 平面上的距离
        target_pitch = std::atan2(dz, horizontal_distance);

        double goal_dis = std::sqrt(
            std::pow(dx, 2) +
            std::pow(dy, 2) +
            std::pow(dz, 2)
        );

        // if (goal_dis > 3)
        // {
        //     current_waypoint_.pose.position.x = current_x + 3 * cos(target_yaw) * cos(target_pitch);
        //     current_waypoint_.pose.position.y = current_y + 3 * sin(target_yaw) * cos(target_pitch);
        //     current_waypoint_.pose.position.z = current_z + 3 * sin(target_pitch);
        // }
        // else
        // {
        //     current_waypoint_.pose.position.x = msg->point.x;
        //     current_waypoint_.pose.position.y = msg->point.y;
        //     current_waypoint_.pose.position.z = msg->point.z;
        // }
        current_waypoint_.pose.position.x = msg->point.x;
        current_waypoint_.pose.position.y = msg->point.y;
        current_waypoint_.pose.position.z = msg->point.z;

        // 将 yaw 角度转换为四元数
        tf::Quaternion quaternion_target_yaw = tf::createQuaternionFromRPY(0.0, 0.0, target_yaw);
        current_waypoint_.header.stamp = ros::Time::now();
        current_waypoint_.pose.orientation.x = quaternion_target_yaw.x();
        current_waypoint_.pose.orientation.y = quaternion_target_yaw.y();
        current_waypoint_.pose.orientation.z = quaternion_target_yaw.z();
        current_waypoint_.pose.orientation.w = quaternion_target_yaw.w();


    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 提取无人机的yaw并存储
        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        // 转换四元数为欧拉角（roll, pitch, yaw）
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // 存储yaw
        std::lock_guard<std::mutex> lock(mutex_);
        current_yaw_ = yaw;
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;
        current_z = msg->pose.position.z;

        // 转发本地位姿到视觉位姿话题
        vision_pose_pub_.publish(*msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber waypoint_sub_, local_pose_sub_;
    ros::Publisher setpoint_pub_, vision_pose_pub_;
    ros::Timer timer_;

    geometry_msgs::PoseStamped current_waypoint_; // 缓存的目标点
    double rate_; // 发布频率
    double current_yaw_ = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;

    std::mutex mutex_; // 用于保护数据一致性的互斥锁
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "control");

    ControlNode control_node;  // 创建单一类的实例

    ros::spin();
    return 0;
}
