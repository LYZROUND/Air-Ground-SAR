#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class GoalPublisher
{
public:
    GoalPublisher()
    {
        // 初始化节点
        ros::NodeHandle nh;

        // 订阅路径话题
        path_sub_ = nh.subscribe("/ugv_path", 10, &GoalPublisher::pathCallback, this);

        // 发布目标点话题
        goal_pub_ = nh.advertise<geometry_msgs::PointStamped>("way_point", 10);

        // 订阅位置估计话题
        odom_sub_ = nh.subscribe("state_estimation", 10, &GoalPublisher::odomCallback, this);

        // 设置目标点范围
        goal_tolerance_ = 0.5;  // 目标点容忍范围，单位米
    }

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        // 保存路径点
        
        path_ = msg->poses;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 获取无人车当前位置
        current_position_ = msg->pose.pose.position;

        // 检查无人车是否到达目标点
        if (current_index_ < path_.size())
        {
            geometry_msgs::PointStamped goal;
            geometry_msgs::PointStamped next_goal;

            next_goal.point.x = path_[current_index_ + 1].pose.position.x;
            next_goal.point.y = path_[current_index_ + 1].pose.position.y;
            next_goal.point.z = 0;

            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = "map";  // 如果使用的是世界坐标系，可以改为"map"
            goal.point.x = path_[current_index_].pose.position.x;
            goal.point.y = path_[current_index_].pose.position.y;
            goal.point.z = 0;
            goal_pub_.publish(goal);
            
            // 检查是否到达目标点
            if (isAtGoal(goal))
            {
                // 如果到达目标点，发布下一个目标点
                goal_pub_.publish(next_goal);
                ROS_INFO("Arrived at goal #%zu", current_index_);
                current_index_++;  // 更新目标点索引
            }
        }
    }

    bool isAtGoal(const geometry_msgs::PointStamped& goal)
    {
        // 计算当前位置与目标点的欧氏距离
        double distance = std::sqrt(
            std::pow(current_position_.x - goal.point.x, 2) +
            std::pow(current_position_.y - goal.point.y, 2)
        );

        // 如果距离小于目标点范围，认为已经到达目标点
        return distance <= goal_tolerance_;
    }

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    std::vector<geometry_msgs::PoseStamped> path_;
    geometry_msgs::Point current_position_;
    size_t current_index_ = 0;
    double goal_tolerance_;  // 目标点容忍范围
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_planner");
    GoalPublisher goal_publisher;
    ros::spin();
    return 0;
}
