#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
using namespace std;

class GoalPointPlanner {
public:
    GoalPointPlanner() : state_(1) {
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/ugv_path", 10);
        // 订阅所有话题
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &GoalPointPlanner::poseCallback, this);
        angle_sub_ = nh_.subscribe("/detected_angles", 10, &GoalPointPlanner::detectedAnglesCallback, this);
        target_sub_ = nh_.subscribe("/target_point", 10, &GoalPointPlanner::targetCallback, this);
        roadmap_sub_ = nh_.subscribe("/global_map", 10, &GoalPointPlanner::mapCallback, this);
    }

    struct Node {
        int x, y;
        double cost, heuristic;

        Node(int _x, int _y, double _cost, double _heuristic)
            : x(_x), y(_y), cost(_cost), heuristic(_heuristic) {}

        bool operator<(const Node& other) const {
            return cost + heuristic > other.cost + other.heuristic;
        }
    };

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        uav_pose_ = *pose_msg;
        tf::Quaternion q(
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z,
            pose_msg->pose.orientation.w
        );
        tf::Matrix3x3(q).getRPY(current_row_, current_pitch_, current_yaw_);
    }

    void detectedAnglesCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (state_ != 1) return;

        ROS_INFO("Detected angles callback triggered");

        double target_angle = msg->point.x;
        double road_angle = msg->point.y;
        bool is_reached = msg->point.z;

        // 计算目标点的坐标
        goal_x = uav_pose_.pose.position.x + step * cos(-target_angle + current_yaw_);
        goal_y = uav_pose_.pose.position.y + step * sin(-target_angle + current_yaw_);
        goal_z = 2.5;

        // 发布目标点
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = goal_z;

        goal_pub_.publish(goal);

        if (is_reached) {
            state_ = 2;
            ugv_goal_x = uav_pose_.pose.position.x;
            ugv_goal_y = uav_pose_.pose.position.y;
            ROS_INFO("State changed to 2");
        }
    }

    // void globalMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    //     if (state_ != 2) return;

    //     ROS_INFO("Global map callback triggered.");

    //     pcl::PointCloud<pcl::PointXYZI> cloud;
    //     pcl::fromROSMsg(*msg, cloud);

    //     Eigen::Vector4f centroid;
    //     pcl::compute3DCentroid(cloud, centroid);

    //     geometry_msgs::PoseStamped goal;
    //     goal.header.stamp = ros::Time::now();
    //     goal.header.frame_id = "map";
    //     goal.pose.position.x = centroid[0];
    //     goal.pose.position.y = centroid[1];
    //     goal.pose.position.z = 2.5;

    //     goal_pub_.publish(goal);

    //     state_ = 3;
    //     ROS_INFO("State changed to 3");
    // }

    void targetCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (state_ != 2) return;

        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointXYZ nearest_point;
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& point : cloud->points) {
            double distance = std::sqrt(
                std::pow(point.x - 0, 2) +
                std::pow(point.y - 0, 2) +
                std::pow(point.z - 2.5, 2)
            );

            if (distance < min_distance) {
                min_distance = distance;
                nearest_point = point;
            }
        }

        goal_x = nearest_point.x;
        goal_y = nearest_point.y;
        goal_z = 2.5;

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = goal_z;
        goal_pub_.publish(goal);
        if (uav_pose_.pose.position.x < 0.5 && uav_pose_.pose.position.x > -0.5 && uav_pose_.pose.position.y < 0.5 && uav_pose_.pose.position.y > -0.5)
        {
            state_ = 3;
        }
    }

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (state_ != 3) return;

        ROS_INFO("Target callback triggered for A* path planning");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // 假设地图是 1000x1000 的二维栅格
        std::vector<std::vector<int>> grid(1000, std::vector<int>(1000, 1));
        for (const auto& point : cloud->points) {
            int x = static_cast<int>(point.x + 500);
            int y = static_cast<int>(point.y + 500);
            if (x >= 0 && y >= 0 && x < grid.size() && y < grid[0].size()) {
                // 根据 intensity 设置不同标记
                if (point.intensity == 1) {
                    grid[x][y] = 0;  // 路面，可通行
                } else if (point.intensity == 2) {
                    grid[x][y] = 1;  // 障碍物，不可通行
                } else if (point.intensity == 3) {
                    grid[x][y] = 2;  // 边缘点，特殊处理
                }
            }
        }

        std::pair<int, int> start = {500, 500};
        std::pair<int, int> goal = {static_cast<int>(ugv_goal_x + 500), static_cast<int>(ugv_goal_y + 500)};

        auto path = aStar(grid, start, goal);
        if (path.empty()) {
            ROS_WARN("A* path planning failed");
        } else {
            publishPath(path);  // 发布路径
            ROS_INFO("Path successfully published");
        }
    }

    // 获取邻居节点
    std::vector<std::pair<int, int>> getNeighbors(const Node& current) {
        // 允许8个方向的移动
        std::vector<std::pair<int, int>> neighbors = {
            {current.x + 1, current.y}, {current.x - 1, current.y},
            {current.x, current.y + 1}, {current.x, current.y - 1},
            {current.x + 1, current.y + 1}, {current.x - 1, current.y - 1},
            {current.x + 1, current.y - 1}, {current.x - 1, current.y + 1}};
        return neighbors;
    }

    double heuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)) * 0.8;
    }

    double getTraversalCost(int x, int y, int nx, int ny, const std::vector<std::vector<int>>& grid, const std::vector<std::vector<double>>& distance_map) {
        if (grid[nx][ny] == 0 || grid[nx][ny] == 2) {
            double distance = (x != nx && y != ny) ? 1.414 : 1.0;
            double distance_to_obstacle = distance_map[x][y];
            double obs_cost = (distance_to_obstacle > 2.0) ? 0.0 : (2.0 - distance_to_obstacle);

            return distance * 10 + obs_cost * 30;  // 路面，代价最低
            //return distance;  // 路面，代价最低
        } else {
            return std::numeric_limits<double>::infinity();  // 障碍物，不可通行
        }
    }

    std::vector<std::pair<int, int>> reconstructPath(
        const std::unordered_map<int, std::pair<int, int>>& came_from,
        Node current) {
        std::vector<std::pair<int, int>> path;
        auto index = [&](int x, int y) { return x * 1000 + y; };  // 假设地图大小 1000x1000
        while (came_from.find(index(current.x, current.y)) != came_from.end()) {
            path.emplace_back(current.x, current.y);
            current = {came_from.at(index(current.x, current.y)).first,
                    came_from.at(index(current.x, current.y)).second,
                    0, 0};  // 假设 cost 和 heuristic 无需跟踪
        }
        std::reverse(path.begin(), path.end());
        return optimizePath(path);
    }

    std::vector<std::vector<double>> computeObstacleDistanceMap(const std::vector<std::vector<int>>& grid) {
        int rows = grid.size(), cols = grid[0].size();
        std::vector<std::vector<double>> distance_map(rows, std::vector<double>(cols, std::numeric_limits<double>::infinity()));
        std::queue<std::pair<int, int>> q;

        // 将所有障碍物点加入队列并初始化距离为 0
        for (int x = 0; x < rows; ++x) {
            for (int y = 0; y < cols; ++y) {
                if (grid[x][y] == 1) {  // 障碍物
                    distance_map[x][y] = 0.0;
                    q.push({x, y});
                }
            }
        }

        // 广度优先搜索（BFS）计算距离
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
        while (!q.empty()) {
            int cx = q.front().first;
            int cy = q.front().second;
            q.pop();
            for (const auto& dir : directions) {
                int dx = dir.first;
                int dy = dir.second;
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && ny >= 0 && nx < rows && ny < cols) {
                    double new_dist = distance_map[cx][cy] + 1.0;  // 假设邻居距离为 1
                    if (new_dist < distance_map[nx][ny]) {
                        distance_map[nx][ny] = new_dist;
                        q.push({nx, ny});
                    }
                }
            }
        }
        return distance_map;
    }

    std::vector<std::pair<int, int>> aStar(
        const std::vector<std::vector<int>>& grid,
        std::pair<int, int> start,
        std::pair<int, int> goal) {
        // 定义 index 函数，转换 (x, y) 坐标为一维数组索引
        auto index = [](int x, int y) { return x * 1000 + y; };
        auto distance_map = computeObstacleDistanceMap(grid);

        std::priority_queue<Node> open_set;
        std::unordered_map<int, std::pair<int, int>> came_from;
        std::unordered_map<int, double> cost_so_far;
        std::vector<std::vector<double>> g_score(1000, std::vector<double>(1000, std::numeric_limits<double>::infinity()));

        int start_x = start.first, start_y = start.second;
        int goal_x = goal.first, goal_y = goal.second;
        
        g_score[start_x][start_y] = 0.0;
        open_set.push(Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y)));
        
        while (!open_set.empty()) {
            
            Node current = open_set.top();
            open_set.pop();

            if (current.x == goal_x && current.y == goal_y) {
                return reconstructPath(came_from, current);
            }

            for (const auto& neighbor : getNeighbors(current)) {
                int nx = neighbor.first, ny = neighbor.second;
                if (nx >= 0 && ny >= 0 && nx < grid.size() && ny < grid[0].size()) {
                    double tentative_g_score = g_score[current.x][current.y] + getTraversalCost(current.x, current.y, nx, ny, grid, distance_map);

                    if (tentative_g_score < g_score[nx][ny]) {
                        came_from[index(nx, ny)] = {current.x, current.y};
                        g_score[nx][ny] = tentative_g_score;
                        open_set.push(Node(nx, ny, tentative_g_score, heuristic(nx, ny, goal_x, goal_y)));
                    }
                }
            }
        }
        return {};  // 如果无法找到路径，返回空路径
    }

    std::vector<std::pair<int, int>> optimizePath(const std::vector<std::pair<int, int>>& path) {
        std::vector<std::pair<int, int>> optimized_path;

        // 如果路径为空或只有一个点，直接返回
        if (path.empty()) return optimized_path;

        optimized_path.push_back(path[0]);  // 保留起点

        for (size_t i = 1; i < path.size() - 1; ++i) {
            const auto& prev = path[i - 1];
            const auto& current = path[i];
            const auto& next = path[i + 1];

            // 检查三点是否在同一条直线上
            // 判断斜率是否相等，即 (current - prev) 和 (next - current) 的斜率是否相同
            double slope1 = (current.second - prev.second) * 1.0 / (current.first - prev.first + 1e-6);  // 防止除零
            double slope2 = (next.second - current.second) * 1.0 / (next.first - current.first + 1e-6);

            // 如果斜率相等，表示这三点在同一条直线上
            if (slope1 != slope2) {
                optimized_path.push_back(current);
            }
        }

        optimized_path.push_back(path.back());  // 保留终点

        return optimized_path;
    }
    
    void publishPath(const std::vector<std::pair<int, int>>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (const auto& p : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = p.first - 500;
            pose.pose.position.y = p.second - 500;
            pose.pose.position.z = 0;  // 假设平面路径
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber roadmap_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber target_sub_;
    geometry_msgs::PoseStamped uav_pose_;

    int state_;
    double goal_x, goal_y, goal_z;
    double current_row_;
    double current_pitch_;
    double current_yaw_;
    const double step = 20; // 目标点默认距离

    double ugv_goal_x, ugv_goal_y;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher");
    GoalPointPlanner planner;

    // 使用 ros::spin() 处理所有回调
    ros::spin();

    return 0;
}
