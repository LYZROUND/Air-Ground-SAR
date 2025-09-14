#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

class MapBuilder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber road_cloud_sub_;
    ros::Subscriber obs_cloud_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Publisher map_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher clustered_edge_pub_;
    ros::Publisher target_point_pub_; 
    ros::Publisher connected_road_pub_;

    geometry_msgs::PoseStamped uav_pose_;
    bool uav_pose_received_;

    double grid_size_;
    double radius_;
    int map_size_;

    // 固定大小的二维栅格地图
    //std::vector<std::vector<std::pair<int, int>>> grid_map_;
    std::vector<std::vector<std::tuple<int, int, int>>> grid_map_;

public:
    MapBuilder() : uav_pose_received_(false), grid_size_(1), radius_(12.0), map_size_(1000) {
        // 参数初始化
        nh_.param<double>("grid_size", grid_size_, 1);
        nh_.param<double>("radius", radius_, 12.0);

        // 初始化固定大小的地图（±500范围，对应1000栅格）
        //grid_map_.resize(map_size_, std::vector<std::pair<int, int>>(map_size_, {0, 0}));
        grid_map_.resize(map_size_, std::vector<std::tuple<int, int, int>>(map_size_, {0, 0, 0}));


        // 订阅话题
        road_cloud_sub_ = nh_.subscribe("/road_seg/ugv_cluster_cloud", 1, &MapBuilder::roadCloudCallback, this);
        obs_cloud_sub_ = nh_.subscribe("/road_seg/ugv_obstacle_cloud", 1, &MapBuilder::obsCloudCallback, this);
        uav_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &MapBuilder::uavPoseCallback, this);

        // 发布全局地图
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
        edge_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/edge_map", 1);
        target_point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_point", 1);
        clustered_edge_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_edge_map", 1);
        connected_road_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/connected_road", 1);
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        uav_pose_ = *pose_msg;
        uav_pose_received_ = true;
    }

    void roadCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        processCloud(cloud_msg, true); // 路面点处理
    }

    void obsCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        processCloud(cloud_msg, false); // 障碍点处理
    }

    void processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, bool is_road) {
        if (!uav_pose_received_) {
            ROS_WARN("UAV pose not received yet.");
            return;
        }

        // 转换点云到 PCL 格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // 裁剪点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cropPointCloud(input_cloud, cropped_cloud);

        // 更新栅格地图
        updateGridMap(cropped_cloud, is_road);

        // 检测边缘点
        detectEdgePoints();

        // 发布地图
        publishMap();
    }

    void cropPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud) {
        for (const auto& point : input_cloud->points) {
            double distance = std::sqrt(
                std::pow(point.x - uav_pose_.pose.position.x, 2) +
                std::pow(point.y - uav_pose_.pose.position.y, 2)
            );
            if (distance <= radius_) {
                output_cloud->points.push_back(point);
            }
        }
    }

    void updateGridMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, bool is_road) {
        for (const auto& point : cloud->points) {
            int grid_x = static_cast<int>((point.x + 500) / grid_size_);
            int grid_y = static_cast<int>((point.y + 500) / grid_size_);

            if (grid_x >= 0 && grid_x < map_size_ && grid_y >= 0 && grid_y < map_size_) {
                auto& cell = grid_map_[grid_x][grid_y];
                if (is_road) {
                    std::get<0>(cell)++;  // 路面点计数
                } else {
                    std::get<1>(cell)++; // 障碍点计数
                }
            }
        }
    }

    void detectEdgePoints() {
        std::vector<std::pair<int, int>> directions = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},         {0, 1},
            {1, -1}, {1, 0}, {1, 1}
        };

        for (int i = 0; i < map_size_; ++i) {
            for (int j = 0; j < map_size_; ++j) {
                const auto& cell = grid_map_[i][j];
                std::get<2>(grid_map_[i][j]) = 0;
                // 仅检查路面点
                if (std::get<0>(cell) > 0 && std::get<0>(cell) > std::get<1>(cell)) {
                    for (const auto& dir : directions) {
                        int ni = i + dir.first;
                        int nj = j + dir.second;

                        // 检查相邻栅格是否在地图范围内
                        if (ni >= 0 && ni < map_size_ && nj >= 0 && nj < map_size_) {
                            const auto& neighbor_cell = grid_map_[ni][nj];

                            // 如果邻居是未知点
                            if (std::get<0>(neighbor_cell) == 0 && std::get<1>(neighbor_cell) == 0) {
                                // 标记为边缘点
                                std::get<2>(grid_map_[i][j])= -1; // 特殊标记
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    void clusterEdgePoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_cloud) {
        if (edge_cloud->points.empty()) {
            ROS_WARN("No edge points to cluster.");
            return;
        }

        // Kd树用于聚类
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(edge_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(3); // 聚类的距离阈值
        ec.setMinClusterSize(4);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(edge_cloud);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_points_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // 存储目标点

        int cluster_id = 0;
        for (const auto& indices : cluster_indices) {
            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            for (const auto& index : indices.indices) {
                pcl::PointXYZI point = edge_cloud->points[index];
                point.intensity = cluster_id; // 用 intensity 标记不同的簇
                clustered_cloud->points.push_back(point);

                // 累计质心位置
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;
            }

            // 计算质心
            int cluster_size = indices.indices.size();
            pcl::PointXYZ centroid;
            centroid.x = sum_x / cluster_size;
            centroid.y = sum_y / cluster_size;
            centroid.z = 2.5; // 固定 z 轴
            target_points_cloud->points.push_back(centroid);

            cluster_id++;
        }

        // 发布聚类后的点云
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*clustered_cloud, cluster_msg);
        cluster_msg.header.frame_id = "map";
        cluster_msg.header.stamp = ros::Time::now();
        clustered_edge_pub_.publish(cluster_msg);

        // 发布目标点云
        sensor_msgs::PointCloud2 target_msg;
        pcl::toROSMsg(*target_points_cloud, target_msg);
        target_msg.header.frame_id = "map";
        target_msg.header.stamp = ros::Time::now();
        target_point_pub_.publish(target_msg);

        // ROS_INFO("Published %lu clusters and %lu target points.", cluster_indices.size(), target_points_cloud->points.size());
    }

    void publishMap() {
        pcl::PointCloud<pcl::PointXYZI> output_cloud;
        pcl::PointCloud<pcl::PointXYZI> edge_cloud;
        for (int i = 0; i < map_size_; ++i) {
            for (int j = 0; j < map_size_; ++j) {
                const auto& cell = grid_map_[i][j];

                if (std::get<0>(cell) > 0 || std::get<1>(cell) > 0) {
                    pcl::PointXYZI point;
                    point.x = (i * grid_size_) - 500;
                    point.y = (j * grid_size_) - 500;
                    point.z = 0.0;

                    if (std::get<2>(cell) == -1) {
                        point.intensity = 3;  // 边缘点
                        edge_cloud.points.push_back(point);
                    } else if (std::get<0>(cell) < std::get<1>(cell)) {
                        point.intensity = 2;  // 障碍
                    } else {
                        point.intensity = 1;  // 路面
                    }
                    output_cloud.points.push_back(point);
                }
            }
        }
        // pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(edge_cloud));
        // clusterEdgePoints(edge_cloud_ptr);

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(output_cloud, output_msg);
        output_msg.header.frame_id = "map";
        output_msg.header.stamp = ros::Time::now();
        map_pub_.publish(output_msg);

        // sensor_msgs::PointCloud2 edge_msg;
        // pcl::toROSMsg(edge_cloud, edge_msg);
        // edge_msg.header.frame_id = "map";
        // edge_msg.header.stamp = ros::Time::now();
        // edge_pub_.publish(edge_msg);

        detectConnectedRegions();
    }

    // void detectConnectedRegions() {
    //     // 初始化访问标记和区域 ID 数据结构
    //     std::vector<std::vector<bool>> visited(map_size_, std::vector<bool>(map_size_, false));
    //     std::vector<std::vector<int>> region_map(map_size_, std::vector<int>(map_size_, 0)); // 用于存储区域 ID
    //     int region_id = 1;

    //     // 定义 8 方向移动
    //     std::vector<std::pair<int, int>> directions = {
    //         {-1, -1}, {-1, 0}, {-1, 1},
    //         {0, -1},         {0, 1},
    //         {1, -1}, {1, 0}, {1, 1}
    //     };

    //     // 用于存储每个连通区域的质心坐标
    //     std::vector<std::pair<double, double>> region_centroids;

    //     for (int i = 0; i < map_size_; ++i) {
    //         for (int j = 0; j < map_size_; ++j) {
    //             // 如果当前栅格为路面且未访问过
    //             if (std::get<0>(grid_map_[i][j]) > 0 && std::get<0>(grid_map_[i][j]) > std::get<1>(grid_map_[i][j]) && !visited[i][j]) {
    //                 // 深度优先搜索 (DFS) 标记连通区域
    //                 markRegionDFS(i, j, region_id, visited, region_map, directions);
    //                 region_id++;
                    
    //                 // 计算当前连通区域的质心
    //                 double sum_x = 0.0, sum_y = 0.0;
    //                 int count = 0;
    //                 for (int x = 0; x < map_size_; ++x) {
    //                     for (int y = 0; y < map_size_; ++y) {
    //                         if (region_map[x][y] == region_id-1) {
    //                             sum_x += (x * grid_size_) - 500;
    //                             sum_y += (y * grid_size_) - 500;
    //                             count++;
    //                         }
    //                     }
    //                 }
    //                 if (count > 0) {
    //                     region_centroids.push_back({sum_x / count, sum_y / count});
    //                 }
    //             }
    //         }
    //     }
        
    //     // 计算与无人机的距离并选择最近的连通区域
    //     double min_distance = std::numeric_limits<double>::max();
    //     int nearest_region_id = -1;
    //     for (int i = 0; i < region_centroids.size(); ++i) {
    //         double dx = region_centroids[i].first - uav_pose_.pose.position.x;
    //         double dy = region_centroids[i].second - uav_pose_.pose.position.y;
    //         double distance = std::sqrt(dx * dx + dy * dy);

    //         if (distance < min_distance) {
    //             min_distance = distance;
    //             nearest_region_id = i + 1; // 区域 ID 是从 1 开始的
    //         }
    //     }

    //     // 发布离无人机最近的连通区域
    //     publishConnectedRegions(region_map, nearest_region_id);
    // }

    void detectConnectedRegions() {
        // 初始化访问标记和区域 ID 数据结构
        std::vector<std::vector<bool>> visited(map_size_, std::vector<bool>(map_size_, false));
        std::vector<std::vector<int>> region_map(map_size_, std::vector<int>(map_size_, 0)); // 用于存储区域 ID
        int region_id = 1;

        // 定义 8 方向移动
        std::vector<std::pair<int, int>> directions = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},         {0, 1},
            {1, -1}, {1, 0}, {1, 1}
        };

        // 用于存储每个连通区域的质心坐标
        std::vector<std::pair<double, double>> region_centroids;
        std::vector<std::vector<std::pair<int, int>>> region_points; // 存储每个连通区域的点坐标

        for (int i = 0; i < map_size_; ++i) {
            for (int j = 0; j < map_size_; ++j) {
                // 如果当前栅格为路面且未访问过
                if (std::get<0>(grid_map_[i][j]) > 0 && std::get<0>(grid_map_[i][j]) > std::get<1>(grid_map_[i][j]) && !visited[i][j]) {
                    // 深度优先搜索 (DFS) 标记连通区域
                    markRegionDFS(i, j, region_id, visited, region_map, directions);

                    // 存储当前连通区域的点
                    std::vector<std::pair<int, int>> points;
                    double sum_x = 0.0, sum_y = 0.0;
                    int count = 0;
                    for (int x = 0; x < map_size_; ++x) {
                        for (int y = 0; y < map_size_; ++y) {
                            if (region_map[x][y] == region_id) {
                                points.push_back({x, y});
                                sum_x += (x * grid_size_) - 500;
                                sum_y += (y * grid_size_) - 500;
                                count++;
                            }
                        }
                    }
                    region_points.push_back(points);

                    // 计算质心
                    if (count > 0) {
                        region_centroids.push_back({sum_x / count, sum_y / count});
                    }

                    region_id++;
                }
            }
        }

        // 寻找距离无人机最近的路面点及其连通区域
        double min_distance = std::numeric_limits<double>::max();
        int nearest_region_id = -1;
        std::pair<int, int> nearest_point;

        for (int r = 0; r < region_points.size(); ++r) {
            for (const auto& point : region_points[r]) {
                int x = point.first;
                int y = point.second;
                double world_x = (x * grid_size_) - 500;
                double world_y = (y * grid_size_) - 500;
                double dx = world_x - uav_pose_.pose.position.x;
                double dy = world_y - uav_pose_.pose.position.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_region_id = r + 1; // 区域 ID 是从 1 开始的
                    nearest_point = {x, y};
                }
            }
        }
        // 发布离无人机最近的连通区域
        publishConnectedRegions(region_map, nearest_region_id);
    }
        
    void markRegionDFS(int x, int y, int region_id, std::vector<std::vector<bool>>& visited,
                    std::vector<std::vector<int>>& region_map,
                    const std::vector<std::pair<int, int>>& directions) {
        std::stack<std::pair<int, int>> stack;
        stack.push({x, y});

        while (!stack.empty()) {
            auto top = stack.top();
            int cx = top.first;
            int cy = top.second;
            stack.pop();

            // 边界和访问检查
            if (cx < 0 || cx >= map_size_ || cy < 0 || cy >= map_size_ || visited[cx][cy]) {
                continue;
            }

            auto& cell = grid_map_[cx][cy];
            if ((std::get<0>(cell) > 0 && std::get<0>(cell) > std::get<1>(cell)) || std::get<2>(cell) == -1) {
                // 标记当前栅格为已访问，并赋予区域标识
                visited[cx][cy] = true;
                region_map[cx][cy] = region_id;

                // 将相邻栅格加入堆栈
                for (const auto& dir : directions) {
                    stack.push({cx + dir.first, cy + dir.second});
                }
            }
        }
    }
    // void publishConnectedRegions(const std::vector<std::vector<int>>& region_map, int nearest_region_id) {
    //     pcl::PointCloud<pcl::PointXYZI> connected_cloud;
    //     pcl::PointCloud<pcl::PointXYZI> current_edge_cloud;
    //     for (int i = 0; i < map_size_; ++i) {
    //         for (int j = 0; j < map_size_; ++j) {
    //             int region_id = region_map[i][j];
    //             const auto& cell = grid_map_[i][j];
    //             if (region_id == nearest_region_id && (cell.first > 0 || cell.first > cell.second || cell.first == -1)) { // 仅处理最近的区域
    //             //if (region_id > 0) { // 仅处理最近的区域
    //                 pcl::PointXYZI point;
    //                 point.x = (i * grid_size_) - 500;
    //                 point.y = (j * grid_size_) - 500;
    //                 point.z = 0.0;
    //                 if (cell.first == -1) {
    //                     point.intensity = 2;  // 边缘点
    //                     current_edge_cloud.points.push_back(point);
    //                 } else if (cell.first > cell.second) {
    //                     point.intensity = 1;  // 路面
    //                 } 
    //                 connected_cloud.points.push_back(point);
    //             }
    //         }
    //     }
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(current_edge_cloud));
    //     clusterEdgePoints(edge_cloud_ptr);
    //     // 发布点云消息
    //     sensor_msgs::PointCloud2 connected_msg;
    //     pcl::toROSMsg(connected_cloud, connected_msg);
    //     connected_msg.header.frame_id = "map";
    //     connected_msg.header.stamp = ros::Time::now();
    //     connected_road_pub_.publish(connected_msg);
    // }
    void publishConnectedRegions(const std::vector<std::vector<int>>& region_map, int nearest_region_id) {
        pcl::PointCloud<pcl::PointXYZI> connected_cloud;
        pcl::PointCloud<pcl::PointXYZI> current_edge_cloud;
        bool is_origin_in_region = false; // 判断 (0, 0) 是否在连通区域内

        for (int i = 0; i < map_size_; ++i) {
            for (int j = 0; j < map_size_; ++j) {
                int region_id = region_map[i][j];
                const auto& cell = grid_map_[i][j];
                
                // 检查是否是最近的连通区域
                if (region_id == nearest_region_id && std::get<0>(cell) > std::get<1>(cell)) {
                    pcl::PointXYZI point;
                    point.x = (i * grid_size_) - 500;
                    point.y = (j * grid_size_) - 500;
                    point.z = 0.0;

                    // 判断是否为 (0, 0)
                    if (point.x == 0 && point.y == 0) {
                        is_origin_in_region = true;
                    }

                    if (std::get<2>(cell) == -1) {
                        point.intensity = 2;  // 边缘点
                        current_edge_cloud.points.push_back(point);
                    } else {
                        point.intensity = 1;  // 路面
                    }
                    connected_cloud.points.push_back(point);
                }
            }
        }

        // 如果 (0, 0) 在连通区域内，将其发布到 /target_point
        if (is_origin_in_region) {
            pcl::PointCloud<pcl::PointXYZ> target_cloud;
            pcl::PointXYZ origin_point;
            origin_point.x = 0.0;
            origin_point.y = 0.0;
            origin_point.z = 2.5;
            target_cloud.points.push_back(origin_point);

            sensor_msgs::PointCloud2 target_msg;
            pcl::toROSMsg(target_cloud, target_msg);
            target_msg.header.frame_id = "map";
            target_msg.header.stamp = ros::Time::now();
            target_point_pub_.publish(target_msg);
        }
        else
        {
            // 处理边缘点并聚类
            pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(current_edge_cloud));
            clusterEdgePoints(edge_cloud_ptr);
        }
        // 发布连通区域的点云消息
        sensor_msgs::PointCloud2 connected_msg;
        pcl::toROSMsg(connected_cloud, connected_msg);
        connected_msg.header.frame_id = "map";
        connected_msg.header.stamp = ros::Time::now();
        connected_road_pub_.publish(connected_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_builder");
    MapBuilder map_builder;
    ros::spin();
    return 0;
}
