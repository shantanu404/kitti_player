#pragma once

#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <sstream>

namespace kitti_utils {
    void read_pointcloud(const std::string& file_path, sensor_msgs::msg::PointCloud2& msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::invalid_argument("point cloud file does not exist");
        }
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            pcl::PointXYZI point;
            iss >> point.x >> point.y >> point.z >> point.intensity;
            cloud.push_back(point);
        }
        pcl::toROSMsg(cloud, msg);
    }
};
