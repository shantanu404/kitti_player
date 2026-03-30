#pragma once

#include <string>
#include <opencv2/opencv.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace kitti_utils {
    void parse_utc(const std::string& utc_stamp, builtin_interfaces::msg::Time& timestamp)
    {
        size_t last_dot = utc_stamp.find_last_of('.');
        auto date_time = utc_stamp.substr(0, last_dot);
        auto nano_part = utc_stamp.substr(last_dot + 1);

        std::tm t{};
        std::istringstream ss(date_time);
        ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
        auto seconds = mktime(&t);
        auto nsecs = std::stoi(nano_part);

        timestamp.sec = seconds;
        timestamp.nanosec = nsecs;
    }

    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type) {
            case CV_8UC1:
            return "mono8";
            case CV_8UC3:
            return "bgr8";
            case CV_16SC1:
            return "mono16";
            case CV_8UC4:
            return "rgba8";
            default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void read_image(const std::string& img_path, sensor_msgs::msg::Image& img)
    {
        auto frame = cv::imread(img_path);
        if (frame.empty()) {
            throw std::invalid_argument("image file does not exist");
        }

        img.height = frame.rows;
        img.width = frame.cols;
        img.encoding = kitti_utils::mat_type2encoding(frame.type());
        img.is_bigendian = false;
        img.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);

        auto size = frame.step * frame.rows;
        img.data.resize(size);
        std::memcpy(&img.data[0], frame.data, size);
    }
};