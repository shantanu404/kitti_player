#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace kitti_utils {
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

    void read_image(const std::string& img_path, sensor_msgs::msg::CompressedImage& msg) {
        std::ifstream file(img_path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            throw std::invalid_argument("image file does not exist");
        }

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        msg.data.resize(static_cast<size_t>(size));

        if (!file.read(reinterpret_cast<char*>(msg.data.data()), size)) {
            throw std::runtime_error("failed to read compressed image file");
        }

        msg.format = "png";
    }
};