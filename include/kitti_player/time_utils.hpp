#pragma once

#include <string>
#include <builtin_interfaces/msg/time.hpp>
#include <sstream>

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
};