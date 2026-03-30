#include <fstream>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace fs = std::filesystem;

namespace kitti_utils {
    void parse_utc(const std::string& utc_stamp, builtin_interfaces::msg::Time& timestamp) {
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

    void read_image(const std::string& img_path, sensor_msgs::msg::Image img) {
        
    }
};

class KITTImageNode : public rclcpp::Node {
public:
    KITTImageNode(std::string path)
        : Node("kitti_image_pub_node")
    {
        auto timestamp_file = path + "/timestamps.txt";
        auto dir_iter = fs::directory_iterator(path + "/data");

        std::ifstream ts_file(timestamp_file);

        if (!ts_file.is_open()) {
            throw std::invalid_argument("file doesn't exist");
        }

        std::string line;
        while (std::getline(ts_file, line)) {
            builtin_interfaces::msg::Time timestamp;
            kitti_utils::parse_utc(line, timestamp);

            timestamps_.push_back(timestamp);
        }

        for (auto img : dir_iter) {
            img_files_.push_back(img.path());
        }
    }

private:
    std::vector<builtin_interfaces::msg::Time> timestamps_;
    std::vector<std::string> img_files_;
};

int main() {

    return 0;
}