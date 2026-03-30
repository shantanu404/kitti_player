#include <chrono>
#include <fstream>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "kitti_player/time_utils.hpp"
#include "kitti_player/pc_utils.hpp"

using namespace std::chrono_literals;

class KITTLidarNode : public rclcpp::Node {
public:
    KITTLidarNode()
        : Node("kitti_lidar_pub_node")
    {
        std::string path;
        this->declare_parameter("data_path", "");
        this->get_parameter("data_path", path);

        double sensor_freq;
        this->declare_parameter("sensor_freq_hz", 10.0);
        this->get_parameter("sensor_freq_hz", sensor_freq);

        std::string topic;
        this->declare_parameter("topic", "/kitti/velodyne_points");
        this->get_parameter("topic", topic);

        this->declare_parameter("frame_id", "base_link");
        this->get_parameter("frame_id", frame_id_);

        // Read the timestamps and files
        auto timestamp_file = path + "/timestamps.txt";
        auto dir_iter = std::filesystem::directory_iterator(path + "/data");

        std::ifstream ts_file(timestamp_file);

        if (!ts_file.is_open()) {
            throw std::invalid_argument("file doesn't exist");
        }

        std::string line;
        while (std::getline(ts_file, line)) {
            builtin_interfaces::msg::Time timestamp;
            kitti_utils::parse_utc(line, timestamp);

            utc_timestamps_.push_back(line);
            timestamps_.push_back(timestamp);
        }

        for (auto pc : dir_iter) {
            pc_files_.push_back(pc.path());
        }
        std::sort(std::begin(pc_files_), std::end(pc_files_));

        assert(pc_files_.size() == timestamps_.size());

        max_pc_ = pc_files_.size();
        cur_pc_ = 0;

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

        auto period = 1000ms / sensor_freq;
        timer_ = this->create_wall_timer(period, std::bind(&KITTLidarNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "publishing in \"topic\" %s with period %f",
            topic.c_str(), period.count()
        );
    }

private:
    void timer_callback()
    {
        if (cur_pc_ >= max_pc_) {
            return;
        }

        sensor_msgs::msg::PointCloud2 pc_msg;
        kitti_utils::read_pointcloud(pc_files_[cur_pc_], pc_msg);
        pc_msg.header.stamp = timestamps_[cur_pc_];
        pc_msg.header.frame_id = frame_id_;

        RCLCPP_INFO(
            this->get_logger(),
            "publishing \"%s\" with timestamp %s",
            pc_files_[cur_pc_].c_str(), utc_timestamps_[cur_pc_].c_str()
        );

        publisher_->publish(pc_msg);
        cur_pc_++;
    }

    size_t max_pc_;
    size_t cur_pc_;

    std::string frame_id_;

    std::vector<std::string> utc_timestamps_;
    std::vector<builtin_interfaces::msg::Time> timestamps_;
    std::vector<std::string> pc_files_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto lidar_pub_node = std::make_shared<KITTLidarNode>();
    rclcpp::spin(lidar_pub_node);
    rclcpp::shutdown();
    return 0;
}