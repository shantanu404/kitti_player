#include <chrono>
#include <fstream>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "kitti_player/utils.hpp"

using namespace std::chrono_literals;

class KITTImageNode : public rclcpp::Node {
public:
    KITTImageNode()
        : Node("kitti_image_pub_node")
    {
        std::string path;
        this->declare_parameter("data_path", "");
        this->get_parameter("data_path", path);

        double sensor_freq;
        this->declare_parameter("sensor_freq_hz", 10.0);
        this->get_parameter("sensor_freq_hz", sensor_freq);

        std::string topic;
        this->declare_parameter("topic", "/kitti/camera/image");
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

        for (auto img : dir_iter) {
            img_files_.push_back(img.path());
        }
        std::sort(std::begin(img_files_), std::end(img_files_));

        assert(img_files_.size() == timestamps_.size());

        max_img_ = img_files_.size();
        cur_img_ = 0;

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);

        auto period = 1000ms / sensor_freq;
        timer_ = this->create_wall_timer(period, std::bind(&KITTImageNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "publishing in \"topic\" %s with period %f",
            topic.c_str(), period.count()
        );
    }

private:
    void timer_callback()
    {
        if (cur_img_ >= max_img_) {
            return;
        }

        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp = timestamps_[cur_img_];
        img_msg.header.frame_id = frame_id_;
        kitti_utils::read_image(img_files_[cur_img_], img_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "publishing \"%s\" with timestamp %s",
            img_files_[cur_img_].c_str(), utc_timestamps_[cur_img_].c_str()
        );

        publisher_->publish(img_msg);
        cur_img_++;
    }

    size_t max_img_;
    size_t cur_img_;

    std::string frame_id_;

    std::vector<std::string> utc_timestamps_;
    std::vector<builtin_interfaces::msg::Time> timestamps_;
    std::vector<std::string> img_files_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto img_pub_node = std::make_shared<KITTImageNode>();
    rclcpp::spin(img_pub_node);
    rclcpp::shutdown();
    return 0;
}