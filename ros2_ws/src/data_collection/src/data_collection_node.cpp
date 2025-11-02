#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <fstream>

struct SavedLaserScan 
{
    std::vector<float> ranges;  // ranges
    int32_t sec;   // header.stamp.sec
    uint32_t nanosec;   // header.stamp.nanosec
};

class DataCollectionNode : public rclcpp::Node
{
public:
    DataCollectionNode() : Node("data_collection_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&DataCollectionNode::lidar_callback, this, std::placeholders::_1));
    }
    void saveToBinary(const std::string &filename)
    {
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filename.c_str());
            return;
        }

        // Save LiDAR
        uint64_t scanCount = scans_.size();
        file.write((char*)&scanCount, sizeof(scanCount));
        for (auto &scan : scans_) {
            uint64_t n = scan.ranges.size();
            file.write((char*)&n, sizeof(n));
            file.write((char*)scan.ranges.data(), n * sizeof(float));
            file.write((char*)&scan.sec, sizeof(int32_t));
            file.write((char*)&scan.nanosec, sizeof(uint32_t));
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu scans to %s", scans_.size(), filename.c_str());
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scans_.push_back({msg->ranges, msg->header.stamp.sec, msg->header.stamp.nanosec});
            RCLCPP_INFO(this->get_logger(), "Sec: %i Nanosec: %u", scans_.back().sec, scans_.back().nanosec);
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        std::vector<SavedLaserScan> scans_;
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
    auto node = std::make_shared<DataCollectionNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    node->saveToBinary("lidar_data2.bin");
  return 0;
}