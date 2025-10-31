#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdint>
#include <vector>
#include <nlohmann/json.hpp>
#include <fstream>


struct SavedLaserScan 
{
    std::vector<float> ranges;  // ranges
    uint32_t nanosec;   // header.stamp.nanosec
};

std::vector<SavedLaserScan> scans;

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

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            // std::stringstream ss;
            // for (float r : msg->ranges) {
            //     ss << r << " ";
            // }
            // RCLCPP_INFO(this->get_logger(), "Laser ranges: %s", ss.str().c_str());
            scans.push_back({msg->ranges, msg->header.stamp.nanosec});
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

void saveToJson(const std::string &filename) {
    nlohmann::json j;

    // Save scans
    for (auto &scan : scans) {
        j["scans"].push_back({
            {"ranges", scan.ranges},
            {"nanosec", scan.nanosec}
        });
    }

    std::ofstream file(filename);
    file << j.dump(2);   // pretty print with 2-space indent
    file.close();

    std::cout << "Data saved to " << filename << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCollectionNode>());
  rclcpp::shutdown();
  saveToJson("lidar_data.json");
  return 0;
}