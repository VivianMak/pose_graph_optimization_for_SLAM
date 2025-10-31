#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
            std::stringstream ss;
            for (float r : msg->ranges) {
                ss << r << " ";
            }
            RCLCPP_INFO(this->get_logger(), "Laser ranges: %s", ss.str().c_str());
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCollectionNode>());
  rclcpp::shutdown();
  return 0;
}