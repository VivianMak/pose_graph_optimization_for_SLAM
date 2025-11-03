#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>

struct SavedLaserScan 
{
    std::vector<float> ranges;  // ranges
    int32_t sec;   // header.stamp.sec
    uint32_t nanosec;   // header.stamp.nanosec
};

struct SavedOdom
{
    int32_t sec;   // header.stamp.sec
    uint32_t nanosec;   // header.stamp.nanosec

    double position_x;  // pose.pose.position.x
    double position_y;  // pose.pose.position.y
    double position_z;  // pose.pose.position.z

    double orientation_x; // pose.pose.orientation.x
    double orientation_y; // pose.pose.orientation.y
    double orientation_z; // pose.pose.orientation.z
    double orientation_w; // pose.pose.orientation.w

    double linear_x;  // twist.twist.linear.x
    double linear_y;  // twist.twist.linear.y
    double linear_z;  // twist.twist.linear.z

    double angular_x;  // twist.twist.angular.x
    double angular_y;  // twist.twist.angular.y
    double angular_z;  // twist.twist.angular.z
};

class DataCollectionNode : public rclcpp::Node
{
public:
    DataCollectionNode() : Node("data_collection_node")
    {
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&DataCollectionNode::lidar_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 
            10, 
            std::bind(&DataCollectionNode::odom_callback, this, std::placeholders::_1));
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

        // Save Odom
        uint64_t odomCount = odoms_.size();
        file.write((char*)&odomCount, sizeof(odomCount));
        for (auto &odom : odoms_) {
            // Save header values
            file.write((char*)&odom.sec, sizeof(int32_t)); file.write((char*)&odom.nanosec, sizeof(uint32_t));

            // Save pose values
            // Estimated pose that is typically relative to a fixed world frame.
            file.write((char*)&odom.position_x, sizeof(double)); file.write((char*)&odom.position_y, sizeof(double)); file.write((char*)&odom.position_z, sizeof(double));
            file.write((char*)&odom.orientation_x, sizeof(double)); file.write((char*)&odom.orientation_y, sizeof(double)); file.write((char*)&odom.orientation_z, sizeof(double)); file.write((char*)&odom.orientation_w, sizeof(double));

            // Save twist values
            // Estimated linear and angular velocity relative to child_frame_id.
            file.write((char*)&odom.linear_x, sizeof(double)); file.write((char*)&odom.linear_y, sizeof(double)); file.write((char*)&odom.linear_z, sizeof(double));
            file.write((char*)&odom.angular_x, sizeof(double)); file.write((char*)&odom.angular_y, sizeof(double)); file.write((char*)&odom.angular_z, sizeof(double));
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu scans to %s", scans_.size(), filename.c_str());
        RCLCPP_INFO(this->get_logger(), "Saved %zu odoms to %s", odoms_.size(), filename.c_str());
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            scans_.push_back({msg->ranges, msg->header.stamp.sec, msg->header.stamp.nanosec});
            RCLCPP_INFO(this->get_logger(), "LaserScan Sec: %i Nanosec: %u", scans_.back().sec, scans_.back().nanosec);
        }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            odoms_.push_back({
                msg->header.stamp.sec, msg->header.stamp.nanosec,
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z
            });
            RCLCPP_INFO(this->get_logger(), "Odom Sec: %i Nanosec: %u", odoms_.back().sec, odoms_.back().nanosec);
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        std::vector<SavedLaserScan> scans_;
        std::vector<SavedOdom> odoms_;
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
    auto node = std::make_shared<DataCollectionNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    node->saveToBinary("robot_data.bin");
  return 0;
}