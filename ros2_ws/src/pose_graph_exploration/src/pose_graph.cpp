#include "pose_graph_header.h"

int main() {
    std::shared_ptr<std::vector<SavedLaserScan>>  scans = std::make_shared<std::vector<SavedLaserScan>>();;
    std::shared_ptr<std::vector<SavedOdom>> odoms = std::make_shared<std::vector<SavedOdom>>();;
    read_data("robot_data.bin", scans, odoms);

    size_t idx = 0;
    
    while (idx < (*scans).size()) {
        SavedOdom curr_odom = (*odoms)[idx];
        std::cout << "------- Pose -------" << "\n";
        std::cout << "position: " << curr_odom.position_x << ", " << curr_odom.position_y << ", " << curr_odom.position_z << "\n";  // z is always 0
        double theta = quaternion2euler(curr_odom.orientation_x, curr_odom.orientation_y, curr_odom.orientation_z, curr_odom.orientation_w)[2]; // roll and pitch are both 0
        std::cout << "theta: " << theta << "\n";
        std::cout << "homogeneous matrix\n" << pose2homogeneous(curr_odom.position_x, curr_odom.position_y, theta) << "\n";
        idx+=100;
    }
}