#include "pose_graph_header.h"

int main() {
    std::shared_ptr<std::vector<SavedLaserScan>>  scans = std::make_shared<std::vector<SavedLaserScan>>();;
    std::shared_ptr<std::vector<SavedOdom>> odoms = std::make_shared<std::vector<SavedOdom>>();;
    read_data("robot_data.bin", scans, odoms);
}