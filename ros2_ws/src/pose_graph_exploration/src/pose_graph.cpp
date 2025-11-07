#include <iostream>
#include "pose_graph_structs.h"

int main() {
    SavedLaserScan temp = {{0.1, 0.2, 0.3}, 2, 30000};
    std::cout << temp.ranges[0] << "\n";
}