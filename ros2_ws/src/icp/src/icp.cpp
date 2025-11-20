#include "icp.h"

int main() {
    auto x = std::make_unique<std::vector<double>>();
    auto y = std::make_unique<std::vector<double>>();
    auto theta = std::make_unique<std::vector<double>>();

    read_odom("noisy_odom_data.csv", x, y, theta);

    for (size_t i = 0; i < 5 && i < (*x).size(); ++i)
        std::cout << (*x)[i] << ", " << (*y)[i] << ", " << (*theta)[i] << "\n";
}
