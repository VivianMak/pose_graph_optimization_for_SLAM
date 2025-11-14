#include "pose_graph_header.h"

int main() {
    std::shared_ptr<std::vector<SavedLaserScan>>  scans = std::make_shared<std::vector<SavedLaserScan>>();;
    std::shared_ptr<std::vector<SavedOdom>> odoms = std::make_shared<std::vector<SavedOdom>>();;
    read_data("robot_data.bin", scans, odoms);

    // Setup rng
    // Seed the random number generator using the current time
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    // Define the mean and standard deviation for the normal distribution
    double mean = 0.0;
    double stddev = 0.001;

    // Create a normal distribution object
    std::normal_distribution<double> distribution(mean, stddev);

    std::vector<Pose> odom_poses;
    std::vector<Pose> noisy_poses;

    double x_noise = 0.0;
    double y_noise = 0.0;

    // Odom vector to Poses vector and noisy Poses vector
    for (const SavedOdom &odom : *odoms) {
        double x_random_value = distribution(generator);
        double y_random_value = distribution(generator);

        x_noise += x_random_value;
        y_noise += y_random_value;

        double theta = quaternion2euler(odom.orientation_x, odom.orientation_y, odom.orientation_z, odom.orientation_w)[2];
        odom_poses.push_back(Pose{odom.position_x, odom.position_y, theta});
        noisy_poses.push_back(Pose{odom.position_x + x_noise, odom.position_y + y_noise, theta});
    }

    std::ofstream odom_file("odom_data.csv");
    odom_file << "x,y,theta\n";
    for (const auto& p : odom_poses) {
        odom_file << p.x << "," << p.y << "," << p.theta << "\n";
    }
    odom_file.close();

    std::ofstream noisy_file("noisy_odom_data.csv");
    noisy_file << "x,y,theta\n";
    for (const auto& p : noisy_poses) {
        noisy_file << p.x << "," << p.y << "," << p.theta << "\n";
    }
    noisy_file.close();

    // Print Poses
    // size_t idx = 0;

    // while (idx < odom_poses.size()) {
    //     Pose pose = odom_poses[idx];
    //     Pose noisy_pose = noisy_poses[idx];

    //     std::cout << "Pose: x = " << pose.x << ", y = " << pose.y << ", theta = " << pose.theta << "\n";
    //     std::cout << "Noisy pose: x = " << noisy_pose.x << ", y = " << noisy_pose.y << ", theta = " << noisy_pose.theta << "\n";
    //     idx+=100;
    // }
}