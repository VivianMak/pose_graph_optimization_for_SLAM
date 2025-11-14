#include <iostream>

#include "utils.hpp"

int main() {
    /*
    * Testing initalizations of structs
    */

    std::cout << "---- TESTING POINT ----" << std::endl;
    utils::Point point(1.0, 2.0, 0.0, 5.0);
    std::cout << point.x << ", " << point.y << ", idx=" << point.index << ", r=" << point.r << std::endl;

    std::cout << "---- TESTING POSE ----" << std::endl;
    utils::Pose pose1(1.5, 1.5, 45);
    utils::Pose pose2(0.5, 0.5, 15);
    
    utils::Pose delta = pose1 - pose2;

    std::cout << "Pose1 is: " << pose1.x << ", " << pose1.y << ", theta=" << pose1.theta << std::endl;
    std::cout << "Pose2 is: " << pose2.x << ", " << pose2.y << ", theta=" << pose2.theta << std::endl;
    std::cout << "Pose1 - Pose2 = (" 
              << delta.x << ", " 
              << delta.y << ", " 
              << delta.theta << ")\n";

    std::cout << "---- TESTING NODE ----" << std::endl;

    return 0;
}