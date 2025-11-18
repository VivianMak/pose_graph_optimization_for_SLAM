#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

#include "utils.hpp"
#include "test_structs.hpp"


Eigen::Matrix3d makeTransform(const utils::Pose& delta) {
    Eigen::Matrix3d T;

    double c = std::cos(delta.theta);
    double s = std::sin(delta.theta);

    T << c, -s, delta.x,
         s,  c, delta.y,
         0,  0,       1;

    return T;
}

void testStructs() {
    /*
    * Testing initalizations of structs
    */

    std::cout << "---- TESTING POINT ----" << std::endl;
    utils::Point point(1.0, 2.0, 0.0, 5.0);
    std::cout << point.x << ", " << point.y << ", idx=" << point.index << ", r=" << point.r << std::endl;

    std::cout << "---- TESTING POSE ----" << std::endl;
    utils::Pose pose1(1.5, 1.5, 45);
    utils::Pose pose2(0.5, 0.5, 15);
    
    utils::Pose delta_p = pose1 - pose2;

    std::cout << "Pose1 is: " << pose1.x << ", " << pose1.y << ", theta=" << pose1.theta << std::endl;
    std::cout << "Pose2 is: " << pose2.x << ", " << pose2.y << ", theta=" << pose2.theta << std::endl;
    std::cout << "Pose1 - Pose2 = (" 
              << delta_p.x << ", " 
              << delta_p.y << ", " 
              << delta_p.theta << ")\n";

    std::cout << "---- TESTING NODE ----" << std::endl;
    // make 5 nodes with connecting edges from node i to i+1
    // form a loop closure with node 1 and 5

    std::vector<utils::Pose> poses;
    poses.push_back({0.0, 0.0, 0.0});
    poses.push_back({2.0, 1.5, 0.0});
    poses.push_back({2.0, 3.0, -30});
    poses.push_back({1.5, 3.0, 270});
    poses.push_back({0.0, 1.5, 180});

    // store node pointer in a vector
    std::vector<std::unique_ptr<utils::Node>> nodes;
    for (int i = 0; i < 5; i++) {
        nodes.push_back(std::make_unique<utils::Node>(i, poses[i]));
    }


    for (int i = 0; i < 4; i++) {
        /*
        * note: would make more sense to have the edge list in the child node
        * then we can encode edges sequentially as we made the node
        * instead of going back to add an edge
        * Edge: pointer to parent and transform
        */
        
        // get transform matrix from poses delta
        utils::Pose delta_temp = nodes[i]->pose - nodes[i+1]->pose;
        Eigen::Matrix3d T_temp = makeTransform(delta_temp);

        // add an edge to each node
        nodes[i]->addEdge(nodes[i+1].get(), T_temp);
    }

    utils::Pose delta2 = nodes[4]->pose - nodes[0]->pose;
    Eigen::Matrix3d T2 = makeTransform(delta2);

    // Add loop closure: Node 4 -> Node 0
    nodes[4]->addEdge(nodes[0].get(), T2);
    
    std::cout << "Created " << nodes.size() << " nodes." << std::endl;
    std::cout << "Node 4 has " << nodes[4]->edges.size()
              << " outgoing edges (should be 1)." << std::endl;

    // IGNORE:
    // std::vector<std::unique_ptr<Pose>> pose_ptrs;
    // pose_ptrs.push_back(std::make_unique<Pose>(0.0, 0.0, 0.0));
    // pose_ptrs.push_back(std::make_unique<Pose>(1.0, 2.0, 0.5));
}