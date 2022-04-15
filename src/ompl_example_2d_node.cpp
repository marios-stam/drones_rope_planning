/*
 * ompl_example_2d_node.cpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/fcl_checker.hpp"
#include "../include/ompl_example_2d/ompl_example_2d.hpp"

#include "../include/catenaries/catenary.hpp"
#include "../include/custom_mesh.hpp"

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char **argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    // // std::string robot = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/robot-scene-triangle.stl";
    // std::string env = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/env-scene-ltu-experiment.stl";
    // std::string robot = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/custom_triangle_robot.stl";

    // ompl_rope_planning::planner planner(robot, env);

    // printf("Setting start and goal\n");
    // float start[6] = {0, 3, 1, 0.0, 0.0, 0.0};
    // float goal[6] = {0, 5, 1, 0.0, 0.0, 0.0};
    // planner.setStartGoal(start, goal);

    // printf("Planning...\n");
    // planner.plan();

    // Eigen::Vector3f start_point(1, 1, 0);
    // Eigen::Vector3f end_point(2, 2, 0);
    // Eigen::Vector3f lowest_point;
    // float L = 3.5;
    // // calculate time taken
    // float time = 0;
    // int times = 1000;
    // for (int i = 0; i < times; i++)
    // {
    //     auto start = std::chrono::high_resolution_clock::now();
    //     lowest_point = catenaries::lowest_point_optimized(start_point, end_point, L);
    //     auto end = std::chrono::high_resolution_clock::now();
    //     auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //     time += diff.count();
    // }

    // std::cout << "Time taken by function: " << time / times << " microseconds." << std::endl;

    // std::cout << "lowest point: " << lowest_point.transpose() << std::endl;

    // start_point = Eigen::Vector3f(4, 8, 0.5);
    // end_point = Eigen::Vector3f(5, 7, 0.7);
    // L = 3.5;
    // lowest_point = catenaries::lowest_point_optimized(start_point, end_point, L);

    // std::cout << "lowest point: " << lowest_point.transpose() << std::endl;
    float L = 3.5;
    custom_mesh::CustomMesh mesh(L);
    mesh.update_mesh(2, 0);

    mesh.update_mesh(2.5, 0);
    return 0;
}
