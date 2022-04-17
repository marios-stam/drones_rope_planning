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

    //=======================================================================================================================

    // std::string robot = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/robot-scene-triangle.stl";
    std::string env = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/env-scene-ltu-experiment.stl";
    std::string robot = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/custom_triangle_robot.stl";

    float L = 3;

    ompl_rope_planning::planner planner(robot, env, L);

    // printf("Setting start and goal\n");
    // float start[6] = {0, 3, 1, 0.0, L * 0.5, 0.0};
    // float goal[6] = {0, 5, 1, 0.0, L * 0.5, 0.0};
    // planner.setStartGoal(start, goal);

    // printf("Planning...\n");
    // planner.plan();
    /*
    //=======================================================================================================================
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
    //=======================================================================================================================

    // float L = 3.5;
    // custom_mesh::CustomMesh mesh(L);
    // mesh.update_mesh(2, 0);

    // mesh.update_mesh(2.5, 0);
    //=======================================================================================================================

    // Eigen::Vector2f P1(0, 0);
    // Eigen::Vector2f P2(2.295827, 0);
    // float L = 3.0;

    // catenaries::getCatenaryCurve2D_optimized_for_lowest_cat_point(P1, P2, L);
    */

    auto space = new ob::RealVectorStateSpace(6);
    ob::State *abstractState = space->allocState();

    float pos[3] = {0, 0, 0};
    float yaw = 0;
    float drones_distance = 0.5;
    float drones_angle = 0;

    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[0] = pos[0];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[1] = pos[1];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[2] = pos[2];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[3] = yaw;
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[4] = drones_distance;
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[5] = drones_angle;

    // calculate time taken

    bool result = planner.isStateValid(abstractState);
    printf("\nresult: %d\n", result);

    printf("==================================================================================================\n");

    pos[0] = 0;
    pos[1] = 4;
    pos[2] = 0;
    yaw = 0;
    drones_distance = 0.5;
    drones_angle = 0;

    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[0] = pos[0];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[1] = pos[1];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[2] = pos[2];
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[3] = yaw;
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[4] = drones_distance;
    abstractState->as<ob::RealVectorStateSpace::StateType>()->values[5] = drones_angle;

    // calculate time taken

    for (int i = 0; i < 1000; i++)
    {
        result = planner.isStateValid(abstractState);
    }

    printf("Done!\n");
    return 0;
}
