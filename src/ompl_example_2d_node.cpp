/*
 * ompl_example_2d_node.cpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/fcl_checker.hpp"
#include "../include/ompl_example_2d/ompl_example_2d.hpp"

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

    // std::string robot = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/robot-scene-triangle.stl";
    std::string env = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/env-scene-ltu-experiment.stl";
    std::string robot = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/custom_triangle_robot.stl";

    ompl_rope_planning::planner planner(robot, env);

    printf("Setting start and goal\n");
    float start[6] = {0, 3, 1, 0.0, 0.0, 0.0};
    float goal[6] = {0, 5, 1, 0.0, 0.0, 0.0};
    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    planner.plan();

    // create fcl checker
    // fcl_checking::checker checker(env, robot);
    // bool collision;

    // float pos[3] = {0, 4, 0};
    // float yaw = 0;

    // tf2::Quaternion q;
    // q = q.normalize();
    // int counter = 0, collisions_counter = 0;

    // float min[3] = {-2.2, 2.8, 0.5};
    // float max[3] = {2.2, 5.0, 2.5};
    // while (counter++ < 100)
    // {
    //     // scan keyboard
    //     std::cout << "Enter coordinates:" << std::endl;
    //     std::cin >> pos[0];
    //     std::cin >> pos[1];
    //     std::cin >> pos[2];

    //     q.setRPY(0, 0, yaw);
    //     q = q.normalize();
    //     float quat[4] = {q.x(), q.y(), q.z(), q.w()};

    //     checker.setRobotTransform(pos, quat);
    //     collision = checker.check_collision();
    //     if (collision)
    //     {
    //         collisions_counter++;
    //         std::cout << "Collision: " << collision << std::endl;
    //         std::cout << "x: " << pos[0] << " y: " << pos[1] << " z: " << pos[2] << std::endl;
    //     }
    // }
    // std::cout << "Collisions: " << collisions_counter << std::endl;
    return 0;
}
