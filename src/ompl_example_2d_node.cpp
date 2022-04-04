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

    // ompl_rope_planning::planner planner_object;
    // planner_object.plan();

    // create fcl_checker object
    printf("Hello World mlkia\n");

    fcl_checking::checker checker;
    checker.loadEnvironment("/home/marios/thesis_ws/src/drones_rope_planning/resources/env-scene.stl");
    checker.loadRobot("/home/marios/thesis_ws/src/drones_rope_planning/resources/robot-scene-triangle.stl");

    printf("Checking collision\n");
    checker.check_collision();

    float pos[3] = {0, 2, 0};
    float q[4] = {0, 0, 0, 1};
    printf("Setting robot transform\n");

    checker.setRobotTransform(pos, q);
    std::cout << "pos: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;

    checker.check_collision();

    return 0;
}
