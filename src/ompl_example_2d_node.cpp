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

    std::string robot = "/home/marios/thesis_ws/src/drones_rope_planning/resources/env-scene-ltu-experiment.stl";
    std::string env = "/home/marios/thesis_ws/src/drones_rope_planning/resources/custom_triangle_robot.stl";

    ompl_rope_planning::planner planner(robot, env);

    printf("Setting start and goal\n");
    float start[6] = {0, 3, 1, 0.0, 0.0, 0.0};
    float goal[6] = {0, 5, 1, 0.0, 0.0, 0.0};
    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    planner.plan();

    return 0;
}
