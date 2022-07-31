/*
 * ompl_example_2d_node.cpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/fcl_checker_realtime.hpp"

#include "../include/ompl_example_2d/ompl_example_2d.hpp"

#include "../include/catenaries/catenary.hpp"
#include "../include/custom_mesh.hpp"

#include "../include/moving_obstacles.hpp"

#include <ros/ros.h>
// include transform listener
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

// custom services
#include <drones_rope_planning/CylinderObstacleData.h>
#include <drones_rope_planning/PlanningRequest.h>
#include <drones_rope_planning/rigid_body_dynamic_path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char **argv)
{

    // init ROS node
    ros::init(argc, argv, "path_planning_offline_node");
    // create node handler
    ros::NodeHandle nodeHandle("~");

    nav_msgs::Path drone_path1, drone_path2;
    ros::Publisher drone_path1_pub = nodeHandle.advertise<nav_msgs::Path>("/drone1Path", 1);
    ros::Publisher drone_path2_pub = nodeHandle.advertise<nav_msgs::Path>("/drone2Path", 1);

    ros::Publisher finished_planning_pub = nodeHandle.advertise<std_msgs::String>("/finished_planning", 1);

    // sleep
    ros::Duration(2.0).sleep();

    //=======================================================================================================================

    // get problem parameters
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    ompl_rope_planning::planner planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;
    float start[6] = {
        prob_prms.start["x"],           prob_prms.start["y"], prob_prms.start["z"], prob_prms.start["yaw"], prob_prms.start["drones_distance"],
        prob_prms.start["drones_angle"]};

    float goal[6] = {
        prob_prms.goal["x"],           prob_prms.goal["y"], prob_prms.goal["z"], prob_prms.goal["yaw"], prob_prms.goal["drones_distance"],
        prob_prms.goal["drones_angle"]};

    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    planner.plan();

    // Decomposition of path planning
    planner.convert_path_to_drones_paths(planner.getPath(), drone_path1, drone_path2);
    printf("Publishing paths!\n");
    drone_path1_pub.publish(drone_path1);
    drone_path2_pub.publish(drone_path2);

    std_msgs::String msg;
    msg.data = "finished";
    finished_planning_pub.publish(msg);

    printf("Done!\n");
    // exit ros node
    ros::shutdown();
    return 0;
}