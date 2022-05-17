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
/*
int main(int argc, char **argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    //=======================================================================================================================

    // get problem parameters
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    ompl_rope_planning::planner planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};
    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    planner.plan();

    printf("Done!\n");

    // exit ros node
    ros::shutdown();

    return 0;
}
*/

int main(int argc, char **argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    //=======================================================================================================================

    // get problem parameters
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    ompl_rope_planning::planner planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};
    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    float sum_time = 0.0;
    // get ros para,
    int times = nodeHandle.param<int>("/times", 10);
    int i = 0;

    auto t0 = ros::Time::now();
    auto dt = ros::Time::now() - t0;
    float dt_sec = dt.toSec();
    for (i = 0; i < times && dt_sec < 100; i++)
    {
        sum_time += planner.plan();
        dt_sec = (ros::Time::now() - t0).toSec();

        ROS_ERROR("Time: %f", dt_sec);
    }

    std::cout << "Average time: " << sum_time / i << std::endl;
    printf("Times:%d", i);

    printf("Done!\n");

    // exit ros node
    ros::shutdown();

    return 0;
}

// int main(int argc, char **argv)
// {
//     // init ROS node
//     ros::init(argc, argv, "ompl_example_2d");

//     // create node handler
//     ros::NodeHandle nodeHandle("~");

//     //=======================================================================================================================

//     // get problem parameters
//     problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
//     ompl_rope_planning::planner planner(prob_prms);

//     printf("Setting start and goal\n");
//     float L = prob_prms.L;

//     float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};
//     float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};
//     planner.setStartGoal(start, goal);

//     std::vector<std::string> planner_names;
//     planner_names.push_back("RRT");
//     planner_names.push_back("RRTstar");
//     planner_names.push_back("RRTConnect");
//     planner_names.push_back("InformedRRTstar");
//     planner_names.push_back("PRM");
//     planner_names.push_back("LBKPIECE1");
//     planner_names.push_back("KPIECE");

//     printf("Planning...\n");
//     for (int i = 0; i < planner_names.size(); i++)
//     {
//         planner.plan(planner_names[i]);
//     }

//     printf("Done!\n");

//     // exit ros node
//     ros::shutdown();

//     return 0;
// }
