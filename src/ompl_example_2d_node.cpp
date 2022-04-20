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

    // get problem definition
    ompl_rope_planning::ProblemParams prob_params = ompl_rope_planning::getProblemParams(nodeHandle);
    {
        std::cout << "==================== PROBLEM DEFINITION ====================" << std::endl;
        std::cout << "timeout: " << prob_params.timeout << std::endl;
        std::cout << "L: " << prob_params.L << std::endl;

        std::cout << "env_filename: " << prob_params.env_filename << std::endl;

        std::cout << "val_check_resolution: " << prob_params.val_check_resolution << std::endl;

        std::cout << "range: " << prob_params.range << std::endl;

        std::cout << "prob_params.start_pos: " << prob_params.start_pos.at("x") << " " << prob_params.start_pos.at("y") << " "
                  << prob_params.start_pos.at("z") << std::endl;
        std::cout << "prob_params.goal_pos: " << prob_params.goal_pos.at("x") << " " << prob_params.goal_pos.at("y") << " "
                  << prob_params.goal_pos.at("z") << std::endl;

        std::cout << "prob_params.bounds.low: " << prob_params.bounds.at("low")[0] << " " << prob_params.bounds.at("low")[1] << " "
                  << prob_params.bounds.at("low")[2] << " " << prob_params.bounds.at("low")[3] << " " << prob_params.bounds.at("low")[4] << " "
                  << prob_params.bounds.at("low")[5] << std::endl;

        std::cout << "prob_params.bounds.high: " << prob_params.bounds.at("high")[0] << " " << prob_params.bounds.at("high")[1] << " "
                  << prob_params.bounds.at("high")[2] << " " << prob_params.bounds.at("high")[3] << " " << prob_params.bounds.at("high")[4] << " "
                  << prob_params.bounds.at("high")[5] << std::endl;

        std::cout << "prob_params.planner_algorithm: " << prob_params.planner_algorithm << std::endl;
        std::cout << "============================================================" << std::endl;
    }

    //=======================================================================================================================

    ompl_rope_planning::planner planner(prob_params);

    printf("Setting start and goal\n");

    // set start state
    float start[6] = {prob_params.start_pos.at("x"), prob_params.start_pos.at("y"), prob_params.start_pos.at("z"), 0.0, prob_params.L * 0.5, 0.0};
    float goal[6] = {prob_params.goal_pos.at("x"), prob_params.goal_pos.at("y"), prob_params.goal_pos.at("z"), 0.0, prob_params.L * 0.5, 0.0};
    planner.setStartGoal(start, goal);

    printf("Planning...\n");
    planner.plan();

    printf("Done!\n");

    // exit ros node
    ros::shutdown();

    return 0;
}
