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

// custom services
#include <drones_rope_planning/CylinderObstacleData.h>
#include <drones_rope_planning/PlanningRequest.h>
#include <drones_rope_planning/rigid_body_dynamic_path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// use planner as global variable
ompl_rope_planning::planner *planner;
ros::Publisher dyn_path_pub;
ros::Publisher drone_path1_pub, drone_path2_pub;
tf::TransformListener *listener;

bool planning_service(drones_rope_planning::PlanningRequest::Request &req, drones_rope_planning::PlanningRequest::Response &res)
{
    // printf("Start: %f %f %f\n", req.start_pos[0], req.start_pos[1], req.start_pos[2]);
    // printf("Goal: %f %f %f\n", req.goal_pos[0], req.goal_pos[1], req.goal_pos[2]);

    std::vector<realtime_obstacles::CylinderDefinition> cylinders_def_vec;

    realtime_obstacles::CylinderDefinition cylinder_def;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < req.config.size(); i++)
    {
        auto cyl = req.config[i];
        // printf("Cylinder: radius: %f height: %f x: %f y: %f z: %f  \n", cyl.radius, cyl.height, cyl.pos[0], cyl.pos[1], cyl.pos[2]);
        cylinder_def.radius = cyl.radius;
        cylinder_def.height = cyl.height;

        cylinder_def.pos[0] = cyl.pos[0];
        cylinder_def.pos[1] = cyl.pos[1];
        cylinder_def.pos[2] = cyl.pos[2];

        cylinder_def.quat[0] = cyl.quat[0];
        cylinder_def.quat[1] = cyl.quat[1];
        cylinder_def.quat[2] = cyl.quat[2];
        cylinder_def.quat[3] = cyl.quat[3];

        cylinders_def_vec.push_back(cylinder_def);
    }
    auto dt = std::chrono::high_resolution_clock::now() - t0;

    auto t02 = std::chrono::high_resolution_clock::now();
    // printf("Setting the environment\n");
    planner->checker->as<fcl_checking_realtime::checker>()->updateEnvironmentTransforms(cylinders_def_vec);
    auto dt2 = std::chrono::high_resolution_clock::now() - t02;

    if (planner->prob_params.setting_new_start)
    {
        // set new start
        auto old_start = planner->getStartState();

        float new_start[6];

        tf::StampedTransform transform, transform2;
        listener->waitForTransform("/world", "/drone1Path", ros::Time(0), ros::Duration(0.1));
        try
        {
            listener->lookupTransform("/world", "/drone1", ros::Time(0), transform);
            listener->lookupTransform("/world", "/drone2", ros::Time(0), transform2);

            new_start[0] = (transform.getOrigin().x() + transform2.getOrigin().x()) / 2;
            new_start[1] = (transform.getOrigin().y() + transform2.getOrigin().y()) / 2;
            new_start[2] = (transform.getOrigin().z() + transform2.getOrigin().z()) / 2;
            float dx = transform.getOrigin().x() - transform2.getOrigin().x();
            float dy = transform.getOrigin().y() - transform2.getOrigin().y();
            float dz = transform.getOrigin().z() - transform2.getOrigin().z();
            // calculate yaw
            new_start[3] = atan2(dy, dx);

            new_start[4] = old_start[4]; // TODO: calculate the appropriate one (iverse dynamic transform)
            new_start[5] = old_start[5]; // TODO: calculate the appropriate one (iverse dynamic transform)

            // calculate distance between start and goal
            auto goal = planner->getGoalState();
            float dist = sqrt(pow(new_start[0] - goal[0], 2) + pow(new_start[1] - goal[1], 2) + pow(new_start[2] - goal[2], 2));

            if (dist < 0.5)
            {
                printf("Start and goal are too close\n");
                new_start[0] = planner->prob_params.start_pos["x"];
                new_start[1] = planner->prob_params.start_pos["y"];
                new_start[2] = planner->prob_params.start_pos["z"];
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            // Its the first time and nothin has been published to the tree
            new_start[0] = old_start[0];
            new_start[1] = old_start[1];
            new_start[2] = old_start[2];
            new_start[3] = old_start[3];
            new_start[4] = old_start[4];
            new_start[5] = old_start[5];
        }

        planner->setStart(new_start);
    }

    // check if previous plan is valid after environment changes
    ompl::geometric::PathGeometric *prev_path = planner->getPath();
    if (prev_path != nullptr)
    {
        bool prev_path_is_valid = true;

        auto states = prev_path->getStates();
        for (int i = 0; i < states.size(); i++)
        {
            auto s = states[i];
            // check state validity
            if (!planner->isStateValid(s))
            {

                ROS_ERROR("Previous path is invalid\n");
                prev_path_is_valid = false;
                break;
            };
        }

        if (prev_path_is_valid)
        {
            ROS_DEBUG("Previous path is valid\n");
            return true;
        }
    }

    // planning
    auto t03 = std::chrono::high_resolution_clock::now();
    auto pth = planner->plan();
    auto dt3 = std::chrono::high_resolution_clock::now() - t03;

    nav_msgs::Path drone_path1, drone_path2;

    planner->convert_path_to_drones_paths(pth, drone_path1, drone_path2);

    drone_path1_pub.publish(drone_path1);
    drone_path2_pub.publish(drone_path2);

    bool print_times = false;

    if (print_times)
    {
        std::cout << "Time to create cylinders: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;
        std::cout << "Time to update environment: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt2).count() << " ms" << std::endl;
        std::cout << "Time to plan: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt3).count() << " ms" << std::endl;
    }

    // print current ros time
    ros::Time current_time = ros::Time::now();
    printf("%f \n", current_time.toSec());
    std::cout << "=====================================================" << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    // diffeent mains
    // main_static_planning(argc, argv);
    // main_Cylinders_test();
    // main_fcl_checker_realtime_test();
    // main_realtime_planning(argc, argv);

    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    // instantiate listener
    listener = new tf::TransformListener();

    // initialize planner
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    planner = new ompl_rope_planning::planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};

    printf("Planning with goal : %f %f %f\n", goal[0], goal[1], goal[2]);
    planner->setStartGoal(start, goal);

    // create ros publishers
    dyn_path_pub = nodeHandle.advertise<drones_rope_planning::rigid_body_dynamic_path>("/dynamicRigiBodyPath", 1);

    drone_path1_pub = nodeHandle.advertise<nav_msgs::Path>("/drone1Path", 1);
    drone_path2_pub = nodeHandle.advertise<nav_msgs::Path>("/drone2Path", 1);

    // create ros service
    ros::ServiceServer service = nodeHandle.advertiseService("ompl_realtime_planning", planning_service);

    ROS_INFO("Service ready!");

    ros::spin();
}
