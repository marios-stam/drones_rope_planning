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
tf::StampedTransform drone1_tf, drone2_tf;

std::vector<realtime_obstacles::CylinderDefinition> get_cylinders_def_vec(drones_rope_planning::PlanningRequest::Request &req)
{
    std::vector<realtime_obstacles::CylinderDefinition> cylinders_def_vec;

    realtime_obstacles::CylinderDefinition cylinder_def;
    // printf("Start: %f %f %f\n", req.start_pos[0], req.start_pos[1], req.start_pos[2]);
    // printf("Goal: %f %f %f\n", req.goal_pos[0], req.goal_pos[1], req.goal_pos[2]);

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

    return cylinders_def_vec;
}

void set_new_start(bool &reset_start_state_to_initial)
{
    // set new start
    auto old_start = planner->getStartState();

    float new_start[6];

    try
    {
        new_start[0] = (drone1_tf.getOrigin().x() + drone2_tf.getOrigin().x()) / 2;
        new_start[1] = (drone1_tf.getOrigin().y() + drone2_tf.getOrigin().y()) / 2;
        new_start[2] = (drone1_tf.getOrigin().z() + drone2_tf.getOrigin().z()) / 2;
        float dx = drone1_tf.getOrigin().x() - drone2_tf.getOrigin().x();
        float dy = drone1_tf.getOrigin().y() - drone2_tf.getOrigin().y();
        float dz = drone1_tf.getOrigin().z() - drone2_tf.getOrigin().z();
        // calculate yaw
        new_start[3] = atan2(dy, dx);

        new_start[4] = old_start[4]; // TODO: calculate the appropriate one (iverse dynamic transform)
        new_start[5] = old_start[5]; // TODO: calculate the appropriate one (iverse dynamic transform)

        // calculate distance between start and goal
        auto goal = planner->getGoalState();
        float dist = sqrt(pow(new_start[0] - goal[0], 2) + pow(new_start[1] - goal[1], 2) + pow(new_start[2] - goal[2], 2));

        float dist_to_goal_threshold = 1;
        if (dist < dist_to_goal_threshold)
        {
            // printf("Start and goal are too close\n");
            reset_start_state_to_initial = true;
            new_start[0] = planner->prob_params.start_pos["x"];
            new_start[1] = planner->prob_params.start_pos["y"];
            new_start[2] = planner->prob_params.start_pos["z"];
        }
    }
    catch (tf::TransformException &ex)
    {
        printf("%s", ex.what());
        // printf("Using old start\n");
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

bool check_prev_path_validity()
{
    ompl::geometric::PathGeometric *prev_path = planner->getPath();

    if (prev_path == nullptr)
    {
        return false;
    }

    bool prev_path_is_valid = true;
    auto states = prev_path->getStates();
    for (int i = 0; i < states.size(); i += 2) //+=2 for less checking --> faster
    {
        auto s = states[i];
        // check state validity
        if (!planner->isStateValid(s))
        {
            // printf("Previous path is invalid\n");
            prev_path_is_valid = false;
            break;
        };
    }

    if (prev_path_is_valid)
    {
        ROS_DEBUG("Previous path is valid\n");
    }

    return prev_path_is_valid;
}

bool planning_service(drones_rope_planning::PlanningRequest::Request &req, drones_rope_planning::PlanningRequest::Response &res)
{
    printf("===============================================================\n");

    // STATS
    const auto times_num = 6;
    static float sum_times[times_num] = {0, 0, 0, 0, 0};
    static float max_times[times_num] = {0, 0, 0, 0, 0};
    float times[times_num] = {0, 0, 0, 0, 0};

    static int times_service_called = 0;
    static auto planning_times = 0;

    times_service_called++;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<realtime_obstacles::CylinderDefinition> cylinders_def_vec = get_cylinders_def_vec(req);

    // printf("Setting the environment\n");
    planner->checker->as<fcl_checking_realtime::checker>()->updateEnvironmentTransforms(cylinders_def_vec);

    auto dt1 = std::chrono::high_resolution_clock::now() - t1;

    bool reset_start_state_to_initial = false;

    if (planner->prob_params.realtime_settings.setting_new_start)
    {
        set_new_start(reset_start_state_to_initial);
    }

    // check if previous plan is valid after environment changes
    // set at false in case we want to plan anyway and dont check about previous plan validity
    bool prev_path_is_valid = false;
    static auto time_of_replanning = ros::Time::now();

    auto t2 = std::chrono::high_resolution_clock::now();
    if (planner->prob_params.realtime_settings.replan_only_if_not_valid == true)
    {
        // Return false because we want to plan anyway
        prev_path_is_valid = check_prev_path_validity();
    }
    auto dt2 = std::chrono::high_resolution_clock::now() - t2;

    bool should_replan = reset_start_state_to_initial || !prev_path_is_valid;

    // stats
    sum_times[1] += std::chrono::duration_cast<std::chrono::milliseconds>(dt1).count();
    sum_times[2] += std::chrono::duration_cast<std::chrono::milliseconds>(dt2).count();

    max_times[1] = std::max(max_times[1], (float)std::chrono::duration_cast<std::chrono::milliseconds>(dt1).count());
    max_times[2] = std::max(max_times[2], (float)std::chrono::duration_cast<std::chrono::milliseconds>(dt2).count());

    if (!should_replan)
    {
        auto time_from_last_replanning = ros::Time::now() - time_of_replanning;

        // Interval of replanning while path maintains valid
        if (time_from_last_replanning.toSec() > planner->prob_params.realtime_settings.replanning_interval / 1000)
        {
            should_replan = true;
        }
        else
        {
            // printf("No need to replan\n");
            return true;
        }
    }

    // replanning
    auto t3 = std::chrono::high_resolution_clock::now();
    time_of_replanning = ros::Time::now();

    // printf("Replanning\n");
    og::PathGeometric *pth;
    try
    {
        pth = planner->plan();
    }
    catch (ompl::Exception &e)
    {
        printf("%s", e.what());
        return false;
    }
    auto dt3 = std::chrono::high_resolution_clock::now() - t3;

    auto t4 = std::chrono::high_resolution_clock::now();
    // converting rigid body path to drone paths
    nav_msgs::Path drone_path1, drone_path2;
    planner->convert_path_to_drones_paths(pth, drone_path1, drone_path2);
    auto dt4 = std::chrono::high_resolution_clock::now() - t4;

    auto t5 = std::chrono::high_resolution_clock::now();
    drone_path1_pub.publish(drone_path1);
    drone_path2_pub.publish(drone_path2);
    auto dt5 = std::chrono::high_resolution_clock::now() - t5;

    auto dt0 = std::chrono::high_resolution_clock::now() - t0;

    // STATS
    planning_times++;

    float dt_to_sec[times_num] = {
        std::chrono::duration_cast<std::chrono::milliseconds>(dt0).count(), std::chrono::duration_cast<std::chrono::milliseconds>(dt1).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(dt2).count(), std::chrono::duration_cast<std::chrono::milliseconds>(dt3).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(dt4).count(), std::chrono::duration_cast<std::chrono::milliseconds>(dt5).count()};

    sum_times[0] += dt_to_sec[0];
    sum_times[1] += dt_to_sec[1];
    sum_times[2] += dt_to_sec[2];
    sum_times[3] += dt_to_sec[3];
    sum_times[4] += dt_to_sec[4];
    sum_times[5] += dt_to_sec[5];

    max_times[0] = std::max(max_times[0], dt_to_sec[0]);
    max_times[3] = std::max(max_times[3], dt_to_sec[3]);
    max_times[4] = std::max(max_times[4], dt_to_sec[4]);
    max_times[5] = std::max(max_times[5], dt_to_sec[5]);

    printf("Total planning time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[0], sum_times[0] / planning_times, max_times[0]);
    printf("\t-Cylindr update time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[1], sum_times[1] / times_service_called,
           max_times[1]);
    printf("\t-Prev_path_vlid time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[2], sum_times[2] / times_service_called,
           max_times[2]);
    printf("\t-plan->solve()  time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[3], sum_times[3] / planning_times,
           max_times[3]);
    printf("\t-RB to drones   time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[4], sum_times[4] / planning_times,
           max_times[4]);
    printf("\t-Publish pths   time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_sec[5], sum_times[5] / planning_times,
           max_times[5]);
    return true;
}

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    // instantiate listener
    auto listener = new tf::TransformListener();

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

    ros::Rate rate(10);

    while (ros::ok())
    {
        try
        {
            listener->lookupTransform("/world", "/drone1", ros::Time(0), drone1_tf);
            // printf("Drone1 Twist: %f, %f, %f\n", twist1.linear.x, twist1.linear.y, twist1.linear.z);
        }
        catch (tf::TransformException ex)
        {
        }

        try
        {
            listener->lookupTransform("/world", "/drone2", ros::Time(0), drone2_tf);
            // printf("Drone2 Twist: %f, %f, %f\n", twist2.linear.x, twist2.linear.y, twist2.linear.z);
        }
        catch (tf::TransformException ex)
        {
        }

        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
}
