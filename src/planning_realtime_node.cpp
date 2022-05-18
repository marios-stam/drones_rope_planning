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

nav_msgs::OdometryPtr drone1_tf(new nav_msgs::Odometry);
nav_msgs::OdometryPtr drone2_tf(new nav_msgs::Odometry);

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
    if (!planner->prob_params.realtime_settings.setting_new_start)
        return;

    // set new start
    auto old_start = planner->getStartState();

    float new_start[6];

    static bool reset_goal_to_initial = false;
    try
    {
        // setting new stat of the path the one that is now formed by the drones positions
        float d1_pos[3] = {drone1_tf->pose.pose.position.x, drone1_tf->pose.pose.position.y, drone1_tf->pose.pose.position.z};
        float d2_pos[3] = {drone2_tf->pose.pose.position.x, drone2_tf->pose.pose.position.y, drone2_tf->pose.pose.position.z};

        new_start[0] = (d1_pos[0] + d2_pos[0]) / 2;
        new_start[1] = (d1_pos[1] + d2_pos[1]) / 2;
        new_start[2] = (d1_pos[2] + d2_pos[2]) / 2;
        float dx = d1_pos[0] - d2_pos[0];
        float dy = d1_pos[1] - d2_pos[1];
        float dz = d1_pos[2] - d2_pos[2];

        // calculate yaw
        new_start[3] = atan2(dy, dx);

        new_start[4] = old_start[4]; // TODO: calculate the appropriate one (iverse dynamic transform)
        new_start[5] = old_start[5]; // TODO: calculate the appropriate one (iverse dynamic transform)

        // calculate distance between start and goal
        auto goal = planner->getGoalState();
        float dist = sqrt(pow(new_start[0] - goal[0], 2) + pow(new_start[1] - goal[1], 2) + pow(new_start[2] - goal[2], 2));

        float dist_to_goal_threshold = planner->prob_params.realtime_settings.distance_to_goal_resetting;
        if (dist < dist_to_goal_threshold && dist_to_goal_threshold > 0.0)
        {
            printf("Start and goal are too close\n");
            reset_start_state_to_initial = true;

            // new_start[0] = planner->prob_params.start_pos["x"];
            // new_start[1] = planner->prob_params.start_pos["y"];
            // new_start[2] = planner->prob_params.start_pos["z"];

            float new_goal[6];
            if (reset_goal_to_initial)
            {
                // Formation is now at the initial start and set the goal to the initial goal
                new_goal[0] = planner->prob_params.goal_pos["x"];
                new_goal[1] = planner->prob_params.goal_pos["y"];
                new_goal[2] = planner->prob_params.goal_pos["z"];
            }
            else
            {
                // Formation is now at the initial goal and set the goal to the initial start
                new_goal[0] = planner->prob_params.start_pos["x"];
                new_goal[1] = planner->prob_params.start_pos["y"];
                new_goal[2] = planner->prob_params.start_pos["z"];
            }
            new_goal[3] = 0.0;
            new_goal[4] = planner->prob_params.L * 0.5;
            new_goal[5] = 0.0;

            planner->setGoal(new_goal);
            reset_goal_to_initial = !reset_goal_to_initial;
        }
    }
    catch (tf::TransformException &ex)
    {
        printf("%s", ex.what());
        printf("Using old start\n");
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

bool check_path_validity(ompl::geometric::PathGeometric *prev_path, int &non_valid_state_id)
{
    if (!planner->prob_params.realtime_settings.replan_only_if_not_valid)
    {
        // set at false in case we want to plan anyway and dont check about previous plan validity
        return false;
    }

    if (prev_path == nullptr)
    {
        return false;
    }

    bool prev_path_is_valid = true;
    auto states = prev_path->getStates();
    static unsigned int start_odd = 0;
    start_odd = start_odd == 0 ? 1 : 0;

    for (int i = start_odd; i < states.size(); i += 2) //+=2 for less checking --> faster
    {
        auto s = states[i];
        // check state validity
        if (!planner->isStateValid(s))
        {
            non_valid_state_id = i;
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

int get_state_closer_to_current_drone_position(og::PathGeometric *path)
{
    auto states = path->getStates();

    float d1_pos[3] = {drone1_tf->pose.pose.position.x, drone1_tf->pose.pose.position.y, drone1_tf->pose.pose.position.z};
    float d2_pos[3] = {drone2_tf->pose.pose.position.x, drone2_tf->pose.pose.position.y, drone2_tf->pose.pose.position.z};

    float curr_state[6];
    curr_state[0] = (d1_pos[0] + d2_pos[0]) / 2;
    curr_state[1] = (d1_pos[1] + d2_pos[1]) / 2;
    curr_state[2] = (d1_pos[2] + d2_pos[2]) / 2;
    float dx = d1_pos[0] - d2_pos[0];
    float dy = d1_pos[1] - d2_pos[1];
    float dz = d1_pos[2] - d2_pos[2];

    // calculate yaw
    curr_state[3] = atan2(dy, dx);

    // TODO: calculate the appropriate one (inverse dynamic transform)
    unsigned int min_index;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < path->getStateCount(); i++)
    {
        auto state = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;

        // auto dist = sqrt(pow(state[0] - curr_state[0], 2) + pow(state[1] - curr_state[1], 2) + pow(state[2] - curr_state[2], 2) +
        //  pow(state[3] - curr_state[3], 2));

        // based only on position
        auto dist = sqrt(pow(state[0] - curr_state[0], 2) + pow(state[1] - curr_state[1], 2) + pow(state[2] - curr_state[2], 2));

        if (dist < min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    // ROS_ERROR("Closer state distance: %f", min_dist);

    if (min_dist > 0.8) // TODO:Make that a ROS param
    {
        return -1;
    }

    return min_index;
}

bool planning_service(drones_rope_planning::PlanningRequest::Request &req, drones_rope_planning::PlanningRequest::Response &res)
{
    printf("===============================================================\n");

    // STATS
    const auto times_num = 6;
    static float sum_times[times_num] = {0, 0, 0, 0, 0};
    static float max_times[times_num] = {0, 0, 0, 0, 0};

    static int times_service_called = 0;
    static auto planning_times = 0;

    times_service_called++;

    auto t0 = ros::Time::now();
    auto t1 = ros::Time::now();

    // All variables used in the planning process
    nav_msgs::Path drone_path1, drone_path2;
    int non_valid_state_id;
    static auto time_of_replanning = ros::Time::now();

    // getting and updating cylinders transforms
    planner->checker->as<fcl_checking_realtime::checker>()->update_obstacles_config(req);

    auto dt1 = ros::Time::now() - t1;

    bool reset_start_state_to_initial = false;

    set_new_start(reset_start_state_to_initial);

    auto t2 = ros::Time::now();
    // check if previous plan is valid after environment changes
    bool prev_path_is_valid = check_path_validity(planner->getPath(), non_valid_state_id);
    auto dt2 = ros::Time::now() - t2;

    // check if new start is valid
    bool new_start_valid = planner->isStateValidSimple(planner->getStartState());
    printf("New start valid: %d\n", new_start_valid);

    //
    /*
        If previous plan  not valid and  invalid state id is small->formation is close to the invalid state
    */
    unsigned int invalid_id_threshold = 2; // TODO:make that a ROS param
    bool formation_close_to_invalid_state = !prev_path_is_valid && non_valid_state_id <= invalid_id_threshold;
    if (formation_close_to_invalid_state || !new_start_valid)
    {
        // ROS_ERROR("Non valid state id: %d", non_valid_state_id);
        // ROS_ERROR("Manual resetting of start state");
        // set new valid start manually beacuse fixing of ompl results to collision with the obstacle

        // curent position of drones os already calculated from the set_new_start function
        std::vector<float> start = planner->getStartState();
        Eigen ::Vector3f start_pos(start[0], start[1], start[2]);

        // get problematic obstacle velocity
        int obs_id = planner->checker->as<fcl_checking_realtime::checker>()->get_id_of_obstacle_nearest(start_pos);
        auto obs_pos = planner->checker->as<fcl_checking_realtime::checker>()->get_position(obs_id);
        auto obs_vel = planner->checker->as<fcl_checking_realtime::checker>()->get_velocity(obs_id);

        printf("obs_vel=%f, %f, %f\n", obs_vel[0], obs_vel[1], obs_vel[2]);
        // calculate the perpandcular vector to the velocity of the obstacle
        Eigen::Vector3f vel_perp(-obs_vel[0], obs_vel[1], 0);

        // check if goal is in the half plane of the perpandicular vector
        Eigen::Vector3f goal_pos(req.goal_pos[0], req.goal_pos[1], req.goal_pos[2]);
        Eigen::Vector3f goal_to_obs = goal_pos - obs_pos;
        float goal_to_obs_dot_vel_perp = goal_to_obs.dot(vel_perp);
        bool goal_in_half_plane = goal_to_obs_dot_vel_perp > 0;

        // if goal is in the half plane of the perpandicular vector reveverse it
        if (!goal_in_half_plane)
            vel_perp = -vel_perp;

        Eigen::Vector3f dstart_pos(vel_perp[0], vel_perp[1], 0);

        dstart_pos.normalize();
        dstart_pos *= planner->prob_params.realtime_settings.fix_invalid_start_dist;

        printf("Robot_start: %f %f %f\n", start_pos.x(), start_pos.y(), start_pos.z());
        printf("Obstacle pos: %f %f %f\n", obs_pos.x(), obs_pos.y(), obs_pos.z());
        printf("Dstart: %f %f %f\n", dstart_pos.x(), dstart_pos.y(), dstart_pos.z());

        float new_pos[6] = {start_pos.x(), start_pos.y(), start_pos.z(), start[3], start[4], start[5]};

        unsigned int times_tried_to_fix = 0;
        do
        {
            printf("Trying to reset new start \n");
            new_pos[0] += dstart_pos.x();
            new_pos[1] += dstart_pos.y();
            new_pos[2] += dstart_pos.z();

            planner->setStart(new_pos);
            times_tried_to_fix++;
        } while (!planner->isStateValidSimple(planner->getStartState()) && times_tried_to_fix < 4);
        // debug messages
        // ROS_ERROR("Manual start state set to: %f, %f, %f", new_pos[0], new_pos[1], new_pos[2]);
    }

    // printf("Previous path valid: %d\n", prev_path_is_valid);

    if (!prev_path_is_valid)
        ROS_ERROR("Previous path not valid");

    bool should_replan = reset_start_state_to_initial || !prev_path_is_valid;

    if (!should_replan)
    {
        auto time_from_last_replanning = (ros::Time::now() - time_of_replanning).toSec();
        float replanning_interval = planner->prob_params.realtime_settings.replanning_interval / 1000;

        // Interval of replanning while path maintains valid
        if (time_from_last_replanning > replanning_interval)
        {
            should_replan = true;
        }
        else
        {
            // No need to replan but will try to simplify path
            printf("No need to replan but will try to simplify path..\n");

            ompl::geometric::PathGeometric *path = planner->getPath();

            // deep copy path to old path
            ompl::geometric::PathGeometric *old_path = new ompl::geometric::PathGeometric(*path);

            bool path_valid_after_simplify = planner->simplifyPath(planner->getPath());

            printf("Path waypoints:%d\n", planner->getPath()->getStateCount());

            printf("Path valid after simplify: %d\n", planner->getPath()->check());

            if (path_valid_after_simplify)
            {
                // planner->interpolate_path(planner->getPath()); //causing invalid path

                // chop path up to the points the drones are at the moment
                // printf("Path valididity before get_state_closer_to_current_drone_position: %d\n", planner->getPath()->check());
                int curr_index = get_state_closer_to_current_drone_position(planner->getPath());
                if (curr_index == -1)
                    return true;

                curr_index = std::min(curr_index, 3); // sometimes it was chopping at 9(TODO: check why)
                auto state_to_keep_after = planner->getPath()->getState(curr_index);
                // printf("Keeping all states after: %d\n", curr_index);

                // printf("Path valididity before keeping after: %d\n", planner->getPath()->check());
                planner->getPath()->keepAfter(state_to_keep_after);
                // printf("Path valididity before interpolating: %d\n", planner->getPath()->check());

                // planner->interpolate_path(planner->getPath());

                // printf("Path valididity before publish: %d\n", planner->getPath()->check());
                planner->convert_path_to_drones_paths(planner->getPath(), drone_path1, drone_path2);
                drone_path1_pub.publish(drone_path1);
                drone_path2_pub.publish(drone_path2);

                return true;
            }
            else
            {
                planner->setPath(old_path);
                printf("Path is set back to the old path before simplifying!\n");
                bool result = planner->getPath()->check();
                printf("Path is valid: %d\n", result);
                return true;
            }
        }
    }

    // replanning
    auto t3 = ros::Time::now();
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
    auto dt3 = ros::Time::now() - t3;

    auto t4 = ros::Time::now();
    // converting rigid body path to drone paths
    planner->convert_path_to_drones_paths(pth, drone_path1, drone_path2);
    auto dt4 = ros::Time::now() - t4;

    auto t5 = ros::Time::now();
    drone_path1_pub.publish(drone_path1);
    drone_path2_pub.publish(drone_path2);
    auto dt5 = ros::Time::now() - t5;

    auto dt0 = ros::Time::now() - t0;

    return true;

    // STATS  TODO:make ros parameter for printing the stats
    planning_times++;

    float dt_to_msec[times_num] = {dt0.toSec() * 1000.0, dt1.toSec() * 1000.0, dt2.toSec() * 1000.0,
                                   dt3.toSec() * 1000.0, dt4.toSec() * 1000.0, dt5.toSec() * 1000.0};
    sum_times[0] += dt_to_msec[0];
    sum_times[1] += dt_to_msec[1];
    sum_times[2] += dt_to_msec[2];
    sum_times[3] += dt_to_msec[3];
    sum_times[4] += dt_to_msec[4];
    sum_times[5] += dt_to_msec[5];

    max_times[0] = std::max(max_times[0], dt_to_msec[0]);
    max_times[1] = std::max(max_times[1], dt_to_msec[1]);
    max_times[2] = std::max(max_times[2], dt_to_msec[2]);
    max_times[3] = std::max(max_times[3], dt_to_msec[3]);
    max_times[4] = std::max(max_times[4], dt_to_msec[4]);
    max_times[5] = std::max(max_times[5], dt_to_msec[5]);

    printf("Total planning time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[0], sum_times[0] / planning_times, max_times[0]);
    printf("\t-Cylindr update time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[1], sum_times[1] / planning_times,
           max_times[1]);
    printf("\t-Prev_path_vlid time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[2], sum_times[2] / planning_times,
           max_times[2]);
    printf("\t-plan->solve()  time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[3], sum_times[3] / planning_times,
           max_times[3]);
    printf("\t-RB to drones   time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[4], sum_times[4] / planning_times,
           max_times[4]);
    printf("\t-Publish pths   time->\t Current: %4f \tAverage: %4f msec \tMax: %4f msec\n", dt_to_msec[5], sum_times[5] / planning_times,
           max_times[5]);
}

void leader_odom_callback(nav_msgs::OdometryConstPtr odom) { drone1_tf->pose = odom->pose; }

void follower_odom_callback(nav_msgs::OdometryConstPtr odom) { drone2_tf->pose = odom->pose; }

int main(int argc, char **argv)
{

    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    // {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");

    // instantiate listener
    auto listener = new tf::TransformListener();

    printf("Ititializing...\n");
    // initialize planner
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    planner = new ompl_rope_planning::planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};

    // Manually setting first drones odometry to start position of formations
    drone1_tf->pose.pose.position.x = start[0] + 0.2; // not sure if sign is correct
    drone1_tf->pose.pose.position.y = start[1];
    drone1_tf->pose.pose.position.z = start[2];

    drone2_tf->pose.pose.position.x = start[0] - 0.2; // not sure if sign is correct
    drone2_tf->pose.pose.position.y = start[1];
    drone2_tf->pose.pose.position.z = start[2];

    printf("Planning with goal : %f %f %f\n", goal[0], goal[1], goal[2]);
    planner->setStartGoal(start, goal);

    // create ros publishers
    printf("Creating publishers\n");
    dyn_path_pub = nodeHandle.advertise<drones_rope_planning::rigid_body_dynamic_path>("/dynamicRigiBodyPath", 1);

    drone_path1_pub = nodeHandle.advertise<nav_msgs::Path>("/drone1Path", 1);
    drone_path2_pub = nodeHandle.advertise<nav_msgs::Path>("/drone2Path", 1);

    // create ros service
    ros::ServiceServer service = nodeHandle.advertiseService("ompl_realtime_planning", planning_service);

    // check if is in simulation mode
    int is_simulation;
    ros::param::get("/is_simulation", is_simulation);

    // check if string is equal
    if (!is_simulation)
    {
        printf("Running in demo mode\n");
        // load leader name from param
        std::string leader_name, follower_name;

        ros::param::get("/cf_leader_name", leader_name);
        ros::param::get("/cf_follower_name", follower_name);

        // Drones odom subscribers
        char leader_topic_name[60], follower_topic_name[60];

        sprintf(leader_topic_name, "/pixy/vicon/%s/%s/odom", leader_name.c_str(), leader_name.c_str());
        sprintf(follower_topic_name, "/pixy/vicon/%s/%s/odom", follower_name.c_str(), follower_name.c_str());

        printf("Subscribing to %s and %s\n", leader_topic_name, follower_topic_name);

        ros::Subscriber leader_sub = nodeHandle.subscribe(leader_topic_name, 1, leader_odom_callback);
        ros::Subscriber follower_sub = nodeHandle.subscribe(follower_topic_name, 1, follower_odom_callback);
        ROS_INFO("Service ready!");
        ros::spin();
    }
    else
    {
        printf("Running in simulation mode\n");
        ros::Rate rate(10);
        tf::StampedTransform tf1, tf2;
        ROS_INFO("Service ready!");

        while (ros::ok())
        {
            try
            {
                listener->lookupTransform("/world", "/drone1", ros::Time(0), tf1);
                geometry_msgs::Point p;
                p.x = tf1.getOrigin().x();
                p.y = tf1.getOrigin().y();
                p.z = tf1.getOrigin().z();

                drone1_tf->pose.pose.position = p;
                // printf("Drone1 Twist: %f, %f, %f\n", twist1.linear.x, twist1.linear.y, twist1.linear.z);
            }
            catch (tf::TransformException ex)
            {
            }

            try
            {
                listener->lookupTransform("/world", "/drone2", ros::Time(0), tf2);
                geometry_msgs::Point p;
                p.x = tf2.getOrigin().x();
                p.y = tf2.getOrigin().y();
                p.z = tf2.getOrigin().z();

                drone2_tf->pose.pose.position = p;
                // printf("Drone2 Twist: %f, %f, %f\n", twist2.linear.x, twist2.linear.y, twist2.linear.z);
            }
            catch (tf::TransformException ex)
            {
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    ros::spin();
}
