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

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

// custom services
#include <drones_rope_planning/CylinderObstacleData.h>
#include <drones_rope_planning/PlanningRequest.h>
#include <drones_rope_planning/rigid_body_dynamic_path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main_static_planning(int argc, char **argv)
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

int main_Cylinders_test()
{
    int number_of_obstacles = 2;
    realtime_obstacles::Cylinders obstacles(number_of_obstacles);

    // set position of obstacle
    float pos[3] = {0, 1, 0};
    float q[4] = {0, 0, 0, 1};

    obstacles.set_cylinder_transform(0, pos, q);

    pos[1] = 0;
    obstacles.set_cylinder_transform(1, pos, q);

    // create robot collision object
    fcl_checking::fcl_mesh robot;

    robot = fcl_checking::fcl_mesh();

    // load robot mesh
    robot.load_stl("/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/custom_V_robot.stl");

    robot.create_collision_object();

    float pos_robot[3] = {0, 0, 0};
    bool collision;

    for (float t = 0; t < 10; t += 0.1)
    {
        pos_robot[1] = cos(2 * M_PI * t / 5);
        robot.set_transform(pos_robot, q);
        printf("pos_robot:%f %f %f\n", pos_robot[0], pos_robot[1], pos_robot[2]);

        robot.set_transform(pos_robot, q);

        // check collision
        collision = obstacles.collision_detection(robot.collision_object);
        std::cout << "Collision: " << collision << std::endl;
    }

    return 0;
}

int main_fcl_checker_realtime_test()
{
    fcl_checker_base *fcl_checker;

    fcl_checker = new fcl_checking_realtime::checker();

    // robot
    fcl_checker->loadRobot("/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/custom_V_robot.stl");
    float pos[3] = {0, 0, 0};
    float q[4] = {0, 0, 0, 1};

    fcl_checker->setRobotTransform(pos, q);

    // environment
    fcl_checker->loadEnvironment(2);

    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 0;

    fcl_checker->as<fcl_checking_realtime::checker>()->update_env_obstacle_transform(0, pos, q);
    pos[1] = 1;
    fcl_checker->as<fcl_checking_realtime::checker>()->update_env_obstacle_transform(1, pos, q);

    float pos_robot[3] = {0, 0, 0};
    bool collision;

    for (float t = 0; t < 10; t += 0.1)
    {
        pos_robot[1] = cos(2 * M_PI * t / 5);
        fcl_checker->setRobotTransform(pos_robot, q);
        printf("pos_robot:%f %f %f\n", pos_robot[0], pos_robot[1], pos_robot[2]);

        // check collision
        collision = fcl_checker->as<fcl_checking_realtime::checker>()->check_collision();

        std::cout << "Collision: " << collision << std::endl;
    }

    return 0;
}

int main_realtime_planning(int argc, char **argv)
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
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};

    // create ros service
    // ros::ServiceServer service = nodeHandle.advertiseService("ompl_planning_service", &ompl_rope_planning::planner::plan, &planner);

    do
    {
        printf("Planning with goal : %f %f %f\n", goal[0], goal[1], goal[2]);
        planner.setStartGoal(start, goal);

        printf("Planning...\n");
        planner.plan();

        printf("Enter new goal position\n");
        printf("x: ");
        std::cin >> goal[0];
        printf("y: ");
        std::cin >> goal[1];
        printf("z: ");
        std::cin >> goal[2];

    } while (goal[2] != 69.0);

    printf("Done!\n");

    // exit ros node
    ros::shutdown();

    return 0;
}

// use planner as global variable
ompl_rope_planning::planner *planner;
ros::Publisher dyn_path_pub;

bool add(drones_rope_planning::PlanningRequest::Request &req, drones_rope_planning::PlanningRequest::Response &res)
{
    printf("===========================================================================\n");
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

        cylinders_def_vec.push_back(cylinder_def);
    }
    auto dt = std::chrono::high_resolution_clock::now() - t0;
    std::cout << "Time to create cylinders: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;

    auto t02 = std::chrono::high_resolution_clock::now();
    // printf("Setting the environment\n");
    planner->checker->as<fcl_checking_realtime::checker>()->updateEnvironmentTransforms(cylinders_def_vec);
    auto dt2 = std::chrono::high_resolution_clock::now() - t02;
    std::cout << "Time to update environment: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt2).count() << " ms" << std::endl;

    printf("Planning...\n");
    auto t03 = std::chrono::high_resolution_clock::now();
    auto dyn_path_msg = planner->plan();
    auto dt3 = std::chrono::high_resolution_clock::now() - t03;
    std::cout << "Time to plan: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt3).count() << " ms" << std::endl;

    dyn_path_pub.publish(dyn_path_msg);
    return true;
}

int main_realtime_planning_test(int argc, char **argv)
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
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};

    printf("Planning with goal : %f %f %f\n", goal[0], goal[1], goal[2]);
    planner.setStartGoal(start, goal);

    // create ros service
    ros::ServiceServer service = nodeHandle.advertiseService("ompl_realtime_planning", add);

    ROS_INFO("Service ready!");

    ros::spin();
}

//=======================================================================================================================

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

    // initialize planner
    problem_params::ProblemParams prob_prms = problem_params::getProblemParams(nodeHandle);
    planner = new ompl_rope_planning::planner(prob_prms);

    printf("Setting start and goal\n");
    float L = prob_prms.L;
    float goal[6] = {prob_prms.goal_pos["x"], prob_prms.goal_pos["y"], prob_prms.goal_pos["z"], 0.0, L * 0.5, 0.0};

    float start[6] = {prob_prms.start_pos["x"], prob_prms.start_pos["y"], prob_prms.start_pos["z"], 0.0, L * 0.5, 0.0};

    printf("Planning with goal : %f %f %f\n", goal[0], goal[1], goal[2]);
    planner->setStartGoal(start, goal);

    // create ros publisher
    dyn_path_pub = nodeHandle.advertise<drones_rope_planning::rigid_body_dynamic_path>("/dynamicRigiBodyPath", 1);

    // create ros service
    ros::ServiceServer service = nodeHandle.advertiseService("ompl_realtime_planning", add);

    ROS_INFO("Service ready!");

    ros::spin();
}
