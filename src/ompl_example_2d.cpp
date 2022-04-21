/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/ompl_example_2d.hpp"

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_rope_planning
{
    ProblemParams getProblemParams(ros::NodeHandle &nh)
    {
        ProblemParams pdef;
        ros::param::get("/planning/timeout", pdef.timeout);
        ros::param::get("/planning/rope_length", pdef.L);
        ros::param::get("/planning/env_mesh", pdef.env_filename);
        printf("Environment filename: %s\n", pdef.env_filename.c_str());
        ros::param::get("/planning/start", pdef.start_pos);
        ros::param::get("/planning/goal", pdef.goal_pos);

        XmlRpc::XmlRpcValue bounds;
        ros::param::get("/planning/bounds", bounds);
        // nh.getParam("/planning/bounds", pdef.bounds);
        XmlRpc::XmlRpcValue low_bounds = bounds["low"];
        XmlRpc::XmlRpcValue high_bounds = bounds["high"];

        pdef.bounds["low"][0] = low_bounds[0];
        pdef.bounds["low"][1] = low_bounds[1];
        pdef.bounds["low"][2] = low_bounds[2];
        pdef.bounds["low"][3] = ((double)low_bounds[3]) * M_PI / 180.0; // convert to radians
        pdef.bounds["low"][4] = ((double)low_bounds[4]) * pdef.L;
        pdef.bounds["low"][5] = ((double)low_bounds[5]) * M_PI / 180.0; // convert to radians

        pdef.bounds["high"][0] = high_bounds[0];
        pdef.bounds["high"][1] = high_bounds[1];
        pdef.bounds["high"][2] = high_bounds[2];
        pdef.bounds["high"][3] = ((double)high_bounds[3]) * M_PI / 180.0; // convert to radians
        pdef.bounds["high"][4] = ((double)high_bounds[4]) * pdef.L;
        pdef.bounds["high"][5] = ((double)high_bounds[5]) * M_PI / 180.0; // convert to radians

        ros::param::get("/planning/planner_algorithm", pdef.planner_algorithm);

        ros::param::get("/planning/val_check_resolution", pdef.val_check_resolution);
        ros::param::get("/planning/range", pdef.range);

        ros::param::get("/planning/simplify_path", pdef.simplify_path);

        ros::param::get("/planning/path_interpolation_points", pdef.path_interpolation_points);

        return pdef;
    }

    void printProblemParams(ProblemParams params)
    {
        printf("\n\n");
        printf("=================================== PROBLEM PARAMETERS: ===================================\n");
        printf("\tTimeout: %f\n", params.timeout);
        printf("\tRope length: %f\n", params.L);
        printf("\tEnvironment filename: %s\n", params.env_filename.c_str());
        printf("\tStart position: %f, %f, %f, \n", params.start_pos["x"], params.start_pos["y"], params.start_pos["z"]);
        printf("\tGoal position: %f, %f, %f, \n", params.goal_pos["x"], params.goal_pos["y"], params.goal_pos["z"]);
        printf("\tBounds:\n");
        printf("\t\tlow: %f, %f, %f, %f, %f, %f\n", params.bounds["low"][0], params.bounds["low"][1], params.bounds["low"][2],
               params.bounds["low"][3], params.bounds["low"][4], params.bounds["low"][5]);
        printf("\t\thigh: %f, %f, %f, %f, %f, %f\n", params.bounds["high"][0], params.bounds["high"][1], params.bounds["high"][2],
               params.bounds["high"][3], params.bounds["high"][4], params.bounds["high"][5]);

        printf("\tPlanner algorithm: %s\n", params.planner_algorithm.c_str());
        printf("\tValidity check resolution: %f\n", params.val_check_resolution);
        printf("\tRange: %f\n", params.range);

        printf("\tSimplify path: %d\n", params.simplify_path);
        printf("\tPath interpolation points: %d\n", params.path_interpolation_points);
        printf("===========================================================================================\n");

        printf("\n\n");
    }

    planner::planner(ProblemParams prob_prms)
    {
        checker = fcl_checking::checker();

        prob_params = prob_prms;
        printProblemParams(prob_params);
        try
        {
            std::string env_mesh = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/" + prob_params.env_filename + ".stl";
            checker.loadEnvironment(env_mesh);
        }
        catch (std::exception &e)
        {
            std::cout << "Using environment mesh from drones_path_planning" << std::endl;
            std::string env_mesh = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/" + prob_params.env_filename + ".stl";
            checker.loadEnvironment(env_mesh);
        }

        // checker.loadRobot(robot_filename);

        L = prob_params.L;

        custom_robot_mesh = new custom_mesh::CustomMesh(L);
        checker.setRobotMesh(custom_robot_mesh->get_fcl_mesh());

        dim = 6;
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(dim));

        printf("Setting bounds\n");
        setBounds();

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        printf("Setting validity checker...\n");
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));
        si->setStateValidityCheckingResolution(prob_params.val_check_resolution);

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set Optimizattion objective
        // pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        std::cout << "Initialized: " << std::endl;
    }

    // Destructor
    planner::~planner() {}

    void planner::setBounds()
    {
        ob::RealVectorBounds bounds(dim);

        float drones_angle = M_PI / 3;

        // Lower bounds
        double lower[6] = {prob_params.bounds["low"][0], prob_params.bounds["low"][1], prob_params.bounds["low"][2],
                           prob_params.bounds["low"][3], prob_params.bounds["low"][4], prob_params.bounds["low"][5]};

        double upper[6] = {prob_params.bounds["high"][0], prob_params.bounds["high"][1], prob_params.bounds["high"][2],
                           prob_params.bounds["high"][3], prob_params.bounds["high"][4], prob_params.bounds["high"][5]};

        bounds.setLow(0, lower[0]);
        bounds.setLow(1, lower[1]);
        bounds.setLow(2, lower[2]);
        bounds.setLow(3, lower[3]);
        bounds.setLow(4, lower[4]);
        bounds.setLow(5, lower[5]);

        // Upper bounds
        bounds.setHigh(0, upper[0]);
        bounds.setHigh(1, upper[1]);
        bounds.setHigh(2, upper[2]);
        bounds.setHigh(3, upper[3]);
        bounds.setHigh(4, upper[4]);
        bounds.setHigh(5, upper[5]);

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    }

    void planner::setStartGoal(float start[6], float goal[6])
    {
        // create RealVector start_state
        ob::ScopedState<> start_state(planner::space);
        ob::ScopedState<> goal_state(planner::space);

        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

        pdef->setStartAndGoalStates(start_state, goal_state);
    }

    void planner::plan()
    {

        // create a planner for the defined space
        // ob::PlannerPtr plan(new og::InformedRRTstar(si));
        // ob::PlannerPtr plan(new og::RRT(si));

        auto plan = std::make_shared<og::RRT>(si);
        printf("Setting  range...\n");
        plan->setRange(prob_params.range);

        // set the problem we are trying to solve for the planner
        printf("Setting  problem definition...\n");
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        printf("Setting  planner up...\n");
        plan->setup();

        // print the settings for this space
        printf("============================ OMPL SPACE SETTINGS: ============================\n");
        si->printSettings(std::cout);
        // printf("===============================================================================\n");

        // print the problem settings
        printf("============================ OMPL PROBLEM SETTINGS: ============================\n");
        pdef->print(std::cout);
        printf("================================================================================\n");

        // create termination condition
        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(10.0));
        // attempt to solve the problem within one second of planning time
        auto t0 = std::chrono::high_resolution_clock::now();
        ob::PlannerStatus solved = plan->solve(ptc);
        std::cout << std::endl;
        auto dt = std::chrono::high_resolution_clock::now() - t0;
        std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution :" << std::endl;

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

            if (prob_params.simplify_path)
            {
                std::cout << "Simplifying path...\n";
                og::PathSimplifier path_simplifier(si, pdef->getGoal());
                path_simplifier.simplify(*pth, 10.0);
            }

            if (prob_params.path_interpolation_points > 0)
            {
                std::cout << "Interpolating path...\n";
                // og::PathGeometric::InterpolationType interp_type = og::PathGeometric::CSPLINE;
                // pth->interpolate(interp_type);

                pth->interpolate(prob_params.path_interpolation_points);
            }

            std::cout << "Finale path :" << std::endl;
            pth->printAsMatrix(std::cout);

            // save path to file
            std::ofstream myfile;
            myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
            pth->printAsMatrix(myfile);
            myfile.close();
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    std::ostream &operator<<(std::ostream &os, const fcl::BVHBuildState &obj)
    {
        os << static_cast<std::underlying_type<fcl::BVHBuildState>::type>(obj);
        return os;
    }
    bool planner::isStateValid(const ob::State *state_check)
    {
        // printf("========================================================\n");
        static int counter = 0;
        static float total_time = 0;
        const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

        // const auto drones_dis = state->values[4];
        // const auto drones_angle = state->values[5];

        float pos[3];
        pos[0] = state->values[0];
        pos[1] = state->values[1];
        pos[2] = state->values[2];

        const auto yaw = state->values[3];

        const float drones_dis = state->values[4];
        const float drones_angle = state->values[5];

        auto t0 = ros::Time::now();
        custom_robot_mesh->update_mesh(drones_dis, drones_angle);
        auto dt = ros::Time::now() - t0;

        // printf("Updated mesh --> ");
        // std::cout << "mesh.state -->" << custom_robot_mesh->get_fcl_mesh()->get_fcl_mesh()->build_state << std::endl;

        checker.update_robot(custom_robot_mesh->get_fcl_mesh());

        // printf("Updated robot --> ");
        // std::cout << "mesh.state:" << custom_robot_mesh->get_fcl_mesh()->get_fcl_mesh()->build_state << std::endl;

        // apply yaw rotation
        tf2::Quaternion q;

        q.setRPY(0, 0, yaw);
        q = q.normalize();

        float quat[4] = {q.x(), q.y(), q.z(), q.w()};
        checker.setRobotTransform(pos, quat);

        // printf("Set tobot transform --> ");
        // std::cout << "mesh.state:" << custom_robot_mesh->get_fcl_mesh()->get_fcl_mesh()->build_state << std::endl;

        // check colllision
        bool result = !checker.check_collision();

        total_time += dt.toSec() * 1000; // sum msecs
        counter++;
        if ((counter % 1000) == 0)
        {
            // printf("x: %f, y: %f, z: %f, yaw: %f --> result: %d \n", x, y, z, yaw, result);
            std::cout << "\r"
                      << "Checking state: " << counter << " avrg time: " << total_time / counter << " msec";

            // std::cout << "\r"
            //   << "Checking state: " << counter << " msec";
        }

        return result;
    }
}
