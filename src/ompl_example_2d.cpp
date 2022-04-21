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
    planner::planner(problem_params::ProblemParams prob_prms)
    {
        checker = fcl_checking::checker();

        prob_params = prob_prms;
        problem_params::printProblemParams(prob_params);

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

        custom_robot_mesh = new custom_mesh::CustomMesh(L, prob_params.safety_offsets);
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
        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));
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

        checker.update_robot(custom_robot_mesh->get_fcl_mesh());

        // apply yaw rotation
        tf2::Quaternion q;

        q.setRPY(0, 0, yaw);
        q = q.normalize();

        float quat[4] = {q.x(), q.y(), q.z(), q.w()};
        checker.setRobotTransform(pos, quat);

        // check colllision
        bool result = !checker.check_collision();

        total_time += dt.toSec() * 1000; // sum msecs
        counter++;

        if ((counter % 5000) == 0)
        {
            std::cout << "\r";

            std::cout << "State Validation: " << counter << " calls " << total_time / 1000 << " secs, " << total_time / counter << " msecs/call, "
                      << counter * 1000.0 / total_time << " calls/sec" << std::flush;
        }

        return result;
    }
}
