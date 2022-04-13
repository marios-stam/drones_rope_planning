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

    planner::planner(std::string robot_filename, std::string environment_filename)
    {
        checker = fcl_checking::checker();
        checker.loadEnvironment(environment_filename);
        checker.loadRobot(robot_filename);

        dim = 4;
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(dim));

        printf("Setting bounds\n");
        setBounds();

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        printf("Setting validity checker...\n");
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));

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
        bounds.setLow(0, -2.2);
        bounds.setLow(1, 2.8);
        bounds.setLow(2, 0.5);
        bounds.setLow(3, -3.14);
        // bounds.setLow(4, 0);
        // bounds.setLow(5, 0);

        bounds.setHigh(0, 2.2);
        bounds.setHigh(1, 5.0);
        bounds.setHigh(2, 2.5);
        bounds.setHigh(3, 3.14);
        // bounds.setHigh(4, 0);
        // bounds.setHigh(5, 0);

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
        // start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        // start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
        // goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
        // goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

        pdef->setStartAndGoalStates(start_state, goal_state);
    }

    void planner::plan()
    {

        // create a planner for the defined space
        // ob::PlannerPtr plan(new og::InformedRRTstar(si));
        // ob::PlannerPtr plan(new og::RRT(si));

        auto plan = std::make_shared<og::RRT>(si);
        printf("Setting  range...\n");
        plan->setRange(1.645338);

        // set the problem we are trying to solve for the planner
        printf("Setting  problem definition...\n");
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        printf("Setting  planner up...\n");
        plan->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);

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

            std::cout << "Simplifying path...\n";
            // pdef->simplifySolution(plan->getSolutionPath());

            std::cout << "Interpolating path...\n";

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth->interpolate(30);

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

    bool planner::isStateValid(const ob::State *state_check)
    {
        static int counter = 0;
        auto t0 = ros::Time::now();

        const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

        const auto x = state->values[0];
        const auto y = state->values[1];
        const auto z = state->values[2];

        const auto yaw = state->values[3];
        // const auto drones_dis = state->values[4];
        // const auto drones_angle = state->values[5];

        float pos[3] = {x, y, z};

        tf2::Quaternion q;

        q.setRPY(0, 0, yaw);
        // q = q.normalize();

        float quat[4] = {q.x(), q.y(), q.z(), q.w()};
        checker.setRobotTransform(pos, quat);

        bool result = !checker.check_collision();

        auto dt = ros::Time::now() - t0;

        counter++;
        if (counter % 5000 == 0)
        {
            // printf("x: %f, y: %f, z: %f, yaw: %f --> result: %d \n", x, y, z, yaw, result);
            std::cout << "\r"
                      << "Checking state: " << counter << " in " << dt.toSec() * 1000 << " msec";
        }

        return result;
    }

}
