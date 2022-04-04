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
    void planner::init_start(void)
    {
        if (!set_start)
            std::cout << "Initialized" << std::endl;
        set_start = true;
    }

    planner::planner(void)
    {
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(6));

        printf("Setting bounds\n");
        setBounds();

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        printf("Setting validity checker...\n");
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        printf("Setting start and goal\n");
        planner::setStartGoal();

        // set Optimizattion objective
        // pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        std::cout << "Initialized: " << std::endl;
    }

    // Destructor
    planner::~planner() {}

    void planner::setBounds()
    {
        ob::RealVectorBounds bounds(6);
        bounds.setLow(0, -1.0);
        bounds.setLow(1, -1.0);
        bounds.setLow(2, -1.0);
        bounds.setLow(3, -1.0);
        bounds.setLow(4, -1.0);
        bounds.setLow(5, -1.0);

        bounds.setHigh(0, 11.0);
        bounds.setHigh(1, 11.0);
        bounds.setHigh(2, 11.0);
        bounds.setHigh(3, 11.0);
        bounds.setHigh(4, 11.0);
        bounds.setHigh(5, 11.0);

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    }

    void planner::setStartGoal()
    {
        // create RealVector start_state
        ob::ScopedState<> start_state(planner::space);
        ob::ScopedState<> goal_state(planner::space);

        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0.0;
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0.0;
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = 0.0;
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = 0.0;

        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = 5;
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = 5;
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = 5;
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = 5;
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = 5;
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = 5;

        pdef->setStartAndGoalStates(start_state, goal_state);
    }

    void planner::plan()
    {

        // create a planner for the defined space
        // ob::PlannerPtr plan(new og::InformedRRTstar(si));
        ob::PlannerPtr plan(new og::RRT(si));

        // set the problem we are trying to solve for the planner
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        plan->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = plan->solve(2);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution:" << std::endl;
            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth->printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    bool planner::isStateValid(const ob::State *state_check)
    {
        const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

        const auto x = state->values[0];
        const auto y = state->values[1];
        const auto z = state->values[2];

        const auto yaw = state->values[3];
        const auto drones_dis = state->values[4];
        const auto drones_angle = state->values[5];

        return true;
    }

}
