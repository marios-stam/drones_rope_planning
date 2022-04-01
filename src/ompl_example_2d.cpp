/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/ompl_example_2d.hpp"

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <limits>
#include <math.h>
#include <string>
#include <thread>

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
        // Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.1));
        // fcl::OcTree *tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
        // tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(6));

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        printf("Setting bounds\n");
        // set the bounds
        setBounds();

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        // create RealVector start_state
        ob::ScopedState<ob::RealVectorStateSpace> start_state(space);
        ob::ScopedState<ob::RealVectorStateSpace> goal_state(space);
        printf("Setting start and goal\n");
        // setStartGoal();
        start_state[0] = 0.0;
        start_state[1] = 0.0;
        start_state[2] = 0.0;
        start_state[3] = 0.0;
        start_state[4] = 0.0;
        start_state[5] = 0.0;

        goal_state[0] = 5;
        goal_state[1] = 5;
        goal_state[2] = 5;
        goal_state[3] = 5;
        goal_state[4] = 5;
        goal_state[5] = 5;

        printf("Setting validity checker\n");
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start_state, goal_state);

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
            // pth->printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    void planner::setStartGoal()
    {
        // set start and goal
        (*start_state)[0] = 0.0;
        (*start_state)[1] = 0.0;
        (*start_state)[2] = 0.0;
        (*start_state)[3] = 0.0;
        (*start_state)[4] = 0.0;
        (*start_state)[5] = 0.0;

        (*goal_state)[0] = 5;
        (*goal_state)[1] = 5;
        (*goal_state)[2] = 5;
        (*goal_state)[3] = 5;
        (*goal_state)[4] = 5;
        (*goal_state)[5] = 5;
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

        // fcl::CollisionObject treeObj((tree_obj));
        // fcl::CollisionObject aircraftObject(Quadcopter);

        // // check validity of state defined by pos & rot
        // fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
        // fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        // aircraftObject.setTransform(rotation, translation);
        // fcl::CollisionRequest requestType(1, false, 1, false);
        // fcl::CollisionResult collisionResult;
        // fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

        // return (!collisionResult.isCollision());
        return true;
    }

}

// int main(int /*argc*/, char ** /*argv*/)
// {
//     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

//     return 0;
// }