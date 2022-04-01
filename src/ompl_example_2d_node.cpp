/*
 * ompl_example_2d_node.cpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/ompl_example_2d.hpp"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/*
nav_msgs::OccupancyGrid globalMap;

// occupancy map callback
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mapMsg) { globalMap = *mapMsg; }



bool isStateValid(const ob::State *state_check)
{
    // cast the abstract state type to the type we expect
    // const auto *state = state->as<ob::RealVectorStateSpace::StateType>();
    // const auto state = state_check->as<ob::CompoundState>();
    const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

    const auto x = state->values[0];
    const auto y = state->values[1];
    const auto z = state->values[2];

    const auto yaw = state->values[3];
    const auto drones_dis = state->values[4];
    const auto drones_angle = state->values[5];

    printf("x: %f, y: %f, z: %f, yaw: %f, drones_dis: %f, drones_angle: %f \n", x, y, z, yaw, drones_dis, drones_angle);

    return true;
}

ob::RealVectorBounds getBounds()
{
    ob::RealVectorBounds bounds(6);

    // Low
    bounds.setLow(0, -1.0); // x
    bounds.setLow(1, -1.0); // y
    bounds.setLow(2, -1.0); // z
    bounds.setLow(3, -1.0); // yaw
    bounds.setLow(4, -1.0); // drones_distance
    bounds.setLow(5, -1.0); // drones_angle

    // High
    bounds.setHigh(0, 13.0); // x
    bounds.setHigh(1, 13.0); // y
    bounds.setHigh(2, 13.0); // z
    bounds.setHigh(3, 13.0); // yaw
    bounds.setHigh(4, 13.0); // drones_distance
    bounds.setHigh(5, 13.0); // drones_angle

    return bounds;
}

void setStartGoal(ob::ScopedState<ob::SE3StateSpace> start, ob::ScopedState<ob::SE3StateSpace> goal)
{
    // define the start state
    start[0] = 0.0;
    start[1] = -2.5;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;

    // define the goal state
    goal[0] = 12.0;
    goal[1] = -4.0;
    goal[2] = 0.0;
    goal[3] = 0.0;
    goal[4] = 0.0;
    goal[5] = 0.0;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(6));

    // set the bounds for space
    ob::RealVectorBounds bounds = getBounds();

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // set start and goal

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);

    // setStartGoal(start, goal);
    // set start and goal
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;

    goal[0] = 5;
    goal[1] = 5;
    goal[2] = 5;
    goal[3] = 5;
    goal[4] = 5;
    goal[5] = 5;

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // print start and goal states

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}
*/

/*
class planner
{
public:
    void init_start(void)
    {
        if (!set_start)
            std::cout << "Initialized" << std::endl;
        set_start = true;
    }

    // void updateMap(std::shared_ptr<fcl::CollisionGeometry> map) { tree_obj = map; }
    void setBounds()
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

    void setStartGoal()
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
    // Constructor
    planner(void)
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
    ~planner() {}
    void replan(void)
    {
        if (path_smooth != NULL && set_start)
        {
            std::cout << "Total Points:" << path_smooth->getStateCount() << std::endl;
            if (path_smooth->getStateCount() <= 2)
                plan();
            else
            {
                for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
                {
                    if (!replan_flag)
                        replan_flag = !isStateValid(path_smooth->getState(idx));
                    else
                        break;
                }
                if (replan_flag)
                    plan();
                else
                    std::cout << "Replanning not required" << std::endl;
            }
        }
    }

    void plan(void)
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

private:
    // construct the state space we are planning in
    ob::StateSpacePtr space;

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef;

    // goal state
    double prev_goal[3];

    og::PathGeometric *path_smooth = NULL;

    bool replan_flag = false;

    ob::ScopedStatePtr start_state;
    ob::ScopedStatePtr goal_state;

    // std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

    // std::shared_ptr<fcl::CollisionGeometry> tree_obj;

    // Flag for initialization
    bool set_start = false;

    bool isStateValid(const ob::State *state_check)
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
};
*/

int main(int argc, char **argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");
    /*
    // ompl_example_2d::Planner2D planner_(nodeHandle);

    // start planning
    printf("Start planning\n");
    plan();
    printf("End planning\n");

    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    // occupancy map subscriber
    ros::Subscriber map_sub = nodeHandle.subscribe("/map", 10, mapCallback);

    while (ros::ok())
    {
        nav_msgs::Path plannedPath;
        // plannedPath = planner_.planPath(globalMap);

        // publish the planned path
        path_pub.publish(plannedPath);

        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    // planner planner_object;
    //
    // planner_object.plan();

    ompl_rope_planning::planner planner_object;

    planner_object.plan();

    return 0;
}
