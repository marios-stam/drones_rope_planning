#pragma once
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

// import tf transforms
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
// planners
#include <ompl/geometric/planners/est/EST.h>
// KPIECE
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

// RRT
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// Propabilistic
#include <ompl/geometric/planners/prm/PRM.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
// include optimization objectives
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/goals/GoalStates.h>

// Custom classes
#include "../custom_mesh.hpp"
#include "custom_goals.hpp"
#include "fcl_checker_realtime.hpp"
#include "optim_objectives.hpp"
#include "problem_params.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_rope_planning
{

    class planner
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        planner(problem_params::ProblemParams prob_prms);

        /*!
         * Destructor.
         */
        virtual ~planner();

        void setBounds(void);

        void plan(void);

        void replan(void);

        void setStartGoal(float start[6], float goal[6]);

        bool isStateValid(const ompl::base::State *state_check);

    private:
        // rope_length
        float L;

        // construct the state space we are planning in
        ompl::base::StateSpacePtr space;

        // construct an instance of  space information from this state space
        ompl::base::SpaceInformationPtr si;

        // create a problem instance
        ompl::base::ProblemDefinitionPtr pdef;

        ompl::geometric::PathGeometric *path_smooth = NULL;

        fcl_checker_base *checker;

        custom_mesh::CustomMesh *custom_robot_mesh;

        int dim;

        problem_params::ProblemParams prob_params;

        ob::PlannerPtr getPlanner(std::string, float range);
    };

}