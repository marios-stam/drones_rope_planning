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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// Custom classes
#include "fcl_checker.hpp"

namespace ompl_rope_planning
{
    class planner
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        planner();

        /*!
         * Destructor.
         */
        virtual ~planner();

        void init_start(void);

        void setBounds(void);

        void plan(void);

        void replan(void);

        void setStartGoal(void);

        ompl::base::ScopedStatePtr setGoal(void);

    private:
        // construct the state space we are planning in
        ompl::base::StateSpacePtr space;

        // construct an instance of  space information from this state space
        ompl::base::SpaceInformationPtr si;

        // create a problem instance
        ompl::base::ProblemDefinitionPtr pdef;

        // goal state
        double prev_goal[3];

        ompl::geometric::PathGeometric *path_smooth = NULL;

        bool replan_flag = false;

        // std::shared_ptr<fcl::CollisionGeometry> tree_obj;

        // Flag for initialization
        bool set_start = false;

        fcl_checking::checker checker;

        bool isStateValid(const ompl::base::State *state_check);
    };

}