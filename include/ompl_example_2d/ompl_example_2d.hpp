/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

// Marios
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <ompl/config.h>

// #include "fcl/broadphase/broadphase.h"
// #include "fcl/collision.h"
// #include "fcl/config.h"
// #include "fcl/math/transform.h"
// #include "fcl/octree.h"
// #include "fcl/traversal/traversal_node_octree.h"

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

        ompl::base::ScopedStatePtr start_state;
        ompl::base::ScopedStatePtr goal_state;

        // std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

        // std::shared_ptr<fcl::CollisionGeometry> tree_obj;

        // Flag for initialization
        bool set_start = false;

        bool isStateValid(const ompl::base::State *state_check);
    };

}