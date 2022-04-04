#pragma once
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "fcl/config.h"
#include "fcl/fcl.h"
#include "fcl/geometry/collision_geometry-inl.h"

#include "fcl/geometry/collision_geometry.h"

#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision.h"

#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"

#include "fcl/geometry/bvh/BVH_model.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"

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

        std::shared_ptr<fcl::CollisionGeometry<double>> Quadcopter;

        // std::shared_ptr<fcl::CollisionGeometry> tree_obj;

        // Flag for initialization
        bool set_start = false;

        bool isStateValid(const ompl::base::State *state_check);
    };

}