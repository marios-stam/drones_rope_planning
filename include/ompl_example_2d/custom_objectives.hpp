#pragma once

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/config.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>

// include costIntegralObjective
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

// include Cost
#include <ompl/base/Cost.h>
#include <ompl/base/goals/GoalState.h>

// custom classes
#include "../custom_mesh.hpp"
#include "fcl_checker_base.hpp"
#include "fcl_checker_realtime.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ob = ompl::base;

class ClearanceObjective : public ob::StateCostIntegralObjective
{
private:
    fcl_checker_base *checker_;
    custom_mesh::CustomMesh *custom_rb_mesh_;

public:
    ClearanceObjective(ob::SpaceInformationPtr si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh);

    ob::Cost stateCost(const ob::State *state_to_cost);

    custom_mesh::CustomMesh *custom_robot_mesh;
};

ob::OptimizationObjectivePtr getClearanceObjective(ob::SpaceInformationPtr si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh);

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh);