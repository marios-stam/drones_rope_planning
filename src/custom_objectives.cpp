#pragma once

#include "../include/ompl_example_2d/custom_objectives.hpp"

ClearanceObjective::ClearanceObjective(ob::SpaceInformationPtr si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh)
    : ob::StateCostIntegralObjective(si, true)
{
    // set checker pointer
    checker_ = checker;

    // set custom robot mesh pointer
    custom_rb_mesh_ = robot_mesh;
}

ob::Cost ClearanceObjective::stateCost(const ob::State *state_to_cost)
{
    printf("Calculating distance\n");
    const ob::RealVectorStateSpace::StateType *state = state_to_cost->as<ob::RealVectorStateSpace::StateType>();

    float pos[3] = {state->values[0], state->values[1], state->values[2]};
    const auto yaw = state->values[3];
    const float drones_dis = state->values[4];
    const float drones_angle = state->values[5];

    auto t0 = ros::Time::now();

    // update dynamic robot meshh
    custom_rb_mesh_->update_mesh(drones_dis, drones_angle);

    checker_->update_robot(custom_rb_mesh_->get_fcl_mesh());

    //  apply yaw rotation
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    q = q.normalize();

    float quat[4] = {q.x(), q.y(), q.z(), q.w()};
    checker_->setRobotTransform(pos, quat);

    // calculate distance
    float dist = checker_->get_distance();

    /*
    // calculate distance without fcl because
    //distance of kdop gemetry and cylinder  is not supported
    auto obs_positions = checker_->as<fcl_checking_realtime::checker>()->getObstaclesTransforms();

    float min_distance = std::numeric_limits<float>::max();
    for (int i = 0; i < obs_positions.rows(); i++)
    {
        float distance = std::sqrt(std::pow(obs_positions(i, 0) - pos[0], 2) + std::pow(obs_positions(i, 1) - pos[1], 2));
        min_distance = std::min(min_distance, distance);
    }
    float dist=min_distance;
    */

    // clearance
    float clearance = 1.0 / dist;

    return ob::Cost(clearance);
}

ob::OptimizationObjectivePtr getClearanceObjective(ob::SpaceInformationPtr si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh)
{
    return ob::OptimizationObjectivePtr(new ClearanceObjective(si, checker, robot_mesh));
}

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si, fcl_checker_base *checker, custom_mesh::CustomMesh *robot_mesh)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj = getClearanceObjective(si, checker, robot_mesh);

    ob::MultiOptimizationObjective *opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
};