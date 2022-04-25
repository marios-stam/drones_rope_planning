#pragma once
#include "../include/ompl_example_2d/fcl_checker_realtime.hpp"
#include "../include/custom_mesh.hpp"

namespace fcl_checking_realtime
{
    checker::checker() {}

    void checker::loadEnvironment(int obs_number)
    {
        // Load the environment
        env = new realtime_obstacles::Cylinders(obs_number);
    }

    void checker::updateEnvironmentTransforms()
    {
        // pass
    }

    void checker::update_env_obstacle_transform(int index, float pos[3], float q[4])
    {
        // update single obstacle transform
        env->set_cylinder_transform(index, pos, q);
    }

    bool checker::check_collision()
    {
        // use the collision detection function of the environment
        return env->collision_detection(robot_mesh->collision_object);
    }

}
