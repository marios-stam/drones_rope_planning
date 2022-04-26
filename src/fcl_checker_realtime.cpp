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

    void checker::loadEnvironment(std::vector<realtime_obstacles::CylinderDefinition> cylinders_def)
    {
        // Load the environment
        printf("Loading environment with %d cylinders\n", cylinders_def.size());
        env = new realtime_obstacles::Cylinders(cylinders_def);
    }

    void checker::updateEnvironmentTransforms(std::vector<realtime_obstacles::CylinderDefinition> cyls_config)
    {
        printf("Cylinders size already in env: %d\n", env->get_cylinders_size());
        float q[4] = {0, 0, 0, 1};
        float pos[3];
        for (int i = 0; i < cyls_config.size(); i++)
        {
            pos[0] = cyls_config[i].pos[0];
            pos[1] = cyls_config[i].pos[1];
            pos[2] = cyls_config[i].pos[2];
            printf("Setting pos:%f %f %f\n", pos[0], pos[1], pos[2]);
            env->set_cylinder_transform(i, pos, q);
        }
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
