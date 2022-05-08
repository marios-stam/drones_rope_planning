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
        // printf("Cylinders size already in env: %d\n", env->get_cylinders_size());
        float q[4] = {0, 0, 0, 1};
        float pos[3];
        for (int i = 0; i < cyls_config.size(); i++)
        {
            pos[0] = cyls_config[i].pos[0];
            pos[1] = cyls_config[i].pos[1];
            pos[2] = cyls_config[i].pos[2];

            q[0] = cyls_config[i].quat[0];
            q[1] = cyls_config[i].quat[1];
            q[2] = cyls_config[i].quat[2];
            q[3] = cyls_config[i].quat[3];

            // printf("Setting q: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
            // printf("Setting pos:%f %f %f\n", pos[0], pos[1], pos[2]);
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

    Eigen::MatrixX3f checker::getObstaclesTransforms() { env->get_cylinders_transforms(); }

    bool checker::check_collision()
    {
        // use the collision detection function of the environment
        return env->collision_detection(robot_mesh->collision_object);
    }

    float checker::get_distance()
    {
        // use the distance function of the environment
        return env->get_distance(robot_mesh->collision_object);
    }

    int checker::get_id_of_obstacle_nearest(Eigen::Vector3f pos)
    {
        Eigen::MatrixX3f tfs = env->get_cylinders_transforms();
        unsigned int nearest_id = 0;

        float min_dist = std::numeric_limits<float>::max();

        for (int i = 0; i < tfs.rows(); i++)
        {
            Eigen::Vector3f row_tf = tfs.row(i);
            Eigen::Vector3f dvector = pos - row_tf;

            float dist = dvector.norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_id = i;
            }
        }

        return nearest_id;
    }

    Eigen::Vector3f checker::get_velocity(int id) { return env->get_cylinder_velocity(id); }

    Eigen::Vector3f checker::get_position(int id) { return env->get_cylinder_position(id); }

    void checker::update_obstacles_config(drones_rope_planning::PlanningRequest::Request req)
    {
        auto cyls_config = req.config;
        for (int i = 0; i < cyls_config.size(); i++)
        {
            auto cyl = cyls_config[i];
            float pos[3] = {cyl.pos[0], cyl.pos[1], cyl.pos[2]};
            float q[4] = {cyl.quat[0], cyl.quat[1], cyl.quat[2], cyl.quat[3]};

            float vel[3] = {cyl.vel[0], cyl.vel[1], cyl.vel[2]};
            float angVel[3] = {cyl.angVel[0], cyl.angVel[1], cyl.angVel[2]};
            printf("Updating obstacle %d\n", i);
            printf("Pos: %f %f %f\n", pos[0], pos[1], pos[2]);
            printf("Quat: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
            printf("Vel: %f %f %f\n", vel[0], vel[1], vel[2]);
            printf("AngVel: %f %f %f\n", angVel[0], angVel[1], angVel[2]);

            env->set_cylinder_transform(i, pos, q);
            env->set_cylinder_velocities(i, vel, angVel);
        }
    }

} // namespace fcl_checking_realtime