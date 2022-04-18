#pragma once
#include "../include/ompl_example_2d/fcl_checker.hpp"
#include "../include/custom_mesh.hpp"

namespace fcl_checking
{
    checker::checker() {}

    checker::checker(std::string environment_filename, std::string robot_filename)
    {
        loadEnvironment(environment_filename);
        loadRobot(robot_filename);
    }

    checker::~checker() {}

    void checker::loadEnvironment(std::string filename)
    {
        environment_mesh.load_stl(filename);
        float pos[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 1};
        environment_mesh.create_collision_object();
    }

    void checker::loadRobot(std::string filename)
    {
        robot_mesh = new fcl_mesh();
        robot_mesh->load_stl(filename);
        float pos[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 1};
        robot_mesh->create_collision_object();

        robot_mesh->set_transform(pos, q);
    }

    void checker::setRobotTransform(float pos[3], float q[4]) { robot_mesh->set_transform(pos, q); }

    bool checker::check_collision()
    {
        result.clear();
        fcl::collide(environment_mesh.collision_object, robot_mesh->collision_object, request, result);

        return result.isCollision();
    }

    void checker::setRobotMesh(fcl_mesh *mesh) { robot_mesh = mesh; }

    void checker::update_robot(fcl_checking::fcl_mesh *rb_mesh)
    {
        robot_mesh = rb_mesh;
        robot_mesh->create_collision_object();
    }

}
