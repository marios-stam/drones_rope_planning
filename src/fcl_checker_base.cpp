#pragma once
#include "../include/ompl_example_2d/fcl_checker_base.hpp"
#include "../include/custom_mesh.hpp"

fcl_checker_base::fcl_checker_base() {}

fcl_checker_base::~fcl_checker_base() {}

void fcl_checker_base::loadRobot(std::string filename)
{
    robot_mesh = new fcl_checking::fcl_mesh();
    robot_mesh->load_stl(filename);
    float pos[3] = {0, 0, 0};
    float q[4] = {0, 0, 0, 1};
    robot_mesh->create_collision_object();

    robot_mesh->set_transform(pos, q);
}

void fcl_checker_base::setRobotTransform(float pos[3], float q[4]) { robot_mesh->set_transform(pos, q); }

void fcl_checker_base::update_robot(fcl_checking::fcl_mesh *rb_mesh)
{
    robot_mesh = rb_mesh;
    robot_mesh->create_collision_object();
}
