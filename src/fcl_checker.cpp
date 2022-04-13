#pragma once
#include "../include/ompl_example_2d/fcl_checker.hpp"

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
        robot_mesh.load_stl(filename);
        float pos[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 1};
        robot_mesh.create_collision_object();

        robot_mesh.set_transform(pos, q);
    }

    void checker::setRobotTransform(float pos[3], float q[4]) { robot_mesh.set_transform(pos, q); }

    bool checker::check_collision()
    {

        result.clear();
        fcl::collide(environment_mesh.collision_object, robot_mesh.collision_object, request, result);

        return result.isCollision();
    }

}

// int main(int argc, char **argv)
// {
//     printf("Hello World mlkia\n");

//     fcl_checking::checker checker;
//     checker.loadEnvironment("/home/marios/thesis_ws/src/drones_rope_planning/resources/env-scene.stl");
//     checker.loadRobot("/home/marios/thesis_ws/src/drones_rope_planning/resources/robot-scene-triangle.stl");

//     checker.check_collision();
//     float pos[3] = {0, 0, 0};
//     float q[4] = {0, 0, 0, 1};
//     checker.setRobotTransform(pos, q);
//     checker.check_collision();

//     return 0;
// }
