#include "../include/ompl_example_2d/fcl_checker_offline.hpp"
#include "../include/custom_mesh.hpp"

namespace fcl_checking_offline
{

    checker::checker() {}

    checker::checker(std::string environment_filename, std::string robot_filename)
    {
        loadEnvironment(environment_filename);
        loadRobot(robot_filename);
    }

    void checker::loadEnvironment(std::string filename)
    {
        // Load the environment
        printf("Loading environment from %s\n", filename.c_str());
        environment_mesh.load_stl(filename);
        float pos[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 1};
        environment_mesh.create_collision_object();
    }

    bool checker::check_collision()
    {
        result.clear();
        fcl::collide(environment_mesh.collision_object, robot_mesh->collision_object, request, result);

        return result.isCollision();
    }

    void checker::setRobotMesh(fcl_checking::fcl_mesh *mesh) { robot_mesh = mesh; }

    float checker::get_distance()
    {
        // return +inf
        return std::numeric_limits<float>::infinity();
    }

    void checker::loadEnvironment(int obs_number) { ROS_ERROR("This wa supposed to be never called\n"); }

} // namespace fcl_checking_offline