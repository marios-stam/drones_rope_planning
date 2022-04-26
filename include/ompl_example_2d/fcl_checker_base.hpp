#pragma once

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/cylinder-inl.h"
#include "fcl/geometry/shape/cylinder.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"
#include "fcl/narrowphase/collision_request-inl.h"
#include "fcl/narrowphase/collision_result-inl.h"

#include "fcl_mesh.hpp"

class fcl_checker_base
{
public:
    fcl_checker_base();

    ~fcl_checker_base();

    virtual void loadEnvironment(int obs_number) = 0;

    // void checker::loadEnvironment(std::vector<realtime_obstacles::CylinderDefinition> cylinders_def) = 0;

    void loadRobot(std::string filename);

    void loadRobot(std::vector<fcl::Vector3<float>> fcl_vertices, std::vector<fcl::Triangle> fcl_tris);

    void setRobotTransform(float pos[3], float q[4]);

    void update_robot(fcl_checking::fcl_mesh *rb_mesh);

    virtual bool check_collision(void) = 0;

    /** \brief Cast this instance to a desired type. */
    template <typename T> T *as() { return dynamic_cast<T *>(this); };

protected:
    fcl_checking::fcl_mesh *robot_mesh;
};