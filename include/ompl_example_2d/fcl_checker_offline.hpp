#pragma once

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"
#include "fcl/narrowphase/collision_request-inl.h"
#include "fcl/narrowphase/collision_result-inl.h"

// custom classes
#include "fcl_checker_base.hpp"
#include "fcl_mesh.hpp"

namespace fcl_checking_offline
{
    class checker : public fcl_checker_base
    {
    public:
        checker();

        checker(std::string environment_filename, std::string robot_filename);

        void loadEnvironment(std::string filename);

        void loadEnvironment(int obs_number) override;

        bool check_collision(void);

        void setRobotMesh(fcl_checking::fcl_mesh *mesh);

        float get_distance(void);

    private:
        fcl_checking::fcl_mesh environment_mesh;

        fcl::CollisionRequest<float> request;

        fcl::CollisionResult<float> result;
    };

}
