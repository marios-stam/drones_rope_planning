#ifndef FCL_CHECKER_HPP
#define FCL_CHECKER_HPP

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/cylinder-inl.h"
#include "fcl/geometry/shape/cylinder.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"
#include "fcl/narrowphase/collision_request-inl.h"
#include "fcl/narrowphase/collision_result-inl.h"

// custom classes
#include "../moving_obstacles.hpp"
#include "fcl_checker_base.hpp"
#include "fcl_mesh.hpp"

namespace fcl_checking_realtime
{
    class checker : public fcl_checker_base
    {
    public:
        checker();

        // environment
        void loadEnvironment(int obs_number) override;

        void loadEnvironment(std::vector<realtime_obstacles::CylinderDefinition> cylinders_def);

        void updateEnvironmentTransforms();

        void updateEnvironmentTransforms(std::vector<realtime_obstacles::CylinderDefinition>);

        void update_env_obstacle_transform(int index, float pos[3], float q[4]);

        Eigen::MatrixX3f getObstaclesTransforms();

        bool check_collision(void);

        float get_distance(void);

    private:
        realtime_obstacles::Cylinders *env;
    };

}

#endif // FCL_CHECKER_HPP