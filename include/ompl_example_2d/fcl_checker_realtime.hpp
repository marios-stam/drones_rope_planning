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

// custom services and msgs
#include <drones_rope_planning/CylinderObstacleData.h>
#include <drones_rope_planning/PlanningRequest.h>

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

        void update_obstacles_config(drones_rope_planning::PlanningRequest::Request req);

        Eigen::MatrixX3f getObstaclesTransforms();

        bool check_collision(void);

        float get_distance(void);

        int get_id_of_obstacle_nearest(Eigen::Vector3f pos);

        Eigen::Vector3f get_velocity(int id);

        Eigen::Vector3f get_position(int id);

    private:
        realtime_obstacles::Cylinders *env;
    };

}

#endif // FCL_CHECKER_HPP