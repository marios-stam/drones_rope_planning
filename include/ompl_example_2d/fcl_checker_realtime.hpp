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
#include "fcl_mesh.hpp"

namespace fcl_checking_realtime
{
    class checker
    {
    public:
        checker();

        /*!
         * Destructor.
         */
        virtual ~checker();

        // environment
        void loadEnvironment(int obs_number);

        void updateEnvironmentTransforms();

        void update_env_obstacle_transform(int index, float pos[3], float q[4]);

        // robot
        void loadRobot(std::string filename);

        void loadRobot(std::vector<fcl::Vector3<float>> fcl_vertices, std::vector<fcl::Triangle> fcl_tris);

        void setRobotTransform(float pos[3], float q[4]);

        bool check_collision(void);

        void update_robot(fcl_checking::fcl_mesh *rb_mesh);

    private:
        fcl_checking::fcl_mesh *robot_mesh;

        realtime_obstacles::Cylinders *env;
    };

}

#endif // FCL_CHECKER_HPP