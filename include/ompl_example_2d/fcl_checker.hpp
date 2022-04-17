#ifndef FCL_CHECKER_HPP
#define FCL_CHECKER_HPP

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"
#include "fcl/narrowphase/collision_request-inl.h"
#include "fcl/narrowphase/collision_result-inl.h"

#include "fcl_mesh.hpp"

namespace fcl_checking
{
    class checker
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        checker();

        checker(std::string environment_filename, std::string robot_filename);
        /*!
         * Destructor.
         */
        virtual ~checker();

        void loadEnvironment(std::string filename);

        void loadRobot(std::string filename);

        void loadRobot(std::vector<fcl::Vector3<float>> fcl_vertices, std::vector<fcl::Triangle> fcl_tris);

        void setRobotTransform(float pos[3], float q[4]);

        bool check_collision(void);

        void update_robot(fcl_checking::fcl_mesh rb_mesh);

        void setRobotMesh(fcl_mesh mesh);

    private:
        fcl_mesh robot_mesh;

        fcl_mesh environment_mesh;

        fcl::CollisionRequest<float> request;

        fcl::CollisionResult<float> result;
    };

}

#endif // FCL_CHECKER_HPP