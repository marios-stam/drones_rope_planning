#pragma once

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"

#define STLLOADER_IMPLEMENTATION
#include "stlloader.hpp"

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

        /*!
         * Destructor.
         */
        virtual ~checker();

        void setEnvironment();

        void setRobotTransform(void);

        bool check_collision(void);

        void update_robot(void);

    private:
    };

    class fcl_mesh
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        fcl_mesh();

        /*!
         * Destructor.
         */
        virtual ~fcl_mesh();

        void load_stl(std::string filename);

        void create_collision_object(void);

        void create_mesh(void);

        void set_transform(float pos[], float quat[]);

        void update_mesh(const std::vector<fcl::Vector3<float>> &new_verts);

    private:
        std::vector<stlloader::Vertex> get_unique_vertices(void);

        stlloader::Mesh stl_mesh;

        fcl::BVHModel<fcl::OBBRSS<float>> *mesh;

        fcl::CollisionObject<float> *collision_object;
    };
}