#ifndef FCL_MESH_HPP
#define FCL_MESH_HPP

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"

#define STLLOADER_IMPLEMENTATION
#include "stlloader.hpp"

// #define BVH_TYPE fcl::kIOS<float>
// #define BVH_TYPE fcl::RSS<float>
#define BVH_TYPE fcl::KDOP<float, 16>

namespace fcl_checking
{
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

        void create_mesh(Eigen::MatrixXf verts, Eigen::MatrixXf tris);

        void set_transform(float pos[], float quat[]);

        void update_mesh(const std::vector<fcl::Vector3<float>> new_verts);

        void update_mesh(const Eigen::MatrixXf new_verts);

        fcl::CollisionObject<float> *collision_object;

        fcl::BVHModel<BVH_TYPE> *get_fcl_mesh(void);

    private:
        std::vector<stlloader::Vertex> get_unique_vertices(void);

        stlloader::Mesh stl_mesh;

        fcl::BVHModel<BVH_TYPE> *mesh;
    };
}

#endif // FCL_MESH_HPP