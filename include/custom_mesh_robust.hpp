#pragma once

// custom classes
#include "ompl_example_2d/fcl_mesh.hpp"
#include "ompl_example_2d/problem_params.hpp"

namespace custom_mesh_robust
{
    // The 6 2D points are the following: (front,back)      1___2         4__5
    //  1. left_safety     (0,1)                             \   \        /  /
    //  2. left drone      (2,3)                              \   \      /  /
    //  3. lower2D         (5,7)                               \   \    /  /
    //  4. right drone     (8,9)                                \   \3 /  /
    //  5. right_safety    (10,11)                               \   \/  /
    //  6. intersection    (4,6)                                  \     /
    //                                                             \   /
    //                                                              \ /
    //                                                               6

    using namespace Eigen;

    struct data2D
    {
        Vector2f left_safety;
        Vector2f left_drone;
        Vector2f lower2D;
        Vector2f right_drone;
        Vector2f right_safety;
        Vector2f intersection;
    };

    struct data3D
    {
        Vector3f p0;
        Vector3f p1;
        Vector3f lower;
        Vector3f upper;
    };

    class CustomMesh
    {
    public:
        CustomMesh(float rope_length, problem_params::SafetyOffsets safety_offsets, float rb_thickness);

        ~CustomMesh();

        void create_custom_robot(float drones_distance, float theta);

        void get_V_2D_points(float drones_distance, float theta);

        void create_3D_V_fcl_mesh();

        void get_V_3D_points();

        void get_tris();

        void drones_formation_2_triangle_points(float drones_distance, float theta);

        void update_mesh(float drones_distance, float theta);

        fcl_checking::fcl_mesh *get_fcl_mesh();

        float get_lowest_z();

        data2D get_V_2D();

    private:
        float L;

        float safe_drones_horiz_offset;
        float safe_drones_vert_offset;

        float safe_lowest_point_distance;

        data2D V_2D;

        data3D V_3D;

        Matrix<float, 12, 3> verts;

        Matrix<float, 20, 3> tris;

        fcl_checking::fcl_mesh m;

        bool is_created;

        float thickness;
    };

}; // namespace custom_mesh
