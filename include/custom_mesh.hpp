#pragma once

// custom classes
#include "ompl_example_2d/fcl_mesh.hpp"
#include "ompl_example_2d/problem_params.hpp"

namespace custom_mesh
{
    using namespace Eigen;

    struct data2D
    {
        Vector2f p0;
        Vector2f p1;

        Vector2f lower;
        Vector2f upper;
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
        CustomMesh(float rope_length, problem_params::SafetyOffsets safety_offsets);

        ~CustomMesh();

        void create_custom_robot(float drones_distance, float theta);

        void get_V_2D_points(float drones_distance, float theta);

        void create_3D_V_fcl_mesh(Vector2f p0, Vector2f p1, Vector2f lower, Vector2f upper);

        void get_V_3D_points(Vector2f p0, Vector2f p1, Vector2f lower, Vector2f upper);

        void get_tris();

        void drones_formation_2_triangle_points(float drones_distance, float theta);

        void update_mesh(float drones_distance, float theta);

        fcl_checking::fcl_mesh *get_fcl_mesh();

        float get_lowest_z();

    private:
        float L;

        float safe_drones_horiz_offset;
        float safe_drones_vert_offset;

        float safe_lowest_point_distance;

        data2D V_2D;

        data3D V_3D;

        Matrix<float, 8, 3> verts;

        Matrix<float, 12, 3> tris;

        fcl_checking::fcl_mesh m;

        bool is_created;
    };

}; // namespace custom_mesh
