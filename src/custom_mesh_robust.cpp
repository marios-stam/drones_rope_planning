#pragma once

#include "../include/custom_mesh_robust.hpp"
#include "../include/catenaries/catenary.hpp"
#include "../include/ompl_example_2d/fcl_mesh.hpp"

// include ros
#include "ros/ros.h"

namespace custom_mesh_robust
{
    using namespace Eigen;

    CustomMesh::CustomMesh(float rope_length, problem_params::SafetyOffsets safety_offsets, float rb_thickness, problem_params::V_robust V_robust)
    {
        L = rope_length;

        safe_drones_horiz_offset = safety_offsets.drones_horizontal;
        safe_drones_vert_offset = safety_offsets.drones_vertical;

        V_deltas = V_robust;

        safe_lowest_point_distance = safety_offsets.lowest_point;

        thickness = rb_thickness;

        is_created = false;

        get_tris();
    }

    CustomMesh::~CustomMesh() {}

    void CustomMesh::create_custom_robot(float drones_distance, float theta)
    {
        get_V_2D_points(drones_distance, theta);

        create_3D_V_fcl_mesh();

        m.create_mesh(verts, tris);
    }

    void CustomMesh::get_V_2D_points(float drones_distance, float theta)
    {
        /*
        This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
        theta : represents the angle that is formed between the line connecting the drones and the horizontal plane
        */
        drones_formation_2_triangle_points(drones_distance, theta);

        V_2D.lower2D = catenaries::getCatenaryCurve2D_optimized_for_lowest_cat_point(V_2D.left_drone, V_2D.right_drone, L);

        auto prob_consts = catenaries::get_cat_problem_constants(V_2D.left_drone, V_2D.right_drone, L);

        auto lines = catenaries::findBoundingLines(prob_consts, V_2D.lower2D, safe_drones_horiz_offset, V_deltas);

        auto right_line = lines[0];
        auto left_line = lines[1];

        V_2D.intersection = right_line.intersection(left_line);

        // Safety offsets
        // V_2D.left_safety = V_2D.left_drone - Eigen::Vector2f(safe_drones_horiz_offset, 0);   already done in findBoundingLines
        // V_2D.right_safety = V_2D.right_drone + Eigen::Vector2f(safe_drones_horiz_offset, 0); already done in findBoundingLines
        V_2D.intersection = V_2D.intersection - Eigen::Vector2f(0, safe_drones_vert_offset);

        // printf("V_2D.left_safety: %f %f\n", V_2D.left_safety[0], V_2D.left_safety[1]);
        // printf("V_2D.left_drone: %f %f\n", V_2D.left_drone[0], V_2D.left_drone[1]);
        // printf("V_2D.lower2D: %f %f\n", V_2D.lower2D[0], V_2D.lower2D[1]);
        // printf("V_2D.right_drone: %f %f\n", V_2D.right_drone[0], V_2D.right_drone[1]);
        // printf("V_2D.right_safety: %f %f\n", V_2D.right_safety[0], V_2D.right_safety[1]);
        // printf("V_2D.intersection: %f %f\n", V_2D.intersection[0], V_2D.intersection[1]);

        // The 6 2D points are the following: (front,back)              1___2         4__5
        //  1. left_safety     (0,1)                                     \   \        /  /
        //  2. left drone      (2,3)                                      \   \      /  /
        //  3. lower2D         (5,7)                                       \   \    /  /
        //  4. right drone     (8,9)                                        \   \3 /  /
        //  5. right_safety    (10,11)                                       \   \/  /
        //  6. intersection    (4,6)                                          \     /
        //                                                                     \   /
        //                                                                      \ /
        //                                                                       6
    }

    void CustomMesh::drones_formation_2_triangle_points(float drones_distance, float theta)
    {
        /*
        This function gets the distnace between the 2 drones and the angle they form
            with the horizontal plane. It deeems a circle with radius equal to the distance/2
            and calculates the points of the triangle.
        */

        // get the distance between the 2 drones
        float r = drones_distance / 2;
        float y_offset = 0;
        // get the points of the triangle
        V_2D.right_drone = Vector2f(r * cos(theta), y_offset + r * sin(theta));
        V_2D.left_drone = Vector2f(-r * cos(theta), y_offset + -r * sin(theta));
    }

    void CustomMesh::create_3D_V_fcl_mesh()
    {
        get_V_3D_points();

        m.create_mesh(verts, tris);
    }

    void CustomMesh::get_V_3D_points()
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

        // Created a matrix with all the vertices needed for the 3D triangle
        float offset = 0; // TODO: makes this 0 (used for comapring with the old one)

        verts.block(0, 0, 1, 3) = Vector3f(V_2D.left_safety(0), offset - thickness / 2, V_2D.left_safety(1)).transpose();
        verts.block(1, 0, 1, 3) = Vector3f(V_2D.left_safety(0), offset + thickness / 2, V_2D.left_safety(1)).transpose();

        verts.block(2, 0, 1, 3) = Vector3f(V_2D.left_drone[0], offset + thickness / 2, V_2D.left_drone[1]).transpose();
        verts.block(3, 0, 1, 3) = Vector3f(V_2D.left_drone[0], offset - thickness / 2, V_2D.left_drone[1]).transpose();

        verts.block(5, 0, 1, 3) = Vector3f(V_2D.lower2D[0], offset + thickness / 2, V_2D.lower2D[1]).transpose();
        verts.block(7, 0, 1, 3) = Vector3f(V_2D.lower2D[0], offset - thickness / 2, V_2D.lower2D[1]).transpose();

        verts.block(8, 0, 1, 3) = Vector3f(V_2D.right_drone[0], offset + thickness / 2, V_2D.right_drone[1]).transpose();
        verts.block(9, 0, 1, 3) = Vector3f(V_2D.right_drone[0], offset - thickness / 2, V_2D.right_drone[1]).transpose();

        verts.block(10, 0, 1, 3) = Vector3f(V_2D.right_safety[0], offset + thickness / 2, V_2D.right_safety[1]).transpose();
        verts.block(11, 0, 1, 3) = Vector3f(V_2D.right_safety[0], offset - thickness / 2, V_2D.right_safety[1]).transpose();

        verts.block(4, 0, 1, 3) = Vector3f(V_2D.intersection[0], offset + thickness / 2, V_2D.intersection[1]).transpose();
        verts.block(6, 0, 1, 3) = Vector3f(V_2D.intersection[0], offset - thickness / 2, V_2D.intersection[1]).transpose();
    }

    void CustomMesh::get_tris()
    {
        tris << 9, 5, 8, 4, 5, 0, 2, 7, 3, 1, 6, 4, 1, 4, 0, 6, 11, 10, 6, 10, 4, 7, 11, 6, 10, 11, 9, 9, 7, 5, 8, 10, 9, 4, 10, 5, 10, 8, 5, 5, 2, 0,
            1, 0, 2, 2, 5, 7, 3, 1, 2, 6, 1, 7, 1, 3, 7, 7, 9, 11;
    }

    void CustomMesh::update_mesh(float drones_distance, float theta)
    {
        if (!is_created)
        {
            create_custom_robot(drones_distance, theta);
            is_created = true;
            return;
        }

        static float tot_time = 0;

        get_V_2D_points(drones_distance, theta);

        get_V_3D_points();

        auto t0 = ros::Time::now();

        // m.create_mesh(verts, tris);
        m.update_mesh(verts);

        auto dt = ros::Time::now() - t0;
        tot_time += dt.toSec() * 1000;
    }

    fcl_checking::fcl_mesh *CustomMesh::get_fcl_mesh() { return &m; }

    float CustomMesh::get_lowest_z()
    {
        // return it with - since the start is at zero and it goes lower
        return -V_2D.lower2D(1);
    }

    data2D CustomMesh::get_V_2D() { return V_2D; }
};