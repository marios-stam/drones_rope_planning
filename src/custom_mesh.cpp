#pragma once

#include "../include/custom_mesh.hpp"
#include "../include/catenaries/catenary.hpp"
#include "../include/ompl_example_2d/fcl_mesh.hpp"

// include ros
#include "ros/ros.h"

namespace custom_mesh
{
    using namespace Eigen;

    CustomMesh::CustomMesh(float rope_length, float safe_drones_dist, float safe_lowest_point_dist)
    {

        L = rope_length;
        safe_drones_distance = safe_drones_dist;
        safe_lowest_point_distance = safe_lowest_point_dist;
        is_created = false;
        get_tris();
    }

    CustomMesh::~CustomMesh() {}

    void CustomMesh::create_custom_robot(float drones_distance, float theta)
    {
        get_V_2D_points(drones_distance, theta);

        create_3D_V_fcl_mesh(V_2D.p0, V_2D.p1, V_2D.lower, V_2D.upper);

        m.create_mesh(verts, tris);
    }

    void CustomMesh::get_V_2D_points(float drones_distance, float theta)
    {
        /*
        This function generated a 3D rigid trinagle body suitable for path planning of the drone swarm
        theta : represents the angle that is formed between the line connecting the drones and the horizontal plane
        */
        drones_formation_2_triangle_points(drones_distance, theta);

        V_3D.p0 = Vector3f(V_2D.p0(0), V_2D.p0(1), 0);
        V_3D.p1 = Vector3f(V_2D.p1(0), V_2D.p1(1), 0);

        Vector3f lower3D = catenaries::lowest_point_optimized(V_3D.p0, V_3D.p1, L);

        Vector2f lower2D = Vector2f(lower3D(0), lower3D(2));

        // add safety distances
        // WARNING !!! : maybe signs are wrong
        V_2D.p0(0) += safe_drones_distance;
        V_2D.p1(0) -= safe_drones_distance;

        V_2D.upper = Vector2f(lower2D(0), lower2D(1) + safe_lowest_point_distance);
        V_2D.lower = Vector2f(lower2D(0), lower2D(1) - safe_lowest_point_distance);
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
        V_2D.p0 = Vector2f(r * cos(theta), y_offset + r * sin(theta));
        V_2D.p1 = Vector2f(-r * cos(theta), y_offset + -r * sin(theta));
    }

    void CustomMesh::create_3D_V_fcl_mesh(Vector2f p0, Vector2f p1, Vector2f lower, Vector2f upper)
    {
        get_V_3D_points(p0, p1, lower, upper);

        m.create_mesh(verts, tris);
    }

    void CustomMesh::get_V_3D_points(Vector2f p0, Vector2f p1, Vector2f lower, Vector2f upper)
    {
        // Created a matrix with all the vertices needed for the 3D triangle
        float thickness = 0.3; // thickness of the triangle ,maybe should be a parameter
        float offset = 0;      // TODO: makes this 0 (used for comapring with thhe old one)

        verts.block(0, 0, 1, 3) = Vector3f(p0(0), offset - thickness / 2, p0(1)).transpose();
        verts.block(1, 0, 1, 3) = Vector3f(p0(0), offset + thickness / 2, p0(1)).transpose();

        verts.block(2, 0, 1, 3) = Vector3f(lower[0], offset - thickness / 2, lower[1]).transpose();
        verts.block(3, 0, 1, 3) = Vector3f(upper[0], offset - thickness / 2, upper[1]).transpose();

        verts.block(4, 0, 1, 3) = Vector3f(lower[0], offset + thickness / 2, lower[1]).transpose();
        verts.block(5, 0, 1, 3) = Vector3f(upper[0], offset + thickness / 2, upper[1]).transpose();

        verts.block(6, 0, 1, 3) = Vector3f(p1[0], offset - thickness / 2, p1[1]).transpose();
        verts.block(7, 0, 1, 3) = Vector3f(p1[0], offset + thickness / 2, p1[1]).transpose();
    }

    void CustomMesh::get_tris()
    {
        tris << 7, 5, 3, 7, 3, 6, 2, 6, 3, 2, 3, 0, 0, 3, 5, 0, 5, 1, 1, 4, 2, 1, 2, 0, 4, 7, 6, 4, 6, 2, 1, 5, 4, 5, 7, 4;
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

        get_V_3D_points(V_2D.p0, V_2D.p1, V_2D.lower, V_2D.upper);

        auto t0 = ros::Time::now();

        // m.create_mesh(verts, tris);
        m.update_mesh(verts);

        auto dt = ros::Time::now() - t0;
        tot_time += dt.toSec() * 1000;
    }

    fcl_checking::fcl_mesh *CustomMesh::get_fcl_mesh() { return &m; }
};