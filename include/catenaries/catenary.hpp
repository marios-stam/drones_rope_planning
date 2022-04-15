#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

namespace catenaries
{
    struct problem_constants
    {
        float x1;
        float x2;
        bool inverse;
        float a;
        float b;
        float c;
    };

    float approximateAnumerically(float r);

    void getCatenaryCurve2D(Eigen::Vector2f P1, Eigen::Vector2f P2, float L, std::vector<Eigen::Vector3f> &curve);

    Eigen::Vector2f getCatenaryCurve2D_optimized_for_lowest_cat_point(Eigen::Vector2f P1, Eigen::Vector2f P2, float L);

    problem_constants get_cat_problem_constants(Eigen::Vector2f P1, Eigen::Vector2f P2, float L);

    void getCatenaryCurve3D(Eigen::Vector2f P1, Eigen::Vector2f P2, float L, std::vector<Eigen::Vector3f> &curve);

    Eigen::Vector3f getCatenaryCurve3D_optimized_lowest_cat_point(Eigen::Vector3f P1, Eigen::Vector3f P2, float L);

    Eigen::Vector3f lowest_point_optimized(Eigen::Vector3f start, Eigen::Vector3f end, float L);

}