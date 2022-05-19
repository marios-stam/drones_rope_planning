#pragma once
#include "../../include/catenaries/math_utils.hpp"
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

    std::vector<math_utils::Line2D> findBoundingLines(problem_constants prob_constants, Eigen::Vector2f lowest, float safety_hor_distance);

    Eigen::Matrix2Xf getCurvePoints(problem_constants constants, float dx);

    bool all_points_left_of_line(Eigen::Matrix2Xf points, math_utils::Line2D line);
    bool all_points_right_of_line(Eigen::Matrix2Xf points, math_utils::Line2D line);

}