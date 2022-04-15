#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
// include tf transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// float cosh(float z);
//
// float sinh(float z);

namespace math_utils
{
    float tanhi(float z);

    float calculate2DAngleBetweenPoints(Eigen::Vector3f p1, Eigen::Vector3f p2);

    Eigen::Matrix4f transformations_rotation_matrix(float angle, Eigen::Vector3f axis);

    Eigen::Matrix4f transformations_translation_matrix(Eigen::Vector3f trans);

    class Transformation
    {
    public:
        Transformation(Eigen::Vector3f rotation, Eigen::Vector3f translation);

        virtual ~Transformation();

        Eigen::Vector3f transformPoint(Eigen::Vector3f point);

        Eigen::Vector3f inverseTransformPoint(Eigen::Vector3f point);

    private:
        // transformation matrix
        Eigen::Matrix4f transformation_matrix;
        // inverse transformation matrix
        Eigen::Matrix4f inverse_transformation_matrix;
    };

    Eigen::Matrix4f getTransformationMatrix(Eigen::Vector3f rotation, Eigen::Vector3f translation);
} // namespace math_utils