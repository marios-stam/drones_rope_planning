#include "../../include/catenaries/math_utils.hpp"

namespace math_utils
{
    float tanhi(float z)
    {
        float tmp = (1 + z) / (1 - z);
        return 0.5 * log(tmp);
    }

    float calculate2DAngleBetweenPoints(Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        float dx = p2(0) - p1(0);
        float dy = p2(1) - p1(1);
        float angle = atan2(dy, dx);
        return angle;
    }

    Eigen::Matrix4f getTransformationMatrix(Eigen::Vector3f rotation, Eigen::Vector3f translation)
    {
        /*
        Returns the transformation matrix frame specified by rotation and translation .

        Parameters
        ----------
        rotation : array,Vector3
            Rotation angles on x,y,z axes.
        translation : array,Vector3
            Translation of the new frame.
        Returns
        -------
        output : ndarray
            Transformation Matrix.
        */

        // convert rotation from degrees to radians
        rotation(0) = rotation(0) * M_PI / 180;
        rotation(1) = rotation(1) * M_PI / 180;
        rotation(2) = rotation(2) * M_PI / 180;

        Eigen::Vector3f xaxis(1, 0, 0);
        Eigen::Vector3f yaxis(0, 1, 0);
        Eigen::Vector3f zaxis(0, 0, 1);

        auto Rx = transformations_rotation_matrix(rotation(0), xaxis);
        auto Ry = transformations_rotation_matrix(rotation(1), yaxis);
        auto Rz = transformations_rotation_matrix(rotation(2), zaxis);

        // concatenate matrices
        auto R = Rx * Ry * Rz;

        // get translation matrix
        auto T = transformations_translation_matrix(-translation);
        // concatenate matrices
        Eigen::Matrix4f transformation_matrix;
        transformation_matrix = R * T;

        return transformation_matrix;
    }

    Eigen::Matrix4f transformations_rotation_matrix(float angle, Eigen::Vector3f axis)
    {
        /*
        Returns the rotation matrix for the given rotation angle and rotation axis.

        Parameters
        ----------
        angle : float
            Rotation angle in radians.
        axis : array,Vector3
            Rotation axis.
        Returns
        -------
        output : ndarray
            Rotation matrix.
        */

        Eigen::Matrix4f rotation_matrix;
        rotation_matrix.setIdentity();

        // convert rotation from degrees to radians
        // std::cout << "Angle: " << angle << "degrees" << std::endl;
        // angle = angle * 3.14 / 180;
        // std::cout << "Angle: " << angle << "radians" << std::endl;

        float sina = sin(angle);
        float cosa = cos(angle);
        // normalize direction vector
        axis.normalize();
        Eigen::Matrix3f R;
        R.setIdentity();

        R(0, 0) = cosa;
        R(1, 1) = cosa;
        R(2, 2) = cosa;

        // outer product of axis and axis
        Eigen::Matrix3f axis_outer_product = axis * axis.transpose();
        R += axis_outer_product * (1.0 - cosa);
        axis *= sina;

        Eigen::Matrix3f m;
        m << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1], axis[0], 0.0;

        R += m;

        Eigen::Matrix4f M;
        M.setIdentity();
        M.block(0, 0, 3, 3) = R;

        return M;
    }

    Eigen::Matrix4f transformations_translation_matrix(Eigen::Vector3f trans)
    {
        Eigen::Matrix4f T;
        T.setIdentity();
        // set block equal to trans
        T.block(0, 3, 3, 1) = trans;

        return T;
    }

    Transformation::Transformation(Eigen::Vector3f rotation, Eigen::Vector3f translation)
    {
        /*
        Initializes the transformation matrix.

        Parameters
        ----------
        rotation : array,Vector3
            Rotation angles on x,y,z axes.
        translation : array,Vector3
            Translation of the new frame.
        */

        transformation_matrix = getTransformationMatrix(rotation, translation);
        // calculate inverse

        inverse_transformation_matrix = transformation_matrix.inverse();
    }

    Transformation::~Transformation() {}

    Eigen::Vector3f Transformation::transformPoint(Eigen::Vector3f point)
    {
        /*
        Transforms a point from the original frame to the new frame.

        Parameters
        ----------
        point : array,Vector3
            Point to be transformed.
        Returns
        -------
        output : ndarray
            Transformed point.
        */

        Eigen::Vector4f point_homogeneous(point(0), point(1), point(2), 1);
        Eigen::Vector4f transformed_point = transformation_matrix * point_homogeneous;
        Eigen::Vector3f transformed_point_3d(transformed_point(0), transformed_point(1), transformed_point(2));

        return transformed_point_3d;
    }

    Eigen::Vector3f Transformation::inverseTransformPoint(Eigen::Vector3f point)
    {
        /*
        Transforms a point from the new frame to the original frame.

        Parameters
        ----------
        point : array,Vector3
            Point to be transformed.
        Returns
        -------
        output : ndarray
            Transformed point.
        */

        Eigen::Vector4f point_homogeneous(point(0), point(1), point(2), 1);
        Eigen::Vector4f transformed_point = inverse_transformation_matrix * point_homogeneous;
        Eigen::Vector3f transformed_point_3d(transformed_point(0), transformed_point(1), transformed_point(2));

        return transformed_point_3d;
    }
} // namespace math_utils