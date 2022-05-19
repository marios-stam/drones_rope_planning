
#include "../../include/catenaries/catenary.hpp"

#define e 2.7182818284590452353602874713527
namespace catenaries
{
    float approximateAnumerically(float r)
    {
        // Solving equation r*A-sinh(A)=0 numerically
        float Ai, A, A_i;

        if (2 * r / e < 1)
        {
            if (r < 3)
                Ai = sqrt(6 * (r - 1));
            else
                Ai = log(2 * r) + log(log(2 * r));

            float max_error = 0.001;
            int counter = 0;
            float num, denum;

            while (r - sinh(Ai) / Ai > max_error)
            {
                num = sinh(Ai) - r * Ai;
                denum = cosh(Ai) - r;
                A_i = A_i - num / denum;
                counter = counter + 1;
            }
            A = Ai;
        }
        else
        {
            float A_approx;
            A_approx = 0.25 * (1 + 3 * log(2 * r)) + sqrt(2 * log(2 * r / e));
            A = A_approx;
        }

        return A;
    }

    problem_constants get_cat_problem_constants(Eigen::Vector2f P1, Eigen::Vector2f P2, float L)
    {
        bool inverse = false;
        // if x1>x2 then swap
        if (P1(0) > P2(0))
        {
            Eigen::Vector2f temp = P1;
            P1 = P2;
            P2 = temp;
            inverse = true;
        }

        float dx = P2(0) - P1(0);
        float dy = P2(1) - P1(1);

        float xb = (P1(0) + P2(0)) / 2;
        // float yb = (P1(0) + P2(0)) / 2;

        assert(L * L > dx * dx + dy * dy);

        assert(dx > 0);

        float r = sqrt(L * L - dy * dy) / dx;
        float A = approximateAnumerically(r);
        // std::cout << "dx: " << dx << " dy: " << dy << " r: " << r << " A: " << A << std::endl;

        if (A == 0)
        {
            std::cout << "A is zero" << std::endl;
        }
        problem_constants constants;

        constants.a = dx / (2 * A);
        constants.b = xb - constants.a * math_utils::tanhi(dy / L);
        constants.c = P1(1) - constants.a * cosh((P1(0) - constants.b) / constants.a);

        constants.x1 = P1(0);
        constants.x2 = P2(0);
        constants.inverse = inverse;

        return constants;
    }

    Eigen::Vector2f getCatenaryCurve2D_optimized_for_lowest_cat_point(Eigen::Vector2f P1, Eigen::Vector2f P2, float L)
    {
        /*
        2D Catenary curve calculation optimized for the lowest point
        starting from the middle point and then going right or left to find the lowest point
        similar to binary search
        */

        problem_constants constants = get_cat_problem_constants(P1, P2, L);
        // std::cout << "constants.a: " << constants.a << std::endl;
        // std::cout << "constants.b: " << constants.b << std::endl;
        // std::cout << "constants.c: " << constants.c << std::endl;
        // std::cout << "constants.x1: " << constants.x1 << std::endl;
        // std::cout << "constants.x2: " << constants.x2 << std::endl;
        // std::cout << "constants.inverse: " << constants.inverse << std::endl;

        float x = P1(0);
        float ddx = 0.01 * 1;
        float length = (P2(0) - P1(0)) / ddx + 1;
        Eigen::Vector2f xy;
        float middle_point = (P2(0) - P1(0)) / 2;

        float a = constants.a;
        float b = constants.b;
        float c = constants.c;

        float x_curr = middle_point;
        float y_curr = a * cosh((middle_point - b) / a) + c;

        float y_right = a * cosh((x_curr + ddx - b) / a) + c;
        float y_left = a * cosh((x_curr - ddx - b) / a) + c;

        bool initial_lowest_is_right;
        if (y_right < y_left)
        {
            initial_lowest_is_right = true;
            x_curr = middle_point + ddx;
        }
        else
        {
            initial_lowest_is_right = false;
            x_curr = middle_point - ddx;
        }

        bool lowest_is_right = initial_lowest_is_right;
        int counter = 0;
        while (initial_lowest_is_right == lowest_is_right)
        {

            y_right = a * cosh((x_curr + ddx - b) / a) + c;
            y_left = a * cosh((x_curr - ddx - b) / a) + c;

            lowest_is_right = y_right < y_left;

            if (lowest_is_right)
                x_curr = x_curr + ddx;
            else
                x_curr = x_curr - ddx;

            counter += 1;
        }

        Eigen ::Vector2f xy_lowest;
        xy_lowest(0) = x_curr;
        xy_lowest(1) = y_curr;

        return xy_lowest;
    }

    Eigen::Vector3f getCatenaryCurve3D_optimized_lowest_cat_point(Eigen::Vector3f P1, Eigen::Vector3f P2, float L)
    {
        float angle = math_utils::calculate2DAngleBetweenPoints(P1, P2);
        // convert radians to degrees
        angle = angle * 180 / M_PI;
        // float rotation[3] = {0, 0, -angle};
        Eigen::Vector3f rotation(0, 0, -angle);
        // std::cout << "rotation: " << rotation << std::endl;

        math_utils::Transformation trans = math_utils::Transformation(rotation, P1);

        auto coords2D = trans.transformPoint(P2);

        float coords2Dx = coords2D(0);
        float coords2Dy = coords2D(2);

        // std::cout << "coords2D:" << coords2D << std::endl;

        Eigen::Vector2f start2D(0, 0);
        Eigen::Vector2f end2D(coords2Dx, coords2Dy);

        // std::cout << "start2D: " << start2D << std::endl;
        // std::cout << "end2D: " << end2D << std::endl;

        Eigen::Vector2f points2D = getCatenaryCurve2D_optimized_for_lowest_cat_point(start2D, end2D, L);
        // std::cout << "points2D: " << points2D << std::endl;

        Eigen::Vector3f points3D = trans.inverseTransformPoint(Eigen::Vector3f(points2D[0], 0, points2D[1]));

        // std::cout << "points3D: " << points3D << std::endl;

        return points3D;
    }

    Eigen::Vector3f lowest_point_optimized(Eigen::Vector3f start, Eigen::Vector3f end, float L)
    {
        // std::cout << "start: " << start(0) << " " << start(1) << " " << start(2) << std::endl;
        // std::cout << "end: " << end(0) << " " << end(1) << " " << end(2) << std::endl;
        return getCatenaryCurve3D_optimized_lowest_cat_point(start, end, L);
    }

    std::vector<math_utils::Line2D> findBoundingLines(problem_constants prob_constants, Eigen::Vector3f lowest, float safety_hor_distance)
    {
        /*
        Finds the bounding lines of the catenary curve

        Parameters
        ----------
        prob_constants: constants of the catenary curve
        lowest: lowest point of the catenary curve

        safety_hor_distance: horizontal distance from the lowest point to the end of the catenary curve

        Returns
        -------
        output: std::vector<math_utils::Line2D> The 2 lines that bound the catenary curve (left and right)
        */

        float dx = 0.08;
        Eigen::Matrix2Xf xy = getCurvePoints(prob_constants, dx);

        Eigen::Vector2f vert_point(lowest(0), lowest(2));

        Eigen::Vector2f safety_start(xy.col(0)[0] - safety_hor_distance, xy.col(0)[1]);
        Eigen::Vector2f safety_end(xy.col(xy.cols() - 1)[0] + safety_hor_distance, xy.col(xy.cols() - 1)[1]);

        // Right line
        math_utils::Line2D *right_line;
        do
        {
            vert_point[1] -= 0.1;
            right_line = new math_utils::Line2D(vert_point, safety_end);

        } while (all_points_left_of_line(xy, *right_line) == false);

        // Left line
        math_utils::Line2D *left_line;
        do
        {
            vert_point[1] -= 0.1;
            left_line = new math_utils::Line2D(vert_point, safety_start);

        } while (all_points_right_of_line(xy, *left_line) == false);

        // Output
        std::vector<math_utils::Line2D> output;
        output.push_back(*left_line);
        output.push_back(*right_line);

        return output;
    }

    bool all_points_left_of_line(Eigen::Matrix2f points, math_utils::Line2D line)
    {
        for (int i = 0; i < points.cols(); i++)
        {
            if (!line.isPointLeft(points.col(i)))
            {
                return false;
            }
        }
        return true;
    }

    bool all_points_right_of_line(Eigen::Matrix2f points, math_utils::Line2D line)
    {
        for (int i = 0; i < points.cols(); i++)
        {
            if (line.isPointLeft(points.col(i)))
            {
                return false;
            }
        }
        return true;
    }

    Eigen::Matrix2Xf getCurvePoints(problem_constants constants, float dx)
    {
        /*
        Finds the points of the catenary curve given a dx
        Parameters
        ----------
        constants: constants of the catenary curve
        dx: step size

        Returns
        -------
        output: Eigen::Matrix2Xf The points of the catenary curve
        */

        float x1 = constants.x1;
        float x2 = constants.x2;
        float a = constants.a;
        float b = constants.b;
        float c = constants.c;
        float inverse = constants.inverse;

        float x = x1;

        const int length = (x2 - x1) / dx + 1;

        Eigen::Matrix2Xf xy;
        xy.resize(2, length);

        for (int i = 0; x < x2 - dx; i++)
        {
            xy(0, i) = x;
            xy(1, i) = a * cosh((x - b) / a) + c;

            x += dx;
        }

        xy(0, length - 1) = x2;
        xy(1, length - 1) = a * cosh((x2 - b) / a) + c;

        return xy;
    }

} // namespace catenaries
