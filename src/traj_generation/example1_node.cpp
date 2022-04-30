#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>

using namespace std;
using namespace ros;
using namespace Eigen;

VectorXd allocateTime(const MatrixXd &wayPs, double vel, double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh_;

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts = VectorXd::Zero(3);
    ts << 1.0, 1.0, 1.0;

    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(1000);
    int groupSize = 10;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;

    int i = 2; // 30 waypoiints

    // first row of route
    printf("Generating route...\n");
    route.resize(3, i + 1);
    printf("Setting 1st row of route...\n");
    route.col(0) << 0.0, 0.0, 0.0;
    printf("Setting 2nd row of route...\n");

    route.col(1) << 1.0, 1.0, 1.0;
    printf("Setting 3rd row of route...\n");

    route.col(2) << 2.0, 2.0, 2.0;

    printf("Route genrated!\n");

    d0 = d1 = 0.0;
    for (int j = 0; j < groupSize && ok(); j++)
    {
        // printf("route rows: %d, cols: %d\n", route.rows(), route.cols());

        iS.col(0) << route.leftCols<1>();
        fS.col(0) << route.rightCols<1>();

        iSS << iS, Eigen::MatrixXd::Zero(3, 1);
        fSS << fS, Eigen::MatrixXd::Zero(3, 1);
        // ts = allocateTime(route, 3.0, 3.0);

        tc1 = std::chrono::high_resolution_clock::now();
        snapOpt.reset(iSS, fSS, route.cols() - 1);
        snapOpt.generate(route.block(0, 1, 3, i - 1), ts);
        snapOpt.getTraj(minSnapTraj);
        tc2 = std::chrono::high_resolution_clock::now();

        d1 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
    }

    std::cout << " MinSnap Comp. Time: " << d1 / groupSize * 1000 << " msec" << std::endl;

    printf("Snap ttrajectory duration: %f\n", minSnapTraj.getTotalDuration());
    float t = 1.0;
    printf("Snap trajectory pos at t=%f: %f, %f, %f\n", t, minSnapTraj.getPos(t).x(), minSnapTraj.getPos(t).y(), minSnapTraj.getPos(t).z());
    t = 2.0;
    printf("Snap trajectory pos at t=%f: %f, %f, %f\n", t, minSnapTraj.getPos(t).x(), minSnapTraj.getPos(t).y(), minSnapTraj.getPos(t).z());

    ros::spinOnce();
    lp.sleep();

    return 0;
}
