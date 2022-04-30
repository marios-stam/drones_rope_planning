#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>
// include nav_msgs::Path
#include <nav_msgs/Path.h>

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

void path_hande_callback(const nav_msgs::Path &wp)
{
    MatrixXd route;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    int waypoints_number = wp.poses.size();
    printf("waypoints_number: %d\n", waypoints_number);
    VectorXd ts = VectorXd::Zero(waypoints_number);
    float total_time = 10;              // sec
    float dur = total_time / ts.size(); // duaration of each piece

    printf("Filling time durations...\n");
    for (int i = 0; i < waypoints_number; i++)
    {
        ts(i) = dur;
    }

    printf("Resizing route...\n");
    route.resize(3, wp.poses.size());
    printf("Filling route...\n");
    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        route.col(k) << pt(0), pt(1), pt(2);
    }

    printf("Resized route: %d x %d\n", route.rows(), route.cols());
    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();

    printf("Resizing iSS...\n");
    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);
    // ts = allocateTime(route, 3.0, 3.0);

    printf("snapOpt stuff...\n");
    auto tc1 = std::chrono::high_resolution_clock::now();
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    printf("calling generate...\n");
    snapOpt.generate(route.block(0, 1, 3, waypoints_number - 2), ts);
    printf("calling getTraj...\n");
    snapOpt.getTraj(minSnapTraj);
    auto tc2 = std::chrono::high_resolution_clock::now();

    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
    std::cout << "min_snap generation time : " << dt * 1000 << " msec" << std::endl;

    printf("Trajectory duration: %f\n", minSnapTraj.getTotalDuration());

    // Testing to prove that generation is working
    float t = 0;
    printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minSnapTraj.getPos(t).x(), minSnapTraj.getPos(t).y(), minSnapTraj.getPos(t).z());

    t = 0.5;
    printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minSnapTraj.getPos(t).x(), minSnapTraj.getPos(t).y(), minSnapTraj.getPos(t).z());

    t = 1;
    printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minSnapTraj.getPos(t).x(), minSnapTraj.getPos(t).y(), minSnapTraj.getPos(t).z());

    int ind = 0;
    printf("Path pos at ind=%d: %f, %f, %f\n", ind, wp.poses[ind].pose.position.x, wp.poses[ind].pose.position.y, wp.poses[ind].pose.position.z);

    ind = 1;
    printf("Path pos at ind=%d: %f, %f, %f\n", ind, wp.poses[ind].pose.position.x, wp.poses[ind].pose.position.y, wp.poses[ind].pose.position.z);

    printf("============================================================\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh("~");

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;

    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(1000);
    int groupSize = 10;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;

    auto path_sub = nh.subscribe("/drone1Path", 1, path_hande_callback);

    ros::spin();

    return 0;
}
