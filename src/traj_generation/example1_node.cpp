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

void generate_traj_from_path(const nav_msgs::Path &wp);
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

void generate_traj_from_path(const nav_msgs::Path &wp)
{
    auto tc1 = std::chrono::high_resolution_clock::now();
    MatrixXd route;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    int waypoints_number = wp.poses.size();

    VectorXd ts = VectorXd::Ones(waypoints_number);
    float total_time = 10;              // sec
    float dur = total_time / ts.size(); // duration of each piece
    ts *= dur;

    route.resize(3, wp.poses.size());
    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        route.col(k) << pt(0), pt(1), pt(2);
    }

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);
    // ts = allocateTime(route, 3.0, 3.0);
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, waypoints_number - 2), ts);
    snapOpt.getTraj(minSnapTraj);
    auto tc2 = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::MatrixXd> pols = minSnapTraj.getPolMatrix();
    // matrix details
    std::cout << "matrix[0] size : " << pols[0].rows() << " x " << pols[0].cols() << std::endl;
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
    std::cout << "min_snap generation time : " << dt * 1000 << " msec" << std::endl;

    bool TEST_GENERATION_CORRECTNESS = false;
    if (TEST_GENERATION_CORRECTNESS)
    {
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
}

void path_hande_callback1(const nav_msgs::Path &wp) { generate_traj_from_path(wp); }
void path_hande_callback2(const nav_msgs::Path &wp) { generate_traj_from_path(wp); }

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

    auto path1_sub = nh.subscribe("/drone1Path", 1, path_hande_callback1);
    // auto path2_sub = nh.subscribe("/drone2Path", 1, path_hande_callback2);

    ros::spin();

    return 0;
}
