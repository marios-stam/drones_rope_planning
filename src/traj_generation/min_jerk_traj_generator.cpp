#include "traj_min_jerk.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// marios headers
#include <execution/TrajectoryPolynomialPieceMarios.h>

double time_alloc_vel, time_alloc_acc;

min_jerk::Trajectory generate_traj_from_path(const nav_msgs::Path &wp, geometry_msgs::Twist init_vel);

using namespace std;
using namespace ros;
using namespace Eigen;

// Gloabl variables TODO: move to a class
ros::Publisher traj_polynomial_pub;
geometry_msgs::Twist twist1, twist2;

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

min_jerk::Trajectory generate_traj_from_path(const nav_msgs::Path &wp, geometry_msgs::Twist init_vel)
{
    printf("Traj waypoints:%d\n", (int)wp.poses.size());

    auto tc1 = std::chrono::high_resolution_clock::now();
    MatrixXd route;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

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

    // The i-th column of "iSS/fSS" is a 3-dimensional specified (i-1)-order derivative.
    // The initial/final position is  stored in the first column of iSS/fSS.
    // The initial/final velocity is  stored in the first column of iSS/fSS.
    // The initial/final acceleration is  stored in the first column of iSS/fSS.

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();

    // setting initial velocity
    iS.col(1) << init_vel.linear.x, init_vel.linear.y, init_vel.linear.z;

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    ts = allocateTime(route, time_alloc_vel, time_alloc_acc); // TODO:Make that a parameter

    printf("Traj time allocation:%f\n", ts.sum());

    // jerkOpt.reset(iSS, fSS, route.cols() - 1);
    jerkOpt.reset(iS, fS, route.cols() - 1);
    jerkOpt.generate(route.block(0, 1, 3, waypoints_number - 2), ts);
    jerkOpt.getTraj(minJerkTraj);
    auto tc2 = std::chrono::high_resolution_clock::now();

    printf("Traj time:%f\n", std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count());

    return minJerkTraj;

    bool TEST_GENERATION_CORRECTNESS = false;
    if (TEST_GENERATION_CORRECTNESS)
    {
        printf("Trajectory duration: %f\n", minJerkTraj.getTotalDuration());

        // Testing to prove that generation is working
        float t = 0;
        printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minJerkTraj.getPos(t).x(), minJerkTraj.getPos(t).y(), minJerkTraj.getPos(t).z());

        t = 0.5;
        printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minJerkTraj.getPos(t).x(), minJerkTraj.getPos(t).y(), minJerkTraj.getPos(t).z());

        t = 1;
        printf("Trajectory pos at t=%f sec : %f, %f, %f\n", t, minJerkTraj.getPos(t).x(), minJerkTraj.getPos(t).y(), minJerkTraj.getPos(t).z());

        int ind = 0;
        printf("Path pos at ind=%d: %f, %f, %f\n", ind, wp.poses[ind].pose.position.x, wp.poses[ind].pose.position.y, wp.poses[ind].pose.position.z);

        ind = 1;
        printf("Path pos at ind=%d: %f, %f, %f\n", ind, wp.poses[ind].pose.position.x, wp.poses[ind].pose.position.y, wp.poses[ind].pose.position.z);

        printf("============================================================\n");
    }
}

void publish_pols(min_jerk::Trajectory traj, int id)
{
    printf("Publishing trajectory...\n");
    std::vector<Eigen::VectorXd> pols = traj.getPolMatrix();
    // matrix details
    // std::cout << "matrix[0] size : " << pols[0].rows() << " x " << pols[0].cols() << std::endl;
    int wps_number = pols[0].rows();
    printf("Trajectory waypoints:%d\n", wps_number);

    execution::TrajectoryPolynomialPieceMarios traj_pol;
    // cf_id
    traj_pol.cf_id = id;

    // pols
    for (int i = 0; i < pols[0].size(); i++) // TODO:make it without for loop
    {
        traj_pol.poly_x.push_back(pols[0](i));
        traj_pol.poly_y.push_back(pols[1](i));
        traj_pol.poly_z.push_back(pols[2](i));
    }

    // durations
    auto durations = traj.getDurations();
    traj_pol.durations.resize(durations.size());
    for (int i = 0; i < durations.size(); i++)
    {
        traj_pol.durations[i] = durations[i];
    }

    traj_polynomial_pub.publish(traj_pol);
}

void publish_path_to_traj(const nav_msgs::Path &wp, int id)
{
    if (wp.poses.size() <= 2)
    {
        // ROS_WARN("Waypoints number is less than 2.\n");
        return;
    }

    auto init_vel = id == 0 ? twist1 : twist2;

    auto t0 = std::chrono::high_resolution_clock::now();
    auto traj = generate_traj_from_path(wp, init_vel);
    auto t1 = std::chrono::high_resolution_clock::now();
    publish_pols(traj, id);
    auto t2 = std::chrono::high_resolution_clock::now();

    // std::cout << "Time taken for generation: " << (float)std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000 << " msec"
    //   << std::endl;
    // std::cout << "Time taken for publishing: " << (float)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000 << " msec"
    //   << std::endl;
    // std::cout << "Total time taken for generating trajectory " << id << ": "
    //   << (float)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t0).count() / 1000 << " msec" << std::endl;
    // auto vel = traj.getVel(0);
    // std::cout << "Velocity of drone " << id << " at t=0: " << vel.x() << " " << vel.y() << " " << vel.z() << std::endl;
}

void path_hande_callback1(const nav_msgs::Path &wp) { publish_path_to_traj(wp, 0); }

void path_hande_callback2(const nav_msgs::Path &wp) { publish_path_to_traj(wp, 1); }

void leader_odom_callback(nav_msgs::OdometryConstPtr odom) { twist1 = odom->twist.twist; }

void follower_odom_callback(nav_msgs::OdometryConstPtr odom) { twist2 = odom->twist.twist; }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh("~");

    try
    {
        ros::param::get("/planning/real_time_settings/time_allocation/velocity", time_alloc_vel);
        ros::param::get("/planning/real_time_settings/time_allocation/acceleration", time_alloc_acc);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("%s", e.what());
        printf("Could not load parameters,using default time allocation parameters.\n");
        time_alloc_vel = 4.0;
        time_alloc_acc = 4.0;
    }

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    MatrixXd route;

    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    int groupSize = 10;

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;

    auto path1_sub = nh.subscribe("/drone1Path", 1, path_hande_callback1);
    auto path2_sub = nh.subscribe("/drone2Path", 1, path_hande_callback2);

    traj_polynomial_pub = nh.advertise<execution::TrajectoryPolynomialPieceMarios>("/piece_pol", 1);

    // transform listener
    tf::TransformListener listener;
    ros::Rate rate(10);

    // set twists to zero
    twist1.linear = geometry_msgs::Vector3();
    twist2.linear = geometry_msgs::Vector3();

    /// check if is in simulation mode
    int is_simulation;
    try
    {
        ros::param::get("/is_simulation", is_simulation);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("%s", e.what());
        is_simulation = 0;
        twist1.linear.x = 0.0;
        twist1.linear.y = 0.0;
        twist1.linear.z = 0.0;

        twist2.linear.x = 0.0;
        twist2.linear.y = 0.0;
        twist2.linear.z = 0.0;
    }

    if (is_simulation)
    {
        ROS_INFO("Running in simulation mode.\n");

        while (ros::ok())
        {
            try
            {
                listener.lookupTwist("/drone1", "/world", ros::Time(0), ros::Duration(0.1), twist1);
                // printf("Drone1 Twist: %f, %f, %f\n", twist1.linear.x, twist1.linear.y, twist1.linear.z);
            }
            catch (tf::TransformException ex)
            {
            }

            try
            {
                listener.lookupTwist("/drone2", "/world", ros::Time(0), ros::Duration(0.1), twist2);
                // printf("Drone2 Twist: %f, %f, %f\n", twist2.linear.x, twist2.linear.y, twist2.linear.z);
            }
            catch (tf::TransformException ex)
            {
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        printf("Running in demo mode\n");
        // load leader name from param
        std::string leader_name, follower_name;

        ros::param::get("/cf_leader_name", leader_name);
        ros::param::get("/cf_follower_name", follower_name);

        // Drones odom subscribers
        char leader_topic_name[60], follower_topic_name[60];

        sprintf(leader_topic_name, "/pixy/vicon/%s/%s/odom", leader_name.c_str(), leader_name.c_str());
        sprintf(follower_topic_name, "/pixy/vicon/%s/%s/odom", follower_name.c_str(), follower_name.c_str());

        printf("Subscribing to %s and %s\n", leader_topic_name, follower_topic_name);

        ros::Subscriber leader_sub = nh.subscribe(leader_topic_name, 1, leader_odom_callback);
        ros::Subscriber follower_sub = nh.subscribe(follower_topic_name, 1, follower_odom_callback);

        ros::spin();
    }

    return 0;
}
