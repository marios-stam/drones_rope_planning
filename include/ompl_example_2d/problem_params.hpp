#pragma once

#include "ros/ros.h"

namespace problem_params
{
    struct Bound
    {
        float arr[6];
    };

    struct SafetyOffsets
    {
        float drones_horizontal;
        float drones_vertical;

        float lowest_point;
    };

    struct ProblemParams
    {
        float timeout;
        float L;

        std::string robot_filename;
        std::string env_filename;

        float val_check_resolution;
        float range;

        std::map<std::string, double> start_pos, goal_pos;

        std::map<std::string, std::array<double, 6>> bounds;

        std::string planner_algorithm;

        bool simplify_path;
        int path_interpolation_points;

        // safety offsets
        SafetyOffsets safety_offsets;
    };

    ProblemParams getProblemParams(ros::NodeHandle &nh);

    void printProblemParams(ProblemParams params);
}; // namespace rope_planning