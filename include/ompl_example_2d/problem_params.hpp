#pragma once

#include "../include/moving_obstacles.hpp"
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

    enum GoalType
    {
        SIMPLE,
        SYMMETRICAL,
        SAMPLABLE,
        MULTIPLE_GOALS
    };

    enum PlanningType
    {
        STATIC,
        MOVING_OBSTACLES
    };

    struct ProblemParams
    {
        // planning
        int planning_type;

        std::string planner_algorithm;

        float timeout;
        float L;

        // state validation
        float val_check_resolution;
        float range;
        bool use_ground_collision_check;

        // robot
        std::string robot_filename;
        float thickness;
        // safety offsets
        SafetyOffsets safety_offsets;

        // environment
        std::string env_filename;
        std::vector<realtime_obstacles::CylinderDefinition> obstacles_config;

        // start-goal
        int goal_type;
        std::map<std::string, double> start_pos, goal_pos;

        // bounds
        std::map<std::string, std::array<double, 6>> bounds;

        // path simplification
        bool simplify_path;
        int path_interpolation_points;
    };

    ProblemParams getProblemParams(ros::NodeHandle &nh);

    void printProblemParams(ProblemParams params);
}; // namespace rope_planning