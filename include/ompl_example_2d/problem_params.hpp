#pragma once

#include "../include/moving_obstacles.hpp"
#include "ros/ros.h"

// TODO: make parameters for:
//  - fixing invalid start state distance
//  - fixing invalid goal state distance
//  - distance to goal resetting

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

    enum SimplifyingPath
    {
        FAST,
        FULL,
        NONE
    };

    struct time_allocation
    {
        double velocity;
        double acceleration;
    };

    struct RealTimeSettings
    {
        bool setting_new_start;
        bool replan_only_if_not_valid;
        float fix_invalid_start_dist;
        float fix_invalid_goal_dist;
        float distance_to_goal_resetting;
        float replanning_interval;
        int simplifying_path;
        time_allocation time_alloc;
    };

    struct V_robust
    {
        float dx;
        float dy;
    };

    struct ProblemParams
    {
        // planning
        int planning_type;

        // realtime planning settings
        RealTimeSettings realtime_settings;

        std::string planner_algorithm;

        float timeout;
        float L;

        // state validation
        float val_check_resolution;
        float range;
        bool use_ground_collision_check;

        float thickness;
        // safety offsets
        SafetyOffsets safety_offsets;

        V_robust v_robust;

        // robot
        std::string robot_filename;

        // environment
        std::string env_filename;
        std::vector<realtime_obstacles::CylinderDefinition> obstacles_config;

        // start-goal
        int goal_type;
        std::map<std::string, double> start, goal;

        // bounds
        std::map<std::string, std::array<double, 6>> bounds;

        // path simplification
        bool simplify_path;
        int path_interpolation_points;
    };

    ProblemParams getProblemParams(ros::NodeHandle &nh);

    void printProblemParams(ProblemParams params);
}; // namespace rope_planning