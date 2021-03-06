#include "../include/ompl_example_2d/problem_params.hpp"

namespace problem_params
{

    ProblemParams getProblemParams(ros::NodeHandle &nh)
    {
        ProblemParams pdef;
        // Select planning type
        std::map<std::string, int> planning_type_map = {// planning types
                                                        {"static", PlanningType::STATIC},
                                                        {"moving_obstacles", PlanningType::MOVING_OBSTACLES}};

        std::string planning_type_str;
        ros::param::get("/planning/planning_type", planning_type_str);
        pdef.planning_type = planning_type_map[planning_type_str];

        ros::param::get("/planning/timeout", pdef.timeout);
        ros::param::get("/planning/rope_length", pdef.L);
        ros::param::get("/planning/env_mesh", pdef.env_filename);
        printf("Environment filename: %s\n", pdef.env_filename.c_str());

        XmlRpc::XmlRpcValue bounds;
        ros::param::get("/planning/bounds", bounds);
        // nh.getParam("/planning/bounds", pdef.bounds);
        XmlRpc::XmlRpcValue low_bounds = bounds["low"];
        XmlRpc::XmlRpcValue high_bounds = bounds["high"];

        pdef.bounds["low"][0] = low_bounds[0];
        pdef.bounds["low"][1] = low_bounds[1];
        pdef.bounds["low"][2] = low_bounds[2];
        pdef.bounds["low"][3] = ((double)low_bounds[3]) * M_PI / 180.0; // convert to radians
        pdef.bounds["low"][4] = ((double)low_bounds[4]) * pdef.L;
        pdef.bounds["low"][5] = ((double)low_bounds[5]) * M_PI / 180.0; // convert to radians

        pdef.bounds["high"][0] = high_bounds[0];
        pdef.bounds["high"][1] = high_bounds[1];
        pdef.bounds["high"][2] = high_bounds[2];
        pdef.bounds["high"][3] = ((double)high_bounds[3]) * M_PI / 180.0; // convert to radians
        pdef.bounds["high"][4] = ((double)high_bounds[4]) * pdef.L;
        pdef.bounds["high"][5] = ((double)high_bounds[5]) * M_PI / 180.0; // convert to radians

        ros::param::get("/planning/start", pdef.start);
        ros::param::get("/planning/goal", pdef.goal);

        // converions of start and goal
        pdef.start["yaw"] *= M_PI / 180.0;
        pdef.start["drones_dist"] *= pdef.L;
        pdef.start["drones_angle"] *= M_PI / 180.0;

        pdef.goal["yaw"] *= M_PI / 180.0;
        pdef.goal["drones_dist"] *= pdef.L;
        pdef.goal["drones_angle"] *= M_PI / 180.0;

        printf("start drones_dist: %f , %f , %f\n", pdef.start["drones_dist"], pdef.bounds["low"][4], pdef.bounds["high"][4]);

        printf("Loaded start and goal\n");
        // check if start and goal are within bounds
        bool start_out_of_bounds = (pdef.start["x"] <= pdef.bounds["low"][0] || pdef.start["x"] >= pdef.bounds["high"][0]) ||
                                   (pdef.start["y"] <= pdef.bounds["low"][1] || pdef.start["y"] >= pdef.bounds["high"][1]) ||
                                   (pdef.start["z"] <= pdef.bounds["low"][2] || pdef.start["z"] >= pdef.bounds["high"][2]) ||
                                   (pdef.start["yaw"] <= pdef.bounds["low"][3] || pdef.start["yaw"] >= pdef.bounds["high"][3]) ||
                                   (pdef.start["drones_dist"] <= pdef.bounds["low"][4] || pdef.start["drones_dist"] >= pdef.bounds["high"][4]);
        // (pdef.start["drones_angle"] <= pdef.bounds["low"][5] || pdef.start["drones_angle"] >= pdef.bounds["high"][5]);

        if (start_out_of_bounds)
        {
            ROS_ERROR("Start is out of bounds");
            throw std::runtime_error("Start is out of bounds");
            exit(1);
        }

        bool goal_out_of_bounds = (pdef.goal["x"] <= pdef.bounds["low"][0] || pdef.goal["x"] >= pdef.bounds["high"][0]) ||
                                  (pdef.goal["y"] <= pdef.bounds["low"][1] || pdef.goal["y"] >= pdef.bounds["high"][1]) ||
                                  (pdef.goal["z"] <= pdef.bounds["low"][2] || pdef.goal["z"] >= pdef.bounds["high"][2]) ||
                                  (pdef.goal["yaw"] <= pdef.bounds["low"][3] || pdef.goal["yaw"] >= pdef.bounds["high"][3]) ||
                                  (pdef.goal["drones_dist"] <= pdef.bounds["low"][4] || pdef.goal["drones_dist"] >= pdef.bounds["high"][4]) ||
                                  (pdef.goal["drones_angle"] <= pdef.bounds["low"][5] || pdef.goal["drones_angle"] >= pdef.bounds["high"][5]);

        if (goal_out_of_bounds)
        {
            ROS_ERROR("Goal is out of bounds");
            throw std::runtime_error("Goal is out of bounds");
            exit(1);
        }

        printf("Out of bounds check ended\n");

        ros::param::get("/planning/planner_algorithm", pdef.planner_algorithm);

        ros::param::get("/planning/val_check_resolution", pdef.val_check_resolution);
        ros::param::get("/planning/range", pdef.range);

        ros::param::get("/planning/simplify_path", pdef.simplify_path);

        ros::param::get("/planning/path_interpolation_points", pdef.path_interpolation_points);

        // Select goal type
        std::map<std::string, int> goal_type_map = {{"simple", GoalType::SIMPLE},
                                                    {"symmetrical", GoalType::SYMMETRICAL},
                                                    {"samplable", GoalType::SAMPLABLE},
                                                    {"multiple_goals", GoalType::MULTIPLE_GOALS}

        };

        std::string goal_type_str;
        ros::param::get("/planning/goal_type", goal_type_str);
        pdef.goal_type = goal_type_map[goal_type_str];

        // Enable ground collision check
        ros::param::get("/planning/use_ground_collision_check", pdef.use_ground_collision_check);

        // safety distances
        ros::param::get("/planning/safety_distances/drones_offsets/horizontal", pdef.safety_offsets.drones_horizontal);
        ros::param::get("/planning/safety_distances/drones_offsets/vertical", pdef.safety_offsets.drones_vertical);

        ros::param::get("/planning/safety_distances/lowest_point", pdef.safety_offsets.lowest_point);

        ros::param::get("/planning/thickness", pdef.thickness);

        if (pdef.planning_type == PlanningType::MOVING_OBSTACLES)
        {
            pdef.obstacles_config = realtime_obstacles::load_cylinders_definition(nh);
        }

        ros::param::get("/planning/real_time_settings/setting_new_start", pdef.realtime_settings.setting_new_start);
        ros::param::get("/planning/real_time_settings/replan_only_if_not_valid", pdef.realtime_settings.replan_only_if_not_valid);
        ros::param::get("/planning/real_time_settings/fix_invalid_start_dist", pdef.realtime_settings.fix_invalid_start_dist);
        ros::param::get("/planning/real_time_settings/fix_invalid_goal_dist", pdef.realtime_settings.fix_invalid_goal_dist);
        ros::param::get("/planning/real_time_settings/distance_to_goal_resetting", pdef.realtime_settings.distance_to_goal_resetting);
        ros::param::get("/planning/real_time_settings/replanning_interval", pdef.realtime_settings.replanning_interval);

        std::map<std::string, int> simplifying_path_map = {
            {"full", SimplifyingPath::FULL}, {"fast", SimplifyingPath::FAST}, {"none", SimplifyingPath::NONE}};

        std::string simplifying_path_str;
        ros::param::get("/planning/real_time_settings/siplifying_path", simplifying_path_str);
        pdef.realtime_settings.simplifying_path = simplifying_path_map[simplifying_path_str];

        ros::param::get("/planning/real_time_settings/time_allocation/velocity", pdef.realtime_settings.time_alloc.velocity);
        ros::param::get("/planning/real_time_settings/time_allocation/acceleration", pdef.realtime_settings.time_alloc.acceleration);

        ros::param::get("/planning/V_robust/dx", pdef.v_robust.dx);
        ros::param::get("/planning/V_robust/dy", pdef.v_robust.dy);
        return pdef;
    }

    void printProblemParams(ProblemParams params)
    {
        printf("\n\n");
        printf("=================================== PROBLEM PARAMETERS: ===================================\n");
        printf("\tPlanning type: %d\n", params.planning_type);
        if (params.planning_type == PlanningType::MOVING_OBSTACLES)
        {
            printf("\t\tSetting new start:%d\n", params.realtime_settings.setting_new_start);
            printf("\t\tReplan if not valid:%d\n", params.realtime_settings.replan_only_if_not_valid);
            printf("\t\tFix invalid start dist:%d\n", params.realtime_settings.fix_invalid_start_dist);
            printf("\t\tFix invalid goal dist:%d\n", params.realtime_settings.fix_invalid_goal_dist);
            printf("\t\tDistance to goal resetting:%f\n", params.realtime_settings.distance_to_goal_resetting);
            printf("\t\tReplanning interval:%f\n", params.realtime_settings.replanning_interval);

            std::map<int, std::string> simplifying_path_map = {
                {SimplifyingPath::FULL, "full"}, {SimplifyingPath::FAST, "fast"}, {SimplifyingPath::NONE, "none"}};

            printf("\t\tSimplifying path:%s\n", simplifying_path_map[params.realtime_settings.simplifying_path]);

            printf("\t\tTime allocation:\n");
            printf("\t\t\tVelocity:%f\n", params.realtime_settings.time_alloc.velocity);
            printf("\t\t\tAcceleration:%f\n", params.realtime_settings.time_alloc.acceleration);
        }

        printf("\tTimeout: %f\n", params.timeout);
        printf("\tRope length: %f\n", params.L);
        printf("\tEnvironment filename: %s\n", params.env_filename.c_str());
        printf("\tStart state: %f, %f, %f, %f, %f, %f, \n", params.start["x"], params.start["y"], params.start["z"], params.start["yaw"],
               params.start["drones_dist"], params.start["drones_angle"]);

        printf("\tGoal state: %f, %f, %f, %f, %f, %f, \n", params.goal["x"], params.goal["y"], params.goal["z"], params.goal["yaw"],
               params.goal["drones_dist"], params.goal["drones_angle"]);

        printf("\tBounds:\n");
        printf("\t\tlow: %f, %f, %f, %f, %f, %f\n", params.bounds["low"][0], params.bounds["low"][1], params.bounds["low"][2],
               params.bounds["low"][3], params.bounds["low"][4], params.bounds["low"][5]);
        printf("\t\thigh: %f, %f, %f, %f, %f, %f\n", params.bounds["high"][0], params.bounds["high"][1], params.bounds["high"][2],
               params.bounds["high"][3], params.bounds["high"][4], params.bounds["high"][5]);

        printf("\tPlanner algorithm: %s\n", params.planner_algorithm.c_str());
        printf("\tValidity check resolution: %f\n", params.val_check_resolution);
        printf("\tRange: %f\n", params.range);

        printf("\tSimplify path: %d\n", params.simplify_path);
        printf("\tPath interpolation points: %d\n", params.path_interpolation_points);

        // print goal type
        switch (params.goal_type)
        {
        case GoalType::SIMPLE:
            printf("\tGoal type: simple\n");
            break;
        case GoalType::SYMMETRICAL:
            printf("\tGoal type: symmetrical\n");
            break;
        case GoalType::SAMPLABLE:
            printf("\tGoal type: samplable\n");
            break;
        case GoalType::MULTIPLE_GOALS:
            printf("\tGoal type: multiple goals\n");
            break;
        default:
            printf("\tGoal type: unknown\n");
            break;
        }

        printf("\tUse ground collision check: %d\n", params.use_ground_collision_check);

        printf("\n\tSafety distances:\n");
        printf("\t\tDrones:\n");

        printf("\t\t\tHorizontal: %f\n", params.safety_offsets.drones_horizontal);
        printf("\t\t\tVertical: %f\n", params.safety_offsets.drones_vertical);

        printf("\t\tLowest point: %f\n", params.safety_offsets.lowest_point);

        printf("\t\t V robust steps:\n");
        printf("\t\t\tHorizontal: %f\n", params.v_robust.dx);
        printf("\t\t\tVertical: %f\n", params.v_robust.dy);

        printf("\tThickness: %f\n", params.thickness);

        printf("\tCYLINDER OBSTACLES CONFIG\n");

        if (params.planning_type == PlanningType::MOVING_OBSTACLES)
        {
            for (int i = 0; i < params.obstacles_config.size(); i++)
            {
                auto c = params.obstacles_config[i];
                printf("\t\tradius: %f height: %f pos : %f %f %f \n", c.radius, c.height, c.pos[0], c.pos[1], c.pos[2]);
            }
            printf("===========================================================================================\n");

            printf("\n\n");
        }
    }

}; // namespace problem_params
