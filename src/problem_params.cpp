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
        ros::param::get("/planning/start", pdef.start_pos);
        ros::param::get("/planning/goal", pdef.goal_pos);

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

        pdef.obstacles_config = realtime_obstacles::load_cylinders_definition(nh);

        ros::param::get("/planning/setting_new_start", pdef.setting_new_start);

        return pdef;
    }

    void printProblemParams(ProblemParams params)
    {
        printf("\n\n");
        printf("=================================== PROBLEM PARAMETERS: ===================================\n");
        printf("\tPlanning type: %d\n", params.planning_type);
        if (params.planning_type == PlanningType::MOVING_OBSTACLES)
        {
            printf("\t\tSetting new start:%d\n", params.setting_new_start);
        }

        printf("\tTimeout: %f\n", params.timeout);
        printf("\tRope length: %f\n", params.L);
        printf("\tEnvironment filename: %s\n", params.env_filename.c_str());
        printf("\tStart position: %f, %f, %f, \n", params.start_pos["x"], params.start_pos["y"], params.start_pos["z"]);
        printf("\tGoal position: %f, %f, %f, \n", params.goal_pos["x"], params.goal_pos["y"], params.goal_pos["z"]);
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

        printf("\tThickness: %f\n", params.thickness);

        printf("\tCYLINDER OBSTACLES CONFIG\n");

        for (int i = 0; i < params.obstacles_config.size(); i++)
        {
            auto c = params.obstacles_config[i];
            printf("\t\tradius: %f height: %f pos : %f %f %f \n", c.radius, c.height, c.pos[0], c.pos[1], c.pos[2]);
        }
        printf("===========================================================================================\n");

        printf("\n\n");
    }
}; // namespace problem_params
