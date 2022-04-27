#include "../include/ompl_example_2d/ompl_example_2d.hpp"

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_rope_planning
{
    typedef std::map<std::string, ob::PlannerPtr> planners_map;

    planner::planner(problem_params::ProblemParams prob_prms)
    {

        prob_params = prob_prms;
        problem_params::printProblemParams(prob_params);

        if (prob_prms.planning_type == problem_params::PlanningType::MOVING_OBSTACLES)
        {
            printf("Planning with moving obstacles\n");
            checker = new fcl_checking_realtime::checker();

            printf("Loading Environment\n");
            auto cyls_conf = prob_params.obstacles_config;
            checker->as<fcl_checking_realtime::checker>()->loadEnvironment(cyls_conf);
            checker->as<fcl_checking_realtime::checker>()->updateEnvironmentTransforms(cyls_conf);
        }
        else if (prob_prms.planning_type == problem_params::PlanningType::STATIC)
        {
            // TODO: make fcl_checker inherit from fcl_checker_base
            // and load environment here

            // try
            // {
            //     std::string env_mesh = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/" + prob_params.env_filename + ".stl";
            //     checker.loadEnvironment(env_mesh);
            // }
            // catch (std::exception &e)
            // {
            //     // moving obstacles
            //     std::cout << "Using environment mesh from drones_path_planning" << std::endl;
            //     std::string env_mesh = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/" + prob_params.env_filename + ".stl";
            //     checker.loadEnvironment(env_mesh);
            // }
        }
        else
        {
            throw std::runtime_error("Planning type not supported");
        }

        L = prob_params.L;

        custom_robot_mesh = new custom_mesh::CustomMesh(L, prob_params.safety_offsets, prob_params.thickness);
        checker->update_robot(custom_robot_mesh->get_fcl_mesh());

        dim = 6;
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(dim));

        printf("Setting bounds\n");
        setBounds();

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        printf("Setting validity checker...\n");
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));
        si->setStateValidityCheckingResolution(prob_params.val_check_resolution);

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set Optimizattion objective
        // ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        // pdef->setOptimizationObjective(obj);

        // ob::OptimizationObjectivePtr obj(new custom_objectives::RopeRelaxedObjective(si, prob_params.L));
        // pdef->setOptimizationObjective(obj);

        // set own heuristic
        auto obj = custom_objectives::custom_heuristic(si);
        pdef->setOptimizationObjective(obj);

        std::cout << "Initialized: " << std::endl;
    }

    // Destructor
    planner::~planner() {}

    void planner::setBounds()
    {
        ob::RealVectorBounds bounds(dim);

        float drones_angle = M_PI / 3;

        // Lower bounds
        double lower[6] = {prob_params.bounds["low"][0], prob_params.bounds["low"][1], prob_params.bounds["low"][2],
                           prob_params.bounds["low"][3], prob_params.bounds["low"][4], prob_params.bounds["low"][5]};

        double upper[6] = {prob_params.bounds["high"][0], prob_params.bounds["high"][1], prob_params.bounds["high"][2],
                           prob_params.bounds["high"][3], prob_params.bounds["high"][4], prob_params.bounds["high"][5]};

        bounds.setLow(0, lower[0]);
        bounds.setLow(1, lower[1]);
        bounds.setLow(2, lower[2]);
        bounds.setLow(3, lower[3]);
        bounds.setLow(4, lower[4]);
        bounds.setLow(5, lower[5]);

        // Upper bounds
        bounds.setHigh(0, upper[0]);
        bounds.setHigh(1, upper[1]);
        bounds.setHigh(2, upper[2]);
        bounds.setHigh(3, upper[3]);
        bounds.setHigh(4, upper[4]);
        bounds.setHigh(5, upper[5]);

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    }

    void planner::setStartGoal(float start[6], float goal[6])
    {
        ob::ScopedState<> start_state(planner::space);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        pdef->addStartState(start_state);

        switch (prob_params.goal_type)
        {
        case problem_params::GoalType::SIMPLE:
        {
            printf("Simple goal\n");
            // Add goal states
            ob::GoalStates *goalStates(new ob::GoalStates(si));

            ob::ScopedState<> goal_state(planner::space);
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            goalStates->addState(goal_state);

            pdef->setGoal(ob::GoalPtr(goalStates));

            break;
        }
        case problem_params::GoalType::SYMMETRICAL:
        {
            printf("Symmetrical goal\n");
            auto goal_region = std::make_shared<SymmetricalGoal>(si, goal, 0.5);
            pdef->setGoal(ob::GoalPtr(goal_region));

            float start_distance = 0.1;
            float goal_distance = 0.5;
            unsigned int max_attempts = 100;
            pdef->fixInvalidInputStates(start_distance, goal_distance, max_attempts);
            break;
        }
        case problem_params::GoalType::SAMPLABLE:
        {
            printf("Samplable goal\n");
            auto samplable_goal = std::make_shared<CustomSamplabeGoal>(si, goal, 0.5);
            pdef->setGoal(ob::GoalPtr(samplable_goal));

            float start_distance = 0.1;
            float goal_distance = 0.5;
            unsigned int max_attempts = 100;
            pdef->fixInvalidInputStates(start_distance, goal_distance, max_attempts);
            break;
        }
        case problem_params::GoalType::MULTIPLE_GOALS:
        {
            printf("Multiple goals\n");
            // Add goal states
            ob::GoalStates *goalStates(new ob::GoalStates(si));

            ob::ScopedState<> s(si);
            s->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            s->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            s->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            s->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
            s->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            s->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            goalStates->addState(s);
            pdef->setGoal(ob::GoalPtr(goalStates));
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown goal type");
        }
        }
    }

    template <typename T> ob::Planner *createInstance(ob::SpaceInformationPtr si) { return std::make_shared<T>(si); }

    typedef std::map<std::string, ob::PlannerPtr (*)()> map_type;

    ob::PlannerPtr planner::getPlanner(std::string planner_name, float range)
    {
        if (planner_name == "PRM")
        {
            auto plan = std::make_shared<og::PRM>(si);
            return plan;
        }
        else if (planner_name == "InformedRRTstar")
        {
            auto plan = std::make_shared<og::InformedRRTstar>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRTConnect")
        {
            auto plan = std::make_shared<og::RRTConnect>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "LBKPIECE")
        {
            auto plan = std::make_shared<og::LBKPIECE1>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "KPIECE")
        {
            auto plan = std::make_shared<og::KPIECE1>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "BKPIECE")
        {
            auto plan = std::make_shared<og::BKPIECE1>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRTstar")
        {
            auto plan = std::make_shared<og::RRTstar>(si);

            printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRT")
        {
            auto plan = std::make_shared<og::RRT>(si);

            printf("Setting  range...\n");
            plan->setRange(range);

            return plan;
        }
        else
        {
            throw std::runtime_error("Unknown planner");
        }
    }

    drones_rope_planning::rigid_body_dynamic_path planner::plan()
    {
        pdef->clearSolutionPaths();

        planner_ = getPlanner(prob_params.planner_algorithm, prob_params.range);

        // set the problem we are trying to solve for the planner
        // printf("Setting  problem definition...\n");
        planner_->setProblemDefinition(pdef);

        // perform setup steps for the planner
        // printf("Setting  planner up...\n");
        planner_->setup();

        if (prob_params.planning_type != problem_params::PlanningType::MOVING_OBSTACLES)
        {
            // print the settings for this space
            printf("============================ OMPL SPACE SETTINGS: ============================\n");
            si->printSettings(std::cout);

            // print the problem settings
            printf("============================ OMPL PROBLEM SETTINGS: ============================\n");
            pdef->print(std::cout);
            printf("================================================================================\n");
        }

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved;
        do
        {
            // If planner is not reset
            // it is allowed  to continue work for more time on an unsolved problem
            // otherwise it will be reset and start from scratch

            //  plan->clear();

            // create termination condition
            ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

            auto t0 = std::chrono::high_resolution_clock::now();
            solved = planner_->solve(ptc);
            std::cout << std::endl;
            auto dt = std::chrono::high_resolution_clock::now() - t0;

            std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;

        } while (solved != ob::PlannerStatus::EXACT_SOLUTION);

        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            // std::cout << "Found exact solution :" << std::endl;

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

            if (prob_params.simplify_path)
            {
                auto t0 = std::chrono::high_resolution_clock::now();
                // std::cout << "Simplifying path...\n";
                og::PathSimplifier path_simplifier(si, pdef->getGoal());

                ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

                // path_simplifier.simplify(*pth, ptc, true);
                path_simplifier.collapseCloseVertices(*pth);

                // path_simplifier.simplifyMax(*pth);
                auto dt = std::chrono::high_resolution_clock::now() - t0;
                std::cout << "Simplifying time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;
            }

            if (prob_params.path_interpolation_points > 0)
            {
                // std::cout << "Interpolating path...\n";
                // og::PathGeometric::InterpolationType interp_type = og::PathGeometric::CSPLINE;
                // pth->interpolate(interp_type);
                auto t0 = std::chrono::high_resolution_clock::now();
                pth->interpolate(prob_params.path_interpolation_points);
                auto dt = std::chrono::high_resolution_clock::now() - t0;
                std::cout << "Interpolating time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;
            }

            // std::cout << "Final path :" << std::endl;
            // pth->printAsMatrix(std::cout);

            // convert path to rigid_body_dynamic_path.msg to be published
            auto dyn_path_msg = convert_path_to_msg(pth);

            return dyn_path_msg;
            // // save path to file
            // auto t0 = std::chrono::high_resolution_clock::now();
            // std::ofstream myfile;
            // myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
            // pth->printAsMatrix(myfile);
            // myfile.close();
            // auto dt = std::chrono::high_resolution_clock::now() - t0;
            // std::cout << "Saving path to file time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;
        }
        else
        {
            std::cout << "No solution found" << std::endl;
        }
    }

    void planner::replan() {}

    std::ostream &operator<<(std::ostream &os, const fcl::BVHBuildState &obj)
    {
        os << static_cast<std::underlying_type<fcl::BVHBuildState>::type>(obj);
        return os;
    }

    bool planner::isStateValid(const ob::State *state_check)
    {
        static int counter = 0;
        static float total_time = 0;
        const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

        float pos[3] = {state->values[0], state->values[1], state->values[2]};
        const auto yaw = state->values[3];
        const float drones_dis = state->values[4];
        const float drones_angle = state->values[5];

        auto t0 = ros::Time::now();

        // update dynamic robot meshh
        custom_robot_mesh->update_mesh(drones_dis, drones_angle);

        checker->update_robot(custom_robot_mesh->get_fcl_mesh());

        // ground collision check
        if (prob_params.use_ground_collision_check)
        {
            float z_offset = custom_robot_mesh->get_lowest_z();
            float distance_from_ground = pos[2] - z_offset;
            bool ground_collision = distance_from_ground <= prob_params.safety_offsets.lowest_point;

            if (ground_collision)
            {
                return false;
            }
        }

        //  apply yaw rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q = q.normalize();

        float quat[4] = {q.x(), q.y(), q.z(), q.w()};
        checker->setRobotTransform(pos, quat);

        // check colllision
        bool result = !checker->check_collision();

        // time measurements
        auto dt = ros::Time::now() - t0;
        total_time += dt.toSec() * 1000; // sum msecs
        counter++;

        if ((counter % 2000) == 0)
        {
            std::cout << "\r"; // print at the same line (update effect)

            std::cout << "State Validation: " << counter << " calls " << total_time / 1000 << " secs, " << total_time / counter << " msecs/call, "
                      << counter * 1000.0 / total_time << " calls/sec" << std::flush;
        }

        return result;
    }

    drones_rope_planning::rigid_body_dynamic_path planner::convert_path_to_msg(og::PathGeometric *pth)
    {
        drones_rope_planning::rigid_body_dynamic_path path_msg;

        auto states = pth->getStates();
        auto num_states = states.size();

        // setting up Path message
        path_msg.Path.header.frame_id = "world";
        path_msg.Path.header.stamp = ros::Time::now();
        path_msg.Path.poses.resize(num_states);

        // setting up vectors for drones distance and angle
        path_msg.drones_distances.resize(num_states);
        path_msg.drones_angles.resize(num_states);

        // maybe need to put this in the loop (dont know if it is passed by reference)
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "world";

        for (auto i = 0; i < num_states; i++)
        {

            auto state = states[i];
            auto state_values = state->as<ob::RealVectorStateSpace::StateType>()->values;

            double yaw = state_values[3];
            double drones_dis = state_values[4];
            double state_angle = state_values[5];

            // pos
            pose_msg.pose.position.x = state_values[0];
            pose_msg.pose.position.y = state_values[1];
            pose_msg.pose.position.z = state_values[2];

            // orientation
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose_msg.pose.orientation = tf2::toMsg(q);

            path_msg.Path.poses[i] = pose_msg;

            // drones distance
            path_msg.drones_distances[i] = drones_dis;

            // drones angle
            path_msg.drones_angles[i] = state_angle;
        }

        return path_msg;
    }

}
