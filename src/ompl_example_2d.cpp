#include "../include/ompl_example_2d/ompl_example_2d.hpp"

//  █▀▄▀█ ▄▀█ █▀█ █ █▀█ █▀   █▀ ▀█▀ ▄▀█ █▀▄▀█ ▄▀█ ▀█▀ █▀█ █▀█ █▀█ █░█ █░░ █▀█ █
//  █░▀░█ █▀█ █▀▄ █ █▄█ ▄█   ▄█ ░█░ █▀█ █░▀░█ █▀█ ░█░ █▄█ █▀▀ █▄█ █▄█ █▄▄ █▄█ ▄█

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
            // load environment here

            printf("Planning offline with static obstacles\n");
            checker = new fcl_checking_offline::checker();

            try
            {
                std::string env_mesh = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/" + prob_params.env_filename + ".stl";
                checker->as<fcl_checking_offline::checker>()->loadEnvironment(env_mesh);
            }
            catch (std::exception &e)
            {
                // moving obstacles
                std::cout << "Using environment mesh from drones_path_planning" << std::endl;
                std::string env_mesh = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/" + prob_params.env_filename + ".stl";
                checker->as<fcl_checking_offline::checker>()->loadEnvironment(env_mesh);
            }
            printf("Loaded stl of environment!\n");
        }
        else
        {
            throw std::runtime_error("Planning type not supported");
        }

        L = prob_params.L;

#ifdef USE_CUSTOM_MESH_ROBUST
        custom_robot_mesh = new custom_mesh_robust::CustomMesh(L, prob_params.safety_offsets, prob_params.thickness, prob_params.v_robust);
#else
        custom_robot_mesh = new custom_mesh::CustomMesh(L, prob_params.safety_offsets, prob_params.thickness);
#endif

        printf("Instanciated custom mesh!\n");
        try
        {
            printf("Try to update robot mesh\n");
            // checker->update_robot(custom_robot_mesh->get_fcl_mesh());
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\n", e.what());
            std::cerr << e.what() << '\n';
            int x;
            std::cin >> x;
        }

        printf("Updated robot mesh!\n");

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

        // printf("Setting optimization objective...\n");
        // pdef->setOptimizationObjective(getBalancedObjective(si, checker, custom_robot_mesh));

        // set own heuristic
        // auto obj = custom_objectives::custom_heuristic(si);
        // pdef->setOptimizationObjective(obj);

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
        start_state_.resize(dim);
        goal_state_.resize(dim);

        for (int i = 0; i < dim; i++)
        {
            start_state_[i] = start[i];
            goal_state_[i] = goal[i];
        }

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

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRTConnect")
        {
            auto plan = std::make_shared<og::RRTConnect>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "LBKPIECE")
        {
            auto plan = std::make_shared<og::LBKPIECE1>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "KPIECE")
        {
            auto plan = std::make_shared<og::KPIECE1>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "BKPIECE")
        {
            auto plan = std::make_shared<og::BKPIECE1>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRTstar")
        {
            auto plan = std::make_shared<og::RRTstar>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);
            return plan;
        }
        else if (planner_name == "RRT")
        {
            auto plan = std::make_shared<og::RRT>(si);

            // printf("Setting  range...\n");
            plan->setRange(range);

            return plan;
        }
        else
        {
            throw std::runtime_error("Unknown planner");
        }
    }

    bool planner::simplifyPath(og::PathGeometric *pth)
    {
        if (!prob_params.simplify_path) // Maybe try to simplify either way?
        {
            return true;
        }

        auto t0 = ros::Time::now();
        og::PathSimplifier path_simplifier(si, pdef->getGoal());

        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

        if (prob_params.realtime_settings.simplifying_path == problem_params::SimplifyingPath::FAST)
        {
            bool changes_made = false;

            changes_made = changes_made || path_simplifier.shortcutPath(*pth);
            changes_made = changes_made || path_simplifier.reduceVertices(*pth);
            changes_made = changes_made || path_simplifier.collapseCloseVertices(*pth);
            changes_made = changes_made || path_simplifier.reduceVertices(*pth);
            // if (changes_made)
            // {
            //     bool path_valid = pth->check();
            //     return path_valid;
            // }
            // else
            // {
            //     return true;
            // }

            return pth->check();
        }
        else if (prob_params.realtime_settings.simplifying_path == problem_params::SimplifyingPath::FULL)
        {
            return path_simplifier.simplify(*pth, ptc, false);
        }
        else
        {
            // no simplyfing
            return true;
        }

        // path_simplifier.simplifyMax(*pth);
    }

    void planner::print_settings()
    {
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
    }

    void planner::save_path(og::PathGeometric *pth)
    {
        // save path to file
        std::ofstream myfile;
        myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
        pth->printAsMatrix(myfile);
        myfile.close();
    }

    void planner::interpolate_path(og::PathGeometric *pth)
    {
        if (prob_params.path_interpolation_points > 0)
        {
            // std::cout << "Interpolating path...\n";
            // og::PathGeometric::InterpolationType interp_type = og::PathGeometric::CSPLINE;
            // pth->interpolate(interp_type);
            pth->interpolate(prob_params.path_interpolation_points);
        }
    }

    og::PathGeometric *planner::plan()
    {
        printf("Calculating plan with start: %f %f %f\n", pdef->getStartState(0)->as<ob::RealVectorStateSpace::StateType>()->values[0],
               pdef->getStartState(0)->as<ob::RealVectorStateSpace::StateType>()->values[1],
               pdef->getStartState(0)->as<ob::RealVectorStateSpace::StateType>()->values[2]);

        // STATS
        const int times = 5;
        static float sum_times[times] = {0, 0, 0, 0, 0};
        static float max_times[times] = {0, 0, 0, 0, 0};
        static int times_called = 0;

        float dts_to_msec[times] = {0, 0, 0, 0, 0};

        auto t0 = ros::Time::now();

        auto t4 = ros::Time::now();
        static int times_fixed = 0;

        printf("Fixinng invalid states...\n");
        // run it ony for a couple of times beacuse it crhashes otherwise TODO: fix it
        if (times_fixed < 10)
        {
            printf("times_fixed: %d\n", times_fixed);
            bool res = pdef->fixInvalidInputStates(prob_params.realtime_settings.fix_invalid_start_dist,
                                                   prob_params.realtime_settings.fix_invalid_goal_dist, 20);
            if (res)
                times_fixed++;
        }

        // // check if start is valid
        // if (!si->isValid(pdef->getStartState(0)))
        // {
        //     throw std::runtime_error("Start state is invalid while checking if start is valid");
        // }

        pdef->clearSolutionPaths();

        printf("Getting planner \n");
        planner_ = getPlanner(prob_params.planner_algorithm, prob_params.range);

        // printf("Setting  problem definition...\n");
        // planner_->setProblemDefinition(pdef);
        //
        // printf("Setting  planner up...\n");
        // planner_->setup();

        // check if having a start
        if (pdef->getStartStateCount() == 0 || pdef->getGoal() == nullptr)
        {
            throw std::runtime_error("No start or goal state defined");
        }

        print_settings();

        // planner_->getPlannerInputStates()

        times_called++;

        auto dt4 = ros::Time::now() - t4;
        dts_to_msec[4] = dt4.toSec() * 1000;
        sum_times[4] += dts_to_msec[4];
        max_times[4] = std::max(max_times[4], dts_to_msec[4]);

        // attempt to solve the problem within one second of planning time
        auto t1 = ros::Time::now();

        printf("Checking if start is valid before planning...\n");
        // check if start is valid
        for (int i = 0; i < pdef->getStartStateCount(); i++)
        {
            if (!si->satisfiesBounds(pdef->getStartState(i)))
            {
                ROS_INFO("Start state with index: %d is not within bounds", i);

                // print state
                std::cout << "Start state: " << std::endl;
                for (int j = 0; j < 6; j++)
                {
                    std::cout << pdef->getStartState(i)->as<ob::RealVectorStateSpace::StateType>()->values[j] << " ";
                }
                std::cout << std::endl;

                // fix invalid start
                fixInvalidStartBounds();
                // planner_->clear();
                // planner_->setProblemDefinition(pdef);
                // planner_->setup();

                std::cout << "Start state: " << std::endl;
                for (int j = 0; j < 6; j++)
                {
                    std::cout << pdef->getStartState(i)->as<ob::RealVectorStateSpace::StateType>()->values[j] << " ";
                }
                std::cout << std::endl;
            }

            if (!si->isValid(pdef->getStartState(i)))
            {
                throw std::runtime_error("Start state with index: " + std::to_string(i) + " is invalid");
            }
        }

        // check if goal is valid
        printf("Checking if goal is valid before planning...\n");
        // if (!si->satisfiesBounds(pdef->getGoal()->as<ob::GoalState>()->getState()))

        fixInvalidGoalBounds();

        // Set problem definition and setup after fixing invalid start and goal
        planner_->setProblemDefinition(pdef);
        planner_->setup();
        printf("Problem definition set\n");

        printf("Solving...\n");
        ob::PlannerStatus solved;
        auto tt0 = ros::Time::now();
        do
        {
            // If planner is not reset,it is allowed  to continue work for more time on an unsolved problem
            // otherwise it will be reset and start from scratch
            //  plan->clear();

            // create termination condition
            ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));
            // printf("Solving!\n");
            solved = planner_->solve(ptc);

            // time check
            auto dt1 = ros::Time::now() - t1;
            dts_to_msec[1] = dt1.toSec() * 1000;
            if (dts_to_msec[1] > 60 * 1000)
                throw std::runtime_error("Planning time is too long");

            int x;
            if (solved == ob::PlannerStatus::INVALID_START)
            {
                throw std::runtime_error("Tried sloving and Start state is invalid");
            }
            else if (solved == ob::PlannerStatus::INVALID_GOAL)
            {
                throw std::runtime_error("Tried sloving and Goal state is invalid");
            }

            if (prob_params.planning_type == problem_params::PlanningType::MOVING_OBSTACLES)
            {
                auto dt2 = ros::Time::now() - tt0;
                auto dt2_msec = dt2.toSec() * 1000;
                if (dt2_msec > 2 * 1000)
                {
                    throw std::runtime_error("Planning time is too long");
                }
            }

        } while (solved != ob::PlannerStatus::EXACT_SOLUTION);

        auto dt1 = ros::Time::now() - t1;

        dts_to_msec[1] = dt1.toSec() * 1000;
        printf("Solved in %f ms\n", dts_to_msec[1]);

        sum_times[1] += dts_to_msec[1];
        max_times[1] = std::max(max_times[1], dts_to_msec[1]);

        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            // std::cout << "Found exact solution :" << std::endl;

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

            // Simplifying
            auto t2 = ros::Time::now();
            simplifyPath(pth);
            auto dt2 = ros::Time::now() - t2;

            dts_to_msec[2] = dt2.toSec() * 1000;
            sum_times[2] += dts_to_msec[2];
            max_times[2] = std::max(max_times[2], dts_to_msec[2]);

            // Interpolating
            auto t3 = ros::Time::now();
            interpolate_path(pth);
            auto dt3 = ros::Time::now() - t3;
            dts_to_msec[3] = dt3.toSec() * 1000;
            sum_times[3] += dts_to_msec[3];
            max_times[3] = std::max(max_times[3], dts_to_msec[3]);

            // pth->printAsMatrix(std::cout);

            auto dt0 = ros::Time::now() - t0;
            dts_to_msec[0] = dt0.toSec() * 1000;
            sum_times[0] += dts_to_msec[0];
            max_times[0] = std::max(max_times[0], dts_to_msec[0]);

            bool print_times = false;
            if (print_times)
            {
                printf("\t-plan->solve()  time->	 Current: %4f \tAverage : %4f msec \t max:%4f\n ", dts_to_msec[0], sum_times[0] / times_called,
                       max_times[0]);
                printf("\t\t-Initialization time->  Current: %4f \tAverage : %4f msec \t max:%4f\n ", dts_to_msec[4], sum_times[4] / times_called,
                       max_times[4]);
                printf("\t\t-planner->solve time->  Current: %4f \tAverage : %4f msec \t max:%4f\n ", dts_to_msec[1], sum_times[1] / times_called,
                       max_times[1]);
                printf("\t\t-Simplifying    time->  Current: %4f \tAverage : %4f msec \t max:%4f\n ", dts_to_msec[2], sum_times[2] / times_called,
                       max_times[2]);
                printf("\t\t-Interpolate    time->  Current: %4f \tAverage : %4f msec \t max:%4f\n ", dts_to_msec[3], sum_times[3] / times_called,
                       max_times[3]);
            }

            if (prob_params.planning_type == problem_params::PlanningType::STATIC)
                save_path(pth);

            path_ = pth;

            return pth;
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

        // update dynamic robot mesh
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
            // std::cout << "\r"; // print at the same line (update effect)

            std::cout << "State Validation: " << counter << " calls " << total_time / 1000 << " secs, " << total_time / counter << " msecs/call, "
                      << counter * 1000.0 / total_time << " calls/sec" << std::flush;
        }

        return result;
    }

    bool planner::isStateValidSimple(std::vector<float> state)
    {
        ob::ScopedState<> start_state(planner::space);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = state[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = state[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = state[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = state[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = state[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = state[5];

        return isStateValid(start_state->as<ob::State>());
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

    void planner::convert_path_to_drones_paths(og::PathGeometric *pth, nav_msgs::Path &drone_pth1, nav_msgs::Path &drone_pth2)
    { // frames
        const std::string rb_path_frame_name = "rb_path";
        const std::string world_frame_name = "world";

        auto states = pth->getStates();
        int num_states = states.size();

        // cretae path for drone1
        drone_pth1.header.frame_id = world_frame_name;
        drone_pth1.header.stamp = ros::Time::now();
        drone_pth1.poses.resize(num_states);

        // create poseStamped for drone1
        geometry_msgs::PoseStamped drone_pose1;
        geometry_msgs::PoseStamped transformed_drone_pose1;
        drone_pose1.header.frame_id = rb_path_frame_name;
        drone_pose1.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        // cretae path for drone2
        drone_pth2.header.frame_id = world_frame_name;
        drone_pth2.header.stamp = ros::Time::now();
        drone_pth2.poses.resize(num_states);

        // create poseStamped for drone2
        geometry_msgs::PoseStamped drone_pose2;
        geometry_msgs::PoseStamped transformed_drone_pose2;
        drone_pose2.header.frame_id = rb_path_frame_name;
        drone_pose2.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        // tf2_ros::Buffer tf_buffer;
        // tf2_ros::TransformListener tf2_listener(tf_buffer);

        geometry_msgs::TransformStamped ts2;

        ts2.header.stamp = ros::Time(0);
        ts2.header.frame_id = world_frame_name;
        ts2.child_frame_id = rb_path_frame_name;

        tf2::Quaternion q;

        for (auto i = 0; i < num_states; i++)
        {

            auto state_values = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;

            double yaw = state_values[3];
            double drones_dis = state_values[4];
            double drones_angle = state_values[5];

            ts2.transform.translation.x = state_values[0];
            ts2.transform.translation.y = state_values[1];
            ts2.transform.translation.z = state_values[2];

            q.setRPY(0, 0, yaw);
            ts2.transform.rotation = tf2::toMsg(q);

            // drones formation to drones positions
            //  get the distance between the 2 drones
            float r = drones_dis / 2;

            // get the points of the triangle
            auto p0 = Eigen::Vector2f(r * cos(drones_angle), r * sin(drones_angle));
            auto p1 = Eigen::Vector2f(-r * cos(drones_angle), -r * sin(drones_angle));

            // drone 1
            drone_pose1.pose.position.x = p0[0];
            drone_pose1.pose.position.y = 0;
            drone_pose1.pose.position.z = p0[1];

            tf2::doTransform(drone_pose1, transformed_drone_pose1, ts2);

            drone_pth1.poses[i] = transformed_drone_pose1;

            // drone2
            drone_pose2.pose.position.x = p1[0];
            drone_pose2.pose.position.y = 0;
            drone_pose2.pose.position.z = p1[1];
            tf2::doTransform(drone_pose2, transformed_drone_pose2, ts2);
            drone_pth2.poses[i] = transformed_drone_pose2;
        }

        // printf("\t-RB to drones   time->\t Current: %4.2f msec \n", dt0.toSec() * 1000);
        // printf("\t\tInitializing the  path took: %4.2f msec\n", dt1.toSec() * 1000);
        // printf("\t\tTransfrom rb_path to drones: %4.2f msec\n", dt2.toSec() * 1000);
    }

    ompl::base::StateSpacePtr planner::getSpace() { return space; }

    std::vector<float> planner::getStartState() { return start_state_; }

    std::vector<float> planner::getGoalState() { return goal_state_; }

    void planner::setStart(float start[6])
    {
        for (int i = 0; i < dim; i++)
        {
            start_state_[i] = start[i];
        }

        ob::ScopedState<> start_state(planner::space);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        pdef->clearStartStates();
        pdef->addStartState(start_state);
    }

    void planner::setGoal(float goal[6])
    {
        for (int i = 0; i < dim; i++)
        {
            goal_state_[i] = goal[i];
        }

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

        pdef->clearGoal();
        pdef->setGoal(ob::GoalPtr(goalStates));
    }

    ompl::geometric::PathGeometric *planner::getPath() { return path_; }

    void planner::setPath(ompl::geometric::PathGeometric *path) { path_ = new ompl::geometric::PathGeometric(*path); }

    ompl::base::SpaceInformationPtr planner::getSpaceInformation() { return si; }

    void planner::fixInvalidStartBounds()
    {
        // limit values in the range of the limits
        for (int i = 0; i < 6; i++)
        {
            std::cout << "State value: " << start_state_[i] << " Bounds:" << prob_params.bounds["low"][i] << " " << prob_params.bounds["high"][i]
                      << std::endl;

            if (start_state_[i] <= prob_params.bounds["low"][i])
                start_state_[i] = prob_params.bounds["low"][i] + 0.01; // small offset in order to be inside the bounds

            else if (start_state_[i] >= prob_params.bounds["high"][i])
                start_state_[i] = prob_params.bounds["high"][i] - 0.01; // small offset in order to be inside the bounds
        }

        // set the start state
        setStart(start_state_.data());
    }

    void planner::fixInvalidGoalBounds()
    {
        // limit values in the range of the limits
        for (int i = 0; i < 6; i++)
        {
            std::cout << "State value: " << goal_state_[i] << " Bounds:" << prob_params.bounds["low"][i] << " " << prob_params.bounds["high"][i]
                      << std::endl;

            if (goal_state_[i] <= prob_params.bounds["low"][i])
            {
                goal_state_[i] = prob_params.bounds["low"][i] + 0.01; // small offset in order to be inside the bounds
                printf("Goal state %d is out of bounds, setting to %f\n", i, goal_state_[i]);
            }
            else if (goal_state_[i] >= prob_params.bounds["high"][i])
            {
                goal_state_[i] = prob_params.bounds["high"][i] - 0.01; // small offset in order to be inside the bounds
                printf("Goal state %d is out of bounds, setting to %f\n", i, goal_state_[i]);
            }

            std::cout << "State value: " << goal_state_[i] << " Bounds:" << prob_params.bounds["low"][i] << " " << prob_params.bounds["high"][i]
                      << std::endl;
        }

        // set the goal state
        setGoal(goal_state_.data());
    }

} // namespace drones_rope_planning