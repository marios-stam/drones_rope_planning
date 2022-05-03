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

    og::PathGeometric *planner::plan()
    {
        printf("Calculating plan...\n");
        // STATS
        const int times = 5;
        static float sum_times[times] = {0, 0, 0, 0, 0};
        static float max_times[times] = {0, 0, 0, 0, 0};
        static int times_called = 0;

        float dts_to_msec[times] = {0, 0, 0, 0, 0};

        auto t0 = ros::Time::now();

        auto t4 = ros::Time::now();
        pdef->fixInvalidInputStates(prob_params.realtime_settings.fix_invalid_start_dist, prob_params.realtime_settings.fix_invalid_goal_dist, 20);
        // check if start is valid
        if (!si->isValid(pdef->getStartState(0)))
        {
            throw std::runtime_error("Start state is invalid");
        }

        pdef->clearSolutionPaths();

        planner_ = getPlanner(prob_params.planner_algorithm, prob_params.range);

        // set the problem we are trying to solve for the planner
        // printf("Setting  problem definition...\n");
        planner_->setProblemDefinition(pdef);

        // perform setup steps for the planner
        // printf("Setting  planner up...\n");
        planner_->setup();

        // check if having a start
        if (pdef->getStartStateCount() == 0 || pdef->getGoal() == nullptr)
        {
            throw std::runtime_error("No start or goal state defined");
        }

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

        // planner_->getPlannerInputStates()

        times_called++;

        auto dt4 = ros::Time::now() - t4;
        dts_to_msec[4] = dt4.toSec() * 1000;
        sum_times[4] += dts_to_msec[4];
        max_times[4] = std::max(max_times[4], dts_to_msec[4]);

        // attempt to solve the problem within one second of planning time
        auto t1 = ros::Time::now();

        ob::PlannerStatus solved;
        do
        {
            // If planner is not reset
            // it is allowed  to continue work for more time on an unsolved problem
            // otherwise it will be reset and start from scratch
            //  plan->clear();

            // create termination condition
            ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

            // printf("Solving!\n");
            solved = planner_->solve(ptc);
        } while (solved != ob::PlannerStatus::EXACT_SOLUTION);

        auto dt1 = ros::Time::now() - t1;
        dts_to_msec[1] = dt1.toSec() * 1000;

        sum_times[1] += dts_to_msec[1];
        max_times[1] = std::max(max_times[1], dts_to_msec[1]);

        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            // std::cout << "Found exact solution :" << std::endl;

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            // printf("Simplifying and interpolating\n");
            auto t2 = ros::Time::now();
            if (prob_params.simplify_path)
            {
                auto t0 = ros::Time::now();
                // std::cout << "Simplifying path...\n";
                og::PathSimplifier path_simplifier(si, pdef->getGoal());

                ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

                if (prob_params.realtime_settings.simplifying_path == problem_params::SimplifyingPath::FAST)
                {
                    path_simplifier.shortcutPath(*pth);
                    path_simplifier.reduceVertices(*pth);
                    path_simplifier.collapseCloseVertices(*pth);
                    path_simplifier.reduceVertices(*pth);

                    /*
                    // path_simplifier.perturbPath(*pth, 0.2);

                    bool isMetric = si->getStateSpace()->isMetricSpace();
                    printf("isMetric: %d\n", isMetric);

                    bool atLeastOnce = true;
                    bool tryMore = true, valid = true;
                    unsigned int total_times = 0;
                    while ((ptc == false || atLeastOnce) && tryMore)
                    {
                        // if ((ptc == false || atLeastOnce) && si->getStateSpace()->isMetricSpace())
                        // {
                        //     bool metricTryMore = true;
                        //     unsigned int times = 0;
                        //     do
                        //     {
                        //         bool shortcut = path_simplifier.shortcutPath(*pth); // split path segments, not just vertices

                        //         metricTryMore = shortcut;
                        //     } while ((ptc == false || atLeastOnce) && ++times <= 4 && metricTryMore);

                        //     // smooth the path with BSpline interpolation
                        //     if (ptc == false || atLeastOnce)
                        //         path_simplifier.smoothBSpline(*pth, 3, (*pth).length() / 100.0);
                        // }
                        // // try a randomized step of connecting vertices
                        // if (ptc == false || atLeastOnce)
                        //     tryMore = path_simplifier.reduceVertices(*pth);

                        // // try to collapse close-by vertices
                        // if (ptc == false || atLeastOnce)
                        //     path_simplifier.collapseCloseVertices(*pth);
                        printf("total_times:%d \n", total_times++);
                        if (total_times > 3)
                            break;
                        bool shortcut;
                        shortcut = path_simplifier.shortcutPath(*pth); // split path segments, not just vertices
                        shortcut = path_simplifier.shortcutPath(*pth); // split path segments, not just vertices
                        shortcut = path_simplifier.shortcutPath(*pth); // split path segments, not just vertices

                        // path_simplifier.smoothBSpline(*pth, 3, (*pth).length() / 100.0);
                        tryMore = path_simplifier.reduceVertices(*pth);

                        path_simplifier.collapseCloseVertices(*pth);

                        unsigned int times = 0;
                        while ((ptc == false || atLeastOnce) && tryMore && ++times <= 3)
                            tryMore = path_simplifier.reduceVertices(*pth);
                    }
                    */
                    /*
                    auto path = *pth;
                    bool atLeastOnce = true;
                    bool tryMore = true, valid = true;
                    while ((ptc == false || atLeastOnce) && tryMore)
                    {
                        // if the space is metric, we can do some additional smoothing
                        if ((ptc == false || atLeastOnce) && si_->getStateSpace()->isMetricSpace())
                        {
                            bool metricTryMore = true;
                            unsigned int times = 0;
                            do
                            {
                                bool shortcut = path_simplifier.shortcutPath(path);          // split path segments, not just vertices
                                bool better_goal = gsr_ ? findBetterGoal(path, ptc) : false; // Try to connect the path to a closer goal

                                metricTryMore = shortcut || better_goal;
                            } while ((ptc == false || atLeastOnce) && ++times <= 5 && metricTryMore);

                            // smooth the path with BSpline interpolation
                            if (ptc == false || atLeastOnce)
                                smoothBSpline(path, 3, path.length() / 100.0);

                            if (ptc == false || atLeastOnce)
                            {
                                // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
                                const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
                                if (!p.second)
                                {
                                    valid = false;
                                    OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
                                }
                                else if (!p.first)
                                    OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but "
                                               "it was "
                                               "successfully fixed.");
                            }
                        }

                        // try a randomized step of connecting vertices
                        if (ptc == false || atLeastOnce)
                            tryMore = reduceVertices(path);

                        // try to collapse close-by vertices
                        if (ptc == false || atLeastOnce)
                            collapseCloseVertices(path);

                        // try to reduce verices some more, if there is any point in doing so
                        unsigned int times = 0;
                        while ((ptc == false || atLeastOnce) && tryMore && ++times <= 5)
                            tryMore = reduceVertices(path);

                        if ((ptc == false || atLeastOnce) && si_->getStateSpace()->isMetricSpace())
                        {
                            // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
                            const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
                            if (!p.second)
                            {
                                valid = false;
                                OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
                            }
                            else if (!p.first)
                                OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but it "
                                           "was "
                                           "successfully fixed.");
                        }

                        atLeastOnce = false;
                    }
                    return valid || path.check();
                    */
                }
                else if (prob_params.realtime_settings.simplifying_path == problem_params::SimplifyingPath::FULL)
                {
                    path_simplifier.simplify(*pth, ptc, false);
                }
                else
                {
                    // no simplyfing
                }

                // path_simplifier.simplifyMax(*pth);
            }
            auto dt2 = ros::Time::now() - t2;

            dts_to_msec[2] = dt2.toSec() * 1000;
            sum_times[2] += dts_to_msec[2];
            max_times[2] = std::max(max_times[2], dts_to_msec[2]);

            auto t3 = ros::Time::now();
            if (prob_params.path_interpolation_points > 0)
            {
                // std::cout << "Interpolating path...\n";
                // og::PathGeometric::InterpolationType interp_type = og::PathGeometric::CSPLINE;
                // pth->interpolate(interp_type);
                pth->interpolate(prob_params.path_interpolation_points);
            }
            auto dt3 = ros::Time::now() - t3;
            dts_to_msec[3] = dt3.toSec() * 1000;
            sum_times[3] += dts_to_msec[3];
            max_times[3] = std::max(max_times[3], dts_to_msec[3]);

            // std::cout << "Final path :" << std::endl;
            // pth->printAsMatrix(std::cout);

            // // save path to file
            // auto t0 = ros::Time::now();
            // std::ofstream myfile;
            // myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
            // pth->printAsMatrix(myfile);
            // myfile.close();
            // auto dt = ros::Time::now() - t0;
            // saving_path_time = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();

            auto dt0 = ros::Time::now() - t0;
            dts_to_msec[0] = dt0.toSec() * 1000;
            sum_times[0] += dts_to_msec[0];
            max_times[0] = std::max(max_times[0], dts_to_msec[0]);

            bool print_times = true;
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

        if ((counter % 200) == 0)
        {
            // std::cout << "\r"; // print at the same line (update effect)

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

    ompl::geometric::PathGeometric *planner::getPath() { return path_; }

    ompl::base::SpaceInformationPtr planner::getSpaceInformation() { return si; }
} // namespace drones_rope_planning