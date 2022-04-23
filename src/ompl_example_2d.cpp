/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

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
        checker = fcl_checking::checker();

        prob_params = prob_prms;
        problem_params::printProblemParams(prob_params);

        try
        {
            std::string env_mesh = "/home/marios/thesis_ws/src/drones_rope_planning/resources/stl/" + prob_params.env_filename + ".stl";
            checker.loadEnvironment(env_mesh);
        }
        catch (std::exception &e)
        {
            std::cout << "Using environment mesh from drones_path_planning" << std::endl;
            std::string env_mesh = "/home/marios/thesis_ws/src/drone_path_planning/resources/stl/" + prob_params.env_filename + ".stl";
            checker.loadEnvironment(env_mesh);
        }

        // checker.loadRobot(robot_filename);

        L = prob_params.L;

        custom_robot_mesh = new custom_mesh::CustomMesh(L, prob_params.safety_offsets, prob_params.thickness);
        checker.setRobotMesh(custom_robot_mesh->get_fcl_mesh());

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
        // pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));
        // pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

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
        /*
            // Add goal states
            ob::GoalStates *goalStates(new ob::GoalStates(si));

            ob::ScopedState<> s(si);
            s->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            s->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            s->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            s->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
            s->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            s->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            ob::ScopedState<> s2(si);
            s2->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            s2->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            s2->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            s2->as<ob::RealVectorStateSpace::StateType>()->values[3] = +M_PI;
            s2->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            s2->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            ob::ScopedState<> s3(si);
            s3->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            s3->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            s3->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            s3->as<ob::RealVectorStateSpace::StateType>()->values[3] = -M_PI;
            s3->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            s3->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            goalStates->addState(s);
            goalStates->addState(s2);
            goalStates->addState(s3);

            // create RealVector start_state
            ob::ScopedState<> start_state(planner::space);
            ob::ScopedState<> goal_state(planner::space);

            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
            goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

            // pdef->setStartAndGoalStates(start_state, goal_state);

            // Set start and goal states
            pdef->addStartState(start_state);
            pdef->setGoal(ob::GoalPtr(goalStates));
        */

        ob::ScopedState<> start_state(planner::space);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        pdef->addStartState(start_state);

        // choose goal state
        if (prob_params.use_dynamic_goal)
        {
            auto goal_region = std::make_shared<SymmetricalGoal>(si, goal, 0.5);
            pdef->setGoal(ob::GoalPtr(goal_region));

            float start_distance = 0.1;
            float goal_distance = 0.5;
            unsigned int max_attempts = 100;
            pdef->fixInvalidInputStates(start_distance, goal_distance, max_attempts);
        }
        else
        {
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
        }
    }

    template <typename T> ob::Planner *createInstance(ob::SpaceInformationPtr si) { return std::make_shared<T>(si); }

    typedef std::map<std::string, ob::PlannerPtr (*)()> map_type;

    ob::PlannerPtr planner::getPlanner(std::string planner_name, float range)
    {
        // choose between ompl planners

        // planners_map plnrs = {
        //     //
        //     {"RRT", std::make_shared<og::RRT>(si)},
        //     {"RRTConnect", std::make_shared<og::RRTConnect>(si)},
        //     {"RRTstar", std::make_shared<og::RRTstar>(si)},
        //     // {"RRTstarProb", std::make_shared<og::RRTstarProb>(si)},
        //     // {"RRTsharp", std::make_shared<og::RRTsharp>(si)},
        //     // {"LBTRRT", std::make_shared<og::LBTRRT>(si)},
        //     {"LazyRRT", std::make_shared<og::LazyRRT>(si)},
        //     // {"LazyRRTstar", std::make_shared<og::LazyRRTstar>(si)},
        //     // {"PRM", std::make_shared<og::PRM>(si)},
        //     // {"PRMstar", std::make_shared<og::PRMstar>(si)},
        //     // {"SBL", std::make_shared<og::SBL>(si)},
        //     // {"SPARS", std::make_shared<og::SPARS>(si)},
        //     // {"SPARStwo", std::make_shared<og::SPARStwo>(si)},
        //     // {"SPARStwoProb", std::make_shared<og::SPARStwoProb>(si)},
        //     // {"SPARStwoProbFast", std::make_shared<og::SPARStwoProbFast>(si)},
        //     // {"SPARStwoProbFastest", std::make_shared<og::SPARStwoProbFastest>(si)},
        //     // {"SPARStwoProbFastest2", std::make_shared<og::SPARStwoProbFastest2>(si)},
        //     // {"SPARStwoProbFastest3", std::make_shared<og::SPARStwoProbFastest3>(si)},
        //     // {"SPARStwoProbFastest4", std::make_shared<og::SPARStwoProbFastest4>(si)},
        //     // {"SPARStwoProbFastest5", std::make_shared<og::SPARStwoProbFastest5>(si)},
        //     // {"SPARStwoProbFastest6", std::make_shared<og::SPARStwoProbFastest6>(si)},
        //     // {"SPARStwoProbFastest7", std::make_shared<og::SPARStwoProbFastest7>(si)},
        //     // {"SPARStwoProbFastest8", std::make_shared<og::SPARStwoProbFastest8>(si)}
        // };

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
        else
        {
            auto plan = std::make_shared<og::RRT>(si);

            printf("Setting  range...\n");
            plan->setRange(range);

            return plan;
        }
    }

    void planner::plan()
    {

        // create a planner for the defined space
        // ob::PlannerPtr plan(new og::InformedRRTstar(si));
        // ob::PlannerPtr plan(new og::RRT(si));

        // auto plan =          std::make_shared<og::RRT>(si);

        // auto plan = std::make_shared<og::RRTConnect>(si);

        auto plan = getPlanner(prob_params.planner_algorithm, prob_params.range);

        // as og::RRT is not correct but it works (need to find a general interface that included setRange)
        // plan->as<og::RRT>()->setRange(prob_params.range);

        // set the problem we are trying to solve for the planner
        printf("Setting  problem definition...\n");
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        printf("Setting  planner up...\n");
        plan->setup();

        // print the settings for this space
        printf("============================ OMPL SPACE SETTINGS: ============================\n");
        si->printSettings(std::cout);
        // printf("===============================================================================\n");

        // print the problem settings
        printf("============================ OMPL PROBLEM SETTINGS: ============================\n");
        pdef->print(std::cout);
        printf("================================================================================\n");

        // create termination condition
        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));
        // attempt to solve the problem within one second of planning time
        auto t0 = std::chrono::high_resolution_clock::now();
        ob::PlannerStatus solved = plan->solve(ptc);
        std::cout << std::endl;
        auto dt = std::chrono::high_resolution_clock::now() - t0;
        std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution :" << std::endl;

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

            if (prob_params.simplify_path)
            {
                std::cout << "Simplifying path...\n";
                og::PathSimplifier path_simplifier(si, pdef->getGoal());
                // path_simplifier.simplify(*pth, 10.0);
                path_simplifier.simplifyMax(*pth);
            }

            if (prob_params.path_interpolation_points > 0)
            {
                std::cout << "Interpolating path...\n";
                // og::PathGeometric::InterpolationType interp_type = og::PathGeometric::CSPLINE;
                // pth->interpolate(interp_type);

                pth->interpolate(prob_params.path_interpolation_points);
            }

            std::cout << "Finale path :" << std::endl;
            pth->printAsMatrix(std::cout);

            // save path to file
            std::ofstream myfile;
            myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
            pth->printAsMatrix(myfile);
            myfile.close();
        }
        else
            std::cout << "No solution found" << std::endl;
    }

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
        checker.update_robot(custom_robot_mesh->get_fcl_mesh());

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
        checker.setRobotTransform(pos, quat);

        // check colllision
        bool result = !checker.check_collision();

        // time measurements
        auto dt = ros::Time::now() - t0;
        total_time += dt.toSec() * 1000; // sum msecs
        counter++;

        if ((counter % 5000) == 0)
        {
            std::cout << "\r"; // print at the same line (update effect)

            std::cout << "State Validation: " << counter << " calls " << total_time / 1000 << " secs, " << total_time / counter << " msecs/call, "
                      << counter * 1000.0 / total_time << " calls/sec" << std::flush;
        }

        return result;
    }
}
