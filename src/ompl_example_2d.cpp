#include "../include/ompl_example_2d/ompl_example_2d.hpp"

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_rope_planning
{
    ProblemParams getProblemParams(ros::NodeHandle &nh)
    {
        ProblemParams pdef;
        ros::param::get("/planning/timeout", pdef.timeout);
        ros::param::get("/planning/rope_length", pdef.L);
        ros::param::get("/planning/env_mesh", pdef.env_filename);

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
        pdef.bounds["low"][3] = low_bounds[3];
        pdef.bounds["low"][4] = low_bounds[4];
        pdef.bounds["low"][5] = low_bounds[5];

        pdef.bounds["high"][0] = high_bounds[0];
        pdef.bounds["high"][1] = high_bounds[1];
        pdef.bounds["high"][2] = high_bounds[2];
        pdef.bounds["high"][3] = high_bounds[3];
        pdef.bounds["high"][4] = high_bounds[4];
        pdef.bounds["high"][5] = high_bounds[5];

        ros::param::get("/planning/planner_algorithm", pdef.planner_algorithm);

        ros::param::get("/planning/val_check_resolution", pdef.val_check_resolution);
        ros::param::get("/planning/range", pdef.range);

        return pdef;
    }

    planner::planner(ProblemParams prob_prms)
    {
        // setting problem parameters
        prob_params = prob_prms;

        checker = fcl_checking::checker();
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

        L = prob_params.L;
        custom_robot_mesh = new custom_mesh::CustomMesh(L);
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
        bounds.setLow(0, prob_params.bounds.at("low")[0]); // x
        bounds.setLow(1, prob_params.bounds.at("low")[1]); // y
        bounds.setLow(2, prob_params.bounds.at("low")[2]); // z
        bounds.setLow(3, prob_params.bounds.at("low")[3]); // yaw
        bounds.setLow(4, prob_params.bounds.at("low")[4]); // drones_distance
        bounds.setLow(5, prob_params.bounds.at("low")[5]); // drones_angle

        // Higher Bounds
        bounds.setHigh(0, prob_params.bounds.at("high")[0]); // x
        bounds.setHigh(1, prob_params.bounds.at("high")[1]); // y
        bounds.setHigh(2, prob_params.bounds.at("high")[2]); // z
        bounds.setHigh(3, prob_params.bounds.at("high")[3]); // yaw
        bounds.setHigh(4, prob_params.bounds.at("high")[4]); // drones_distance
        bounds.setHigh(5, prob_params.bounds.at("high")[5]); // drones_angle

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    }

    void planner::setStartGoal(float start[6], float goal[6])
    {
        // create RealVector start_state
        ob::ScopedState<> start_state(planner::space);
        ob::ScopedState<> goal_state(planner::space);

        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start[3];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start[4];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start[5];

        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal[3];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal[4];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal[5];

        pdef->setStartAndGoalStates(start_state, goal_state);
    }

    void planner::plan()
    {

        // create a planner for the defined space
        // ob::PlannerPtr plan(new og::InformedRRTstar(si));
        // ob::PlannerPtr plan(new og::RRT(si));

        auto plan = std::make_shared<og::RRT>(si);
        printf("Setting  range...\n");
        plan->setRange(prob_params.range);

        // set the problem we are trying to solve for the planner
        printf("Setting  problem definition...\n");
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        printf("Setting  planner up...\n");
        plan->setup();

        // print the settings for this space
        printf("\n ========================== SPACE SETTINGS: ==========================\n");
        si->printSettings(std::cout);

        // print the problem settings
        printf("\n ========================== PROBLEM SETTINGS: ==========================\n");
        pdef->print(std::cout);

        // create termination condition
        ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(prob_params.timeout));

        auto t0 = std::chrono::high_resolution_clock::now();

        ob::PlannerStatus solved = plan->solve(ptc);
        std::cout << std::endl;

        auto dt = std::chrono::high_resolution_clock::now() - t0;
        std::cout << "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() << " ms" << std::endl;

        if (solved.EXACT_SOLUTION)
        {
            std::cout << "Found exact solution!" << std::endl;
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path

            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();

            og::PathSimplifier path_simplifier(si, pdef->getGoal());
            std::cout << "Simplifying path...\n";
            path_simplifier.simplify(*pth, 40.0);

            std::cout << "Interpolating path...\n";
            pth->interpolate(30);

            pth->printAsMatrix(std::cout);

            // save path to file
            std::ofstream myfile;
            myfile.open("/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt");
            pth->printAsMatrix(myfile);
            myfile.close();
        }
        else if (solved.APPROXIMATE_SOLUTION)
        {
            std::cout << "Found approximate solution!" << std::endl;
        }
        else
        {
            std::cout << "Did not find a solution!" << std::endl;
        }
    }

    // std::ostream &operator<<(std::ostream &os, const fcl::BVHBuildState &obj)
    // {
    //     os << static_cast<std::underlying_type<fcl::BVHBuildState>::type>(obj);
    //     return os;
    // }

    bool planner::isStateValid(const ob::State *state_check)
    {
        // printf("========================================================\n");
        static int counter = 0;
        static float total_time = 0;
        static float total_time_2 = 0;
        const ob::RealVectorStateSpace::StateType *state = state_check->as<ob::RealVectorStateSpace::StateType>();

        auto t0 = ros::Time::now();

        float pos[3];
        pos[0] = state->values[0];
        pos[1] = state->values[1];
        pos[2] = state->values[2];

        const auto yaw = state->values[3];

        const float drones_dis = state->values[4];
        const float drones_angle = state->values[5];

        custom_robot_mesh->update_mesh(drones_dis, drones_angle);

        checker.update_robot(custom_robot_mesh->get_fcl_mesh());

        // apply yaw rotation
        tf2::Quaternion q;

        q.setRPY(0, 0, yaw);
        q = q.normalize();

        float quat[4] = {q.x(), q.y(), q.z(), q.w()};
        checker.setRobotTransform(pos, quat);

        auto t0_2 = ros::Time::now();

        // check colllision
        bool result = !checker.check_collision();

        auto dt2 = ros::Time::now() - t0_2;

        auto dt = ros::Time::now() - t0;

        total_time += dt.toSec() * 1000;    // sum msecs
        total_time_2 += dt2.toSec() * 1000; // sum msecs

        counter++;
        if ((counter % 10000) == 0)
        {
            std::cout << "Collision checking() : " << total_time_2 / counter << " msecs" << std::endl;
            std::cout << "Checking state: " << counter << " avrg time: " << total_time / counter << " msec";

            // std::cout << "\r"
            //   << "Checking state: " << counter << " msec";
        }

        return result;
    }
}
