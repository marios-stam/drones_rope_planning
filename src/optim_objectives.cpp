#include "../include/ompl_example_2d/optim_objectives.hpp"

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace custom_objectives
{

    ob::Cost goalRegionCostToGo(const ob::State *state, const ob::Goal *goal)
    {
        // cast the state to what we expect it to be
        double goal_pos[3], pos[3];
        memcpy(pos, state->as<ob::RealVectorStateSpace::StateType>()->values, 3 * sizeof(double));
        memcpy(goal_pos, goal->as<ob::GoalState>()->getState()->as<ob::RealVectorStateSpace::StateType>()->values, 3 * sizeof(double));

        float dist = sqrt(pow(pos[0] - goal_pos[0], 2) + pow(pos[1] - goal_pos[1], 2) + pow(pos[2] - goal_pos[2], 2));

        float threshold = 0.2;
        // Ensures that all states within the goal region's threshold to
        // have a cost-to-go of exactly zero.
        // find max of 2 values
        float max = std::max<float>(dist - threshold, 0.0);

        return ob::Cost(max);
    }

    ob::OptimizationObjectivePtr custom_heuristic(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
        obj->setCostToGoHeuristic(&goalRegionCostToGo);
        return obj;
    }

    // TODO: make a function that chooses which objective to use based on the cooresponding parameter
}; // namespace custom_objective