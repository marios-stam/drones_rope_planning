#pragma once
// include std::max
#include <algorithm>
#include <stdlib.h>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>

// include stateCostIntegralObjective
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

// include Cost
#include <ompl/base/Cost.h>
#include <ompl/base/goals/GoalState.h>

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace custom_objectives
{
    class RopeRelaxedObjective : public ob::StateCostIntegralObjective
    {
    public:
        RopeRelaxedObjective(const ob::SpaceInformationPtr &si, float rope_length) : ob::StateCostIntegralObjective(si)
        {
            // set the cost threshold
            cost_threshold = 0.0;
            L = rope_length;
        }

        ~RopeRelaxedObjective() = default;

        virtual ob::Cost stateCost(const ob::State *state) const override
        {
            // cast the state to what we expect it to be
            double ropes_distance = state->as<ob::RealVectorStateSpace::StateType>()->values[4];

            // check if the distance is less than some threshold
            float dist = abs(ropes_distance - L);

            // return the cost
            return ob::Cost(dist);
        }

        virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override
        {
            // cast the states to what we expect it to be
            double ropes_distance = s2->as<ob::RealVectorStateSpace::StateType>()->values[4];

            // check if the distance is less than some threshold
            float dist = abs(ropes_distance - L);

            // return the cost
            return ob::Cost(dist);
        }

    private:
        float L;
        float cost_threshold;
    };

    class DistanceToGoalObjective : public ob::StateCostIntegralObjective
    {
    public:
        DistanceToGoalObjective(const ob::SpaceInformationPtr &si, float goal[6]) : ob::StateCostIntegralObjective(si)
        {
            // set the goal state
            goal_pos[0] = goal[0];
            goal_pos[1] = goal[1];
            goal_pos[2] = goal[2];

            // set the cost threshold
            cost_threshold = 0.0;
        }

        ~DistanceToGoalObjective() = default;

        virtual ob::Cost stateCost(const ob::State *state) const override
        {
            // cast the state to what we expect it to be
            double pos[3];
            pos[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            pos[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pos[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];

            // check if the distance is less than some threshold
            float dist = sqrt(pow(pos[0] - goal_pos[0], 2) + pow(pos[1] - goal_pos[1], 2) + pow(pos[2] - goal_pos[2], 2));

            // return the cost
            return ob::Cost(dist);
        }

    private:
        float goal_pos[3], cost_threshold;
    };

    ob::Cost goalRegionCostToGo(const ob::State *state, const ob::Goal *goal);

    ob::OptimizationObjectivePtr custom_heuristic(const ob::SpaceInformationPtr &si);

    // TODO: make a function that chooses which objective to use based on the cooresponding parameter

}; // namespace custom_objectives