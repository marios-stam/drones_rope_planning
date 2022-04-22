#pragma once

#include <ompl/base/goals/GoalRegion.h>

#include <ompl/config.h>

// Define the goal we want to reach
class SymmetricalGoal : public ob::GoalRegion
{
public:
    SymmetricalGoal(const ob::SpaceInformationPtr &si, const ob::State *goal, float dist_thres) : ob::GoalRegion(si)
    {
        distance_threshold = dist_thres;

        // set the goal state
        goal_pos[0] = goal->as<ob::RealVectorStateSpace::StateType>()->values[0];
        goal_pos[1] = goal->as<ob::RealVectorStateSpace::StateType>()->values[1];
        goal_pos[2] = goal->as<ob::RealVectorStateSpace::StateType>()->values[2];
    }

    virtual ~SymmetricalGoal() = default;

    virtual bool isSatisfied(const ob::State *state) const override
    {
        // check if the distance is less than some threshold
        float dist = distanceGoal(state);

        return dist < distance_threshold;
    }

    virtual float distanceGoal(const ob::State *state) const override
    {
        // cast the state to what we expect it to be
        float pos[3];
        pos[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        pos[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        pos[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];

        // check if the distance is less than some threshold
        float dist = sqrt(pow(pos[0] - goal_pos[0], 2) + pow(pos[1] - goal_pos[1], 2) + pow(pos[2] - goal_pos[2], 2));

        return dist;
    }

    virtual void print(std::ostream &out) const override { out << "SymmetricalGoal"; }

private:
    float goal_pos[3];

    float distance_threshold;
};
