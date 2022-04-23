#pragma once

#include <ompl/base/goals/GoalRegion.h>

#include <ompl/config.h>

// Define the goal we want to reach

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SymmetricalGoal : public ob::GoalRegion
{
public:
    SymmetricalGoal(const ob::SpaceInformationPtr &si, const float goal[6], float dist_thres) : ob::GoalRegion(si)
    {
        distance_threshold = dist_thres;

        // set the goal state
        goal_pos[0] = goal[0];
        goal_pos[1] = goal[1];
        goal_pos[2] = goal[2];
    }

    ~SymmetricalGoal() = default;

    virtual bool isSatisfied(const ob::State *state) const override
    {
        // check if the distance is less than some threshold
        float dist = distanceGoal(state);

        return dist < distance_threshold;
    }

    virtual double distanceGoal(const ob::State *state) const override
    {
        // cast the state to what we expect it to be
        double pos[3];
        pos[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        pos[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        pos[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];

        // check if the distance is less than some threshold
        float dist = sqrt(pow(pos[0] - goal_pos[0], 2) + pow(pos[1] - goal_pos[1], 2) + pow(pos[2] - goal_pos[2], 2));

        return dist;
    }

    void print(std::ostream &out) const override { out << "SymmetricalGoal"; }

private:
    float goal_pos[3];

    float distance_threshold;
};

class CustomSamplabeGoal : public ob::GoalSampleableRegion
{
public:
    CustomSamplabeGoal(const ob::SpaceInformationPtr &si, float goal[6], float dist_thres) : ob::GoalSampleableRegion(si)
    {
        distance_threshold = dist_thres;

        // set the goal state
        goal_pos[0] = goal[0];
        goal_pos[1] = goal[1];
        goal_pos[2] = goal[2];

        goal_drones_distance=goal[4];
    }

    ~CustomSamplabeGoal() = default;

    virtual void sampleGoal(ob::State *state) const override
    {
        state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_pos[0];
        state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_pos[1];
        state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_pos[2];
        state->as<ob::RealVectorStateSpace::StateType>()->values[3] = 0;
        state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal_drones_distance;
        state->as<ob::RealVectorStateSpace::StateType>()->values[5] = 0;
    }

    virtual bool isSatisfied(const ob::State *state) const override
    {
        // check if the distance is less than some threshold
        float dist = distanceGoal(state);

        return dist < distance_threshold;
    }

    virtual double distanceGoal(const ob::State *state) const override
    {
        // cast the state to what we expect it to be
        double pos[3];
        pos[0] = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
        pos[1] = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
        pos[2] = state->as<ob::RealVectorStateSpace::StateType>()->values[2];

        // check if the distance is less than some threshold
        float dist = sqrt(pow(pos[0] - goal_pos[0], 2) + pow(pos[1] - goal_pos[1], 2) + pow(pos[2] - goal_pos[2], 2));

        return dist;
    }

    void print(std::ostream &out) const override
    {
        out << "CustomSamplabeGoal";

        // print the state
        out << "(" << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << ")";
    }

    unsigned int maxSampleCount() const override { return 1; }

private:
    float goal_pos[3];

    float goal_drones_distance;

    float distance_threshold;
};
