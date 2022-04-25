#pragma once

// fcl
#include "fcl/common/types.h"
#include "fcl/config.h"
#include "fcl/fcl.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/math/geometry-inl.h"
#include "fcl/narrowphase/collision-inl.h"

// ROS
#include <ros/ros.h>

namespace realtime_obstacles
{
    struct CylinderDefinition
    {
        double radius;
        double height;
        double pos[3];
    };

    class Cylinders
    {
    public:
        Cylinders(std::vector<CylinderDefinition> cylinders_def);

        Cylinders(int obs_number);

        ~Cylinders();

        void loadCylinders();

        void update_cylinders_transforms(std::vector<fcl::Transform3f> cylinders_transforms);

        void set_cylinder_transform(int index, float pos[3], float q[4]);

        void get_collision_objects();

        bool collision_detection(fcl::CollisionObject<float> *robot);

    private:
        void create_collision_objects();

        std::vector<std::shared_ptr<fcl::Cylinder<float>>> cylinders;

        // collison object
        std::vector<fcl::CollisionObject<float> *> collision_objects;

        // helping variables for collision checking
        fcl::CollisionResult<float> result;
        fcl::CollisionRequest<float> request;

        // keeps tracks of the changes in the cylinders made
        // std::vector<bool> cylinders_coll_object_updated;
    };

    std::vector<CylinderDefinition> load_cylinders_definition(ros::NodeHandle &nh);
}; // namespace realtime_obstacles
