#pragma once

// fcl core

#include "fcl/geometry/shape/cylinder.h"
// collisions
#include "fcl/narrowphase/collision-inl.h"
// Vector3
#include "fcl/math/geometry-inl.h"
// Transform3
#include "fcl/config.h"
#include "fcl/fcl.h"

// CollisionObject
#include "fcl/common/types.h"

namespace realtime_obstacles
{
    class Cylinders
    {
    public:
        Cylinders(int N);

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

}; // namespace realtime_obstacles
