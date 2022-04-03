#pragma once

#include "fcl/geometry/bvh/BVH_model-inl.h"
#include "fcl/geometry/bvh/BVH_model.h"

// #include "fcl/math/bv/utility-inl.h"
#include "fcl/math/bv/OBBRSS.h"
#include "fcl/narrowphase/collision-inl.h"

#define STLLOADER_IMPLEMENTATION
#include "stlloader.hpp"

namespace fcl_checking
{
    class checker
    {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        checker();

        /*!
         * Destructor.
         */
        virtual ~checker();

        void setEnvironment();

        void setRobotTransform(void);

        bool check_collision(void);

        void update_robot(void);

    private:
    };
}