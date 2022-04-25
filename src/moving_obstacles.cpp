#include "../include/moving_obstacles.hpp"

namespace realtime_obstacles
{

    Cylinders::Cylinders(int N)
    {
        // resizing the vectors
        cylinders.resize(N);
        collision_objects.resize(N);

        loadCylinders();
        create_collision_objects();

        // fcl collision detection
        request.num_max_contacts = 1;
        request.enable_contact = false;
    }

    Cylinders::~Cylinders() {}

    void Cylinders::loadCylinders()
    {
        for (int i = 0; i < cylinders.size(); i++)
        {
            float radius = 0.1;
            float height = 2;

            // cylinders[i] = new fcl::Cylinder<float>(radius, height);
            auto c = std::make_shared<fcl::Cylinder<float>>(radius, height);
            cylinders[i] = c;
        }
    }

    void Cylinders::create_collision_objects()
    {
        // fcl identity transform
        fcl::Transform3f identity_transform;
        identity_transform.setIdentity();

        for (int i = 0; i < cylinders.size(); i++)
        {
            collision_objects[i] = new fcl::CollisionObject<float>(cylinders[i], identity_transform);
        }
    }

    void Cylinders::update_cylinders_transforms(std::vector<fcl::Transform3f> cylinders_transforms)
    {
        for (int i = 0; i < cylinders_transforms.size(); i++)
        {
            collision_objects[i]->setTransform(cylinders_transforms[i]);
        }
    }

    void Cylinders::set_cylinder_transform(int index, float pos[3], float q[4])
    {
        collision_objects[index]->setTranslation(fcl::Vector3<float>(pos[0], pos[1], pos[2]));
        collision_objects[index]->setQuatRotation(fcl::Quaternion<float>(q[0], q[1], q[2], q[3]));
    }

    bool Cylinders::collision_detection(fcl::CollisionObject<float> *robot)
    {
        for (int i = 0; i < collision_objects.size(); i++)
        {
            result.clear();
            fcl::collide(robot, collision_objects[i], request, result);
            if (result.isCollision())
            {
                return true;
            }
        }

        return false;
    }

} // namespace realtime_obstacles
