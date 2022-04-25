#include "../include/moving_obstacles.hpp"

namespace realtime_obstacles
{
    Cylinders::Cylinders(std::vector<CylinderDefinition> cylinders_def)
    {
        // resizing the vectors
        int N = cylinders_def.size();

        cylinders.resize(N);
        collision_objects.resize(N);

        loadCylinders();
        create_collision_objects();

        // fcl collision detection
        request.num_max_contacts = 1;
        request.enable_contact = false;
    }

    Cylinders::Cylinders(int obs_number)
    {
        cylinders.resize(obs_number);
        collision_objects.resize(obs_number);

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
            float radius = 0.2;
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
        printf("Setting index: %d at pos : %f %f %f \n", index, pos[0], pos[1], pos[2]);

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

    std::vector<CylinderDefinition> load_cylinders_definition(ros::NodeHandle &nh)
    {
        // initializing the cylinders
        std::vector<CylinderDefinition> cylinders_def;
        int N;
        ros::param::get("/obstacles/cylinders_number", N);

        XmlRpc::XmlRpcValue cyls_array;
        ros::param::get("/obstacles/cylinders", cyls_array);

        // XmlRpc::XmlRpcValue x = cyls_array[0];

        // double r = x["radius"];

        // printf("mlkia %f\n", r);

        for (int i = 0; i < N; i++)
        {
            CylinderDefinition cyl_def;

            XmlRpc::XmlRpcValue cyl = cyls_array[i];

            cyl_def.radius = cyl["radius"];
            cyl_def.height = cyl["height"];

            cyl_def.pos[0] = cyl["x"];
            cyl_def.pos[1] = cyl["y"];
            cyl_def.pos[2] = cyl["z"];

            cylinders_def.push_back(cyl_def);
        }

        for (int i = 0; i < N; i++)
        {
            printf("%f %f %f %f %f\n", cylinders_def[i].radius, cylinders_def[i].height, cylinders_def[i].pos[0], cylinders_def[i].pos[1],
                   cylinders_def[i].pos[2]);
        }

        return cylinders_def;
    }

} // namespace realtime_obstacles
