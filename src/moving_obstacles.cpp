#include "../include/moving_obstacles.hpp"

namespace realtime_obstacles
{
    Cylinders::Cylinders(std::vector<CylinderDefinition> cylinders_def)
    {
        // resizing the vectors
        int N = cylinders_def.size();

        cylinders.resize(N);
        collision_objects.resize(N);

        // velocity calculation
        velocities.resize(N, 3);
        prev_pos.setZero(N, 3);
        times.setConstant(N, 1, ros::Time::now().toSec());

        loadCylinders(cylinders_def);
        create_collision_objects();
        update_cylinders_transforms(cylinders_def);

        // fcl collision detection
        request.num_max_contacts = 1;
        request.enable_contact = false;
    }

    Cylinders::Cylinders(int obs_number)
    {
        cylinders.resize(obs_number);
        collision_objects.resize(obs_number);

        velocities.resize(obs_number, 3);
        prev_pos.setZero(obs_number, 3);
        times.setConstant(obs_number, 1, ros::Time::now().toSec());

        loadCylinders();
        create_collision_objects();

        // fcl collision detection
        request.num_max_contacts = 1;
        request.enable_contact = false;
    }
    Cylinders::~Cylinders() {}

    void Cylinders::loadCylinders(std::vector<CylinderDefinition> cylinders_def)
    {
        fcl::Transform3f identity_transform;
        identity_transform.setIdentity();

        for (int i = 0; i < cylinders_def.size(); i++)
        {
            float radius = cylinders_def[i].radius;
            float height = cylinders_def[i].height;

            // cylinders[i] = new fcl::Cylinder<float>(radius, height);
            auto c = std::make_shared<fcl::Cylinder<float>>(radius, height);
            cylinders[i] = c;
        }
    }

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
        float curr_time = ros::Time::now().toSec();

        for (int i = 0; i < cylinders_transforms.size(); i++)
        {
            auto prev_position = collision_objects[i]->getTransform().translation(); // TODO: check if this is deep copy

            collision_objects[i]->setTransform(cylinders_transforms[i]);

            velocities.row(i) = (cylinders_transforms[i].translation() - prev_position) / (curr_time - times(i));
            printf("velocity2: %f %f %f\n", velocities(i, 0), velocities(i, 1), velocities(i, 2));
        }

        times.setConstant(curr_time);
    }

    void Cylinders::update_cylinders_transforms(std::vector<CylinderDefinition> cylinders_def)
    {
        float q[4] = {1, 0, 0, 0};

        for (int i = 0; i < cylinders_def.size(); i++)
        {
            float pos[3] = {cylinders_def[i].pos[0], cylinders_def[i].pos[1], cylinders_def[i].pos[2]};
            set_cylinder_transform(i, pos, q);
        }
    }

    void Cylinders::set_cylinder_transform(int index, float pos[3], float q[4])
    {
        // data for calcluating celocity in visualization
        double curr_time = ros::Time::now().toSec();
        auto prev_position = collision_objects[index]->getTransform().translation(); // TODO: check if this is deep copy

        // setting transform
        collision_objects[index]->setTranslation(fcl::Vector3<float>(pos[0], pos[1], pos[2]));
        collision_objects[index]->setQuatRotation(fcl::Quaternion<float>(q[0], q[1], q[2], q[3]));

        printf("previous time: %f\n", times(index));
        printf("current time: %f\n", curr_time);
        printf("Previous position: %f %f %f\n", prev_position[0], prev_position[1], prev_position[2]);
        printf("Current position: %f %f %f\n", pos[0], pos[1], pos[2]);

        // calcualting velocity and saving both it and time of saving
        velocities.row(index) = (collision_objects[index]->getTransform().translation() - prev_position) / (curr_time - times(index));
        printf("velocity : %f %f %f\n", velocities(index, 0), velocities(index, 1), velocities(index, 2));

        times[index] = curr_time;
    }

    Eigen::MatrixX3f Cylinders::get_cylinders_transforms()
    {
        Eigen::MatrixX3f pos_matrix;
        pos_matrix.resize(cylinders.size(), 3);

        for (int i = 0; i < cylinders.size(); i++)
        {
            auto pos = collision_objects[i]->getTranslation();
            pos_matrix.row(i) << pos[0], pos[1], pos[2];
        }

        return pos_matrix;
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

    float Cylinders::get_distance(fcl::CollisionObject<float> *robot)
    {
        // find min distance
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < collision_objects.size(); i++)
        {
            result.clear();
            printf("Node types:%d ,%d\n", robot->getNodeType(), collision_objects[0]->getNodeType());
            fcl::distance(robot, collision_objects[0], distance_request, distance_result);
            min_distance = std::min(min_distance, distance_result.min_distance);
        }

        return min_distance;
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

    int Cylinders::get_cylinders_size()
    {
        // retunr the number of cylinders
        return cylinders.size();
    }

    Eigen::Vector3f Cylinders::get_cylinder_velocity(int id) { return velocities.row(id); }

    Eigen::Vector3f Cylinders::get_cylinder_position(int id) { return collision_objects[id]->getTranslation(); }

} // namespace realtime_obstacles
