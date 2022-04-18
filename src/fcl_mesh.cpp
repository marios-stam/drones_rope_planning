#include "../include/ompl_example_2d/fcl_mesh.hpp"
#include "ros/ros.h"

int get_index_of_vert(std::vector<stlloader::Vertex> &verts, stlloader::Vertex &vert)
{
    for (int i = 0; i < verts.size(); ++i)
    {
        if (stlloader::vertices_equal(verts[i], vert))
        {
            return i;
        }
    }
    return -1;
}

namespace fcl_checking
{
    fcl_mesh::fcl_mesh() {}

    fcl_mesh::~fcl_mesh() {}

    void fcl_mesh::load_stl(std::string filename)
    {
        stlloader::parse_file(filename.c_str(), fcl_mesh::stl_mesh);

        std::vector<stlloader::Vertex> uniq_verts = get_unique_vertices();

        int index;
        int tris[stl_mesh.facets.size()][3];
        int i = 0;
        for (stlloader::Facet facet : fcl_mesh::stl_mesh.facets)
        {
            for (int vi = 0; vi < 3; ++vi)
            {
                stlloader::Vertex &coord = facet.vertices[vi]; // x, y, z
                index = get_index_of_vert(uniq_verts, coord);
                tris[i][vi] = index;
            }
            i++;
        }

        // print tris
        // for (int i = 0; i < stl_mesh.facets.size(); ++i)
        // {
        //     printf("%d %d %d\n", tris[i][0], tris[i][1], tris[i][2]);
        // }

        // create bvh model
        mesh = new fcl::BVHModel<BVH_TYPE>();
        std::vector<fcl::Vector3<float>> fcl_vertices;
        std::vector<fcl::Triangle> fcl_tris;

        for (auto &vert : uniq_verts)
        {
            fcl_vertices.push_back(fcl::Vector3<float>(vert.x, vert.y, vert.z));
        }

        for (int i = 0; i < stl_mesh.facets.size(); ++i)
        {
            fcl::Triangle fcl_tri(tris[i][0], tris[i][1], tris[i][2]);
            fcl_tris.push_back(fcl_tri);
        }
        mesh->beginModel(stl_mesh.facets.size(), uniq_verts.size());
        mesh->addSubModel(fcl_vertices, fcl_tris);
        mesh->endModel();

        printf("Loaded STL file: %s\n", filename.c_str());
    }

    std::vector<stlloader::Vertex> fcl_mesh::get_unique_vertices(void)
    {
        // throw all in a vector
        std::vector<stlloader::Vertex> vertices;
        for (stlloader::Facet facet : fcl_mesh::stl_mesh.facets)
        {
            for (int vi = 0; vi < 3; ++vi)
            {
                stlloader::Vertex &coord = facet.vertices[vi]; // x, y, z
                vertices.push_back(coord);
            }
        }

        std::vector<stlloader::Vertex> unique_vertices;
        // get unique vertices
        // std::cout << "Vertices: " << vertices.size() << std::endl;
        std::sort(vertices.begin(), vertices.end(), stlloader::vertices_smaller);

        for (int i = 0; i < vertices.size(); ++i)
        {
            if (i == 0 || !stlloader::vertices_equal(vertices[i], vertices[i - 1]))
            {
                unique_vertices.push_back(vertices[i]);
            }
        }

        // std::cout << "Unique vertices: " << unique_vertices.size() << std::endl;
        return unique_vertices;
    }

    void fcl_mesh::create_collision_object(void)
    {
        // create collision object
        collision_object = new fcl::CollisionObject<float>(std::shared_ptr<fcl::CollisionGeometry<float>>(mesh));
        collision_object->setIdentityTransform();
    }

    void fcl_mesh::set_transform(float pos[], float quat[])
    {

        // fc::Quaternion expects w, x, y, z
        fcl::Quaternion<float> q = fcl::Quaternion<float>(quat[3], quat[0], quat[1], quat[2]);

        // printf("set_transform: %f %f %f %f\n", pos[0], pos[1], pos[2], quat[3]);
        collision_object->setTranslation(fcl::Vector3f(pos[0], pos[1], pos[2]));
        // printf("setting rotation\n");

        collision_object->setQuatRotation(q);
    }

    void fcl_mesh::create_mesh(Eigen::MatrixXf verts, Eigen::MatrixXf tris)
    {
        // transform Eigen matrix to fcl matrix
        std::vector<fcl::Vector3<float>> fcl_vertices;
        std::vector<fcl::Triangle> fcl_tris;

        for (int i = 0; i < verts.rows(); ++i)
        {
            fcl::Vector3<float> fcl_vert(verts(i, 0), verts(i, 1), verts(i, 2));
            fcl_vertices.push_back(fcl_vert);
        }

        for (int i = 0; i < tris.rows(); ++i)
        {
            fcl::Triangle fcl_tri(tris(i, 0), tris(i, 1), tris(i, 2));
            fcl_tris.push_back(fcl_tri);
        }

        mesh = new fcl::BVHModel<BVH_TYPE>();

        auto t0 = ros::Time::now();
        mesh->beginModel(fcl_tris.size(), fcl_vertices.size());
        mesh->addSubModel(fcl_vertices, fcl_tris);
        mesh->endModel();
        auto dt = ros::Time::now() - t0;
    }

    void fcl_mesh::update_mesh(const std::vector<fcl::Vector3<float>> new_verts)
    {
        mesh->beginUpdateModel();
        mesh->updateSubModel(new_verts);
        mesh->endUpdateModel();

        create_collision_object();
    }

    void fcl_mesh::update_mesh(const Eigen::MatrixXf new_verts)
    {
        std::vector<fcl::Vector3<float>> fcl_vertices;
        for (int i = 0; i < new_verts.rows(); ++i)
        {
            fcl::Vector3<float> fcl_vert(new_verts(i, 0), new_verts(i, 1), new_verts(i, 2));
            fcl_vertices.push_back(fcl_vert);
        }

        bool refit = true; // if true -> doesn't rebuild the whole BVH tree

        // top-down way (slow but more compact)
        // bottom-up way (fast but less compact)
        bool bottom_up = true;

        // mesh->beginUpdateModel();
        // mesh->updateSubModel(fcl_vertices);
        // mesh->endUpdateModel(refit, bottom_up);

        mesh->beginReplaceModel();
        mesh->replaceSubModel(fcl_vertices);
        mesh->endReplaceModel(refit, bottom_up);
    }

    fcl::BVHModel<BVH_TYPE> *fcl_mesh::get_fcl_mesh(void) { return mesh; }
}
