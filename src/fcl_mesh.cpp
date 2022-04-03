#include "../include/ompl_example_2d/fcl_mesh.hpp"

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
    fcl_mesh::fcl_mesh() { printf("FCL mesh called\n"); }

    fcl_mesh::~fcl_mesh() { printf("FCL mesh destructor called\n"); }

    void fcl_mesh::load_stl(std::string filename)
    {
        printf("Loading STL file: %s\n", filename.c_str());
        const char *filename2 = "/home/marios/thesis_ws/src/drones_rope_planning/resources/env-scene.stl";
        stlloader::parse_file(filename2, fcl_mesh::stl_mesh);

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
        mesh = new fcl::BVHModel<fcl::OBBRSS<float>>();
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
        std::cout << "Vertices: " << vertices.size() << std::endl;
        std::sort(vertices.begin(), vertices.end(), stlloader::vertices_smaller);

        for (int i = 0; i < vertices.size(); ++i)
        {
            if (i == 0 || !stlloader::vertices_equal(vertices[i], vertices[i - 1]))
            {
                unique_vertices.push_back(vertices[i]);
            }
        }

        std::cout << "Unique vertices: " << unique_vertices.size() << std::endl;
        return unique_vertices;
    }

    void fcl_mesh::create_collision_object(void)
    {
        // create collision object
        collision_object = new fcl::CollisionObject<float>(std::shared_ptr<fcl::CollisionGeometry<float>>(mesh));
    }

    void fcl_mesh::set_transform(float pos[], float quat[])
    {

        // set transform
        fcl::Vector3<float> trans;
        trans.data()[0] = pos[0];
        trans.data()[1] = pos[2];
        trans.data()[3] = pos[3];

        fcl::Quaternion<float> q = fcl::Quaternion<float>(quat[0], quat[1], quat[2], quat[3]);
        collision_object->setTransform(q, trans);
    }

    void fcl_mesh::update_mesh(const std::vector<fcl::Vector3<float>> &new_verts)
    {
        mesh->beginUpdateModel();
        mesh->updateSubModel(new_verts);
        mesh->endUpdateModel();
    }
}

int main(int argc, char **argv)
{
    printf("Hello World\n");

    fcl_checking::fcl_mesh fcl_mesh_object;
    fcl_mesh_object.load_stl("/home/marios/thesis_ws/src/drones_rope_planning/resources/env-scene.stl");

    return 0;
}
