#include <algorithm>
#include <unordered_set>

#include <btBulletCollisionCommon.h>
#include <Utils/b3BulletDefaultFileIO.h>

#include "kineverse_mesh_loader.h"
#include "kineverse_compound_shape.h"
#include "kineverse_bvh_triangle_mesh.h"
#include "kineverse_utils.h"

tinyobj::attrib_t load_stl_mesh(const std::string& filename) {
    FILE* file = fopen(filename.c_str(),"rb");
    tinyobj::attrib_t out;
    if (file) {
        int size=0;
        if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {
            throw std::runtime_error(string_format("Error: Cannot access file to determine size of %s\n", filename.c_str()));
        } else if (size) {
            //b3Warning("Open STL file of %d bytes\n",size);
            char* memoryBuffer = new char[size + 1];
            int actualBytesRead = fread(memoryBuffer, 1, size, file);
            if (actualBytesRead != size) {
                delete[] memoryBuffer;
                throw std::runtime_error(string_format("Error reading from file %s", filename.c_str()));
            } else {
                int numTriangles = *(int*)&memoryBuffer[80];
                
                if (numTriangles) {
                    //perform a sanity check instead of crashing on invalid triangles/STL files
                    int expectedBinaryFileSize = numTriangles * 50 + 84;
                    if (expectedBinaryFileSize != size) {
                        delete[] memoryBuffer;
                        throw std::runtime_error(string_format("STL file has wrong size! Expected size is %d actual size is %d", expectedBinaryFileSize, size));
                    }

                    out.vertices.reserve(numTriangles * 9);
                    out.normals.reserve(numTriangles  * 3);
                    for (int i = 0; i < numTriangles; i++) {
                        char*     curPtr = &memoryBuffer[84 + i * 50];
                        float* float_ptr = (float*)curPtr;
                        //MySTLTriangle tmp;
                        //memcpy(&tmp,curPtr,sizeof(MySTLTriangle));
                        
                        for (int v = 0; v < 3; v++)
                            out.normals.push_back(*(float_ptr + v));

                        for (int v = 3; v < 12; v++)
                            out.vertices.push_back(*(float_ptr + v));
                    }
                }
            }
            
            delete[] memoryBuffer;
        }
        fclose(file);
    }
    return out;
}

std::vector<std::shared_ptr<btCollisionShape>> m_flat_shape_cache;
std::unordered_map<std::string, std::shared_ptr<btCollisionShape>> m_convex_shape_cache;
std::unordered_map<std::string, std::shared_ptr<btCollisionShape>> m_trimesh_shape_cache;
std::unordered_map<const btCollisionShape*, std::string> collision_shape_filenames;

std::shared_ptr<btCollisionShape> load_convex_shape(std::string filename, bool use_cache, btScalar shape_margin) {
    if (use_cache && m_convex_shape_cache.find(filename) != m_convex_shape_cache.end())
        return m_convex_shape_cache[filename];


    std::string lc_filename = filename;
    std::transform(lc_filename.begin(), lc_filename.end(), lc_filename.begin(), [](unsigned char c) { return std::tolower(c); });

    std::shared_ptr<btCollisionShape> shape_ptr;
    if (lc_filename.substr(lc_filename.size() - 4) == ".obj") {
        b3BulletDefaultFileIO file_io;
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::string error_msg = tinyobj::LoadObj(attrib, shapes, filename.c_str(), 0, &file_io);

        for (const auto& shape : shapes) {
            auto new_shape = std::make_shared<btConvexHullShape>();
            new_shape->setMargin(shape_margin);
            
            std::unordered_set<int> used_indices;

            for (int i = 0; i < shape.mesh.indices.size(); i++) {
                if (used_indices.find(shape.mesh.indices[i].vertex_index) == used_indices.end()) {
                    new_shape->addPoint(btVector3(attrib.vertices[shape.mesh.indices[i].vertex_index * 3 + 0], 
                                                  attrib.vertices[shape.mesh.indices[i].vertex_index * 3 + 1], 
                                                  attrib.vertices[shape.mesh.indices[i].vertex_index * 3 + 2]));
                }
                used_indices.insert(shape.mesh.indices[i].vertex_index);
            }
            
            new_shape->recalcLocalAabb();
            new_shape->optimizeConvexHull();
            m_flat_shape_cache.push_back(new_shape);
        }
        
        if (shapes.size() == 1) {
            shape_ptr = m_flat_shape_cache[m_flat_shape_cache.size() - 1];
        } else {
            auto compound_ptr = std::make_shared<KineverseCompoundShape>();
            for (int i = m_flat_shape_cache.size() - shapes.size(); i < m_flat_shape_cache.size(); i++)
                compound_ptr->addChildShape(btTransform::getIdentity(), m_flat_shape_cache[i]);
            shape_ptr = compound_ptr;
        }
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".stl") {
        auto attrib = load_stl_mesh(filename);
        auto hull_ptr = std::make_shared<btConvexHullShape>();
        hull_ptr->setMargin(shape_margin);

        for (int i = 0; i < attrib.vertices.size(); i += 3) 
            hull_ptr->addPoint(btVector3(attrib.vertices[i + 0], 
                                         attrib.vertices[i + 1], 
                                         attrib.vertices[i + 2]));
        hull_ptr->recalcLocalAabb();
        hull_ptr->optimizeConvexHull();
        shape_ptr = hull_ptr;
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".dae") {
        throw std::runtime_error(string_format(".dae export is not yet supported. File: %s", filename).c_str());
    } else {
        throw std::runtime_error(string_format("File '%s' is not .obj, .stl, or .dae", filename.c_str()));
    }

    m_convex_shape_cache[filename] = shape_ptr;
    collision_shape_filenames[shape_ptr.get()] = filename;
    return shape_ptr;
}

std::shared_ptr<btCollisionShape> load_trimesh_shape(std::string filename, bool use_cache) {
    if (use_cache && m_trimesh_shape_cache.find(filename) != m_trimesh_shape_cache.end())
        return m_trimesh_shape_cache[filename];


    std::string lc_filename = filename;
    std::transform(lc_filename.begin(), lc_filename.end(), lc_filename.begin(), [](unsigned char c) { return std::tolower(c); });

    std::shared_ptr<btCollisionShape> shape_ptr;
    if (lc_filename.substr(lc_filename.size() - 4) == ".obj") {
        b3BulletDefaultFileIO file_io;
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::string error_msg = tinyobj::LoadObj(attrib, shapes, filename.c_str(), 0, &file_io);
        
        for (const auto& shape : shapes) {
            auto mesh_interface = std::make_shared<btTriangleMesh>();

            for (int i = 0; i < shape.mesh.indices.size(); i += 3) {
                mesh_interface->addTriangle(btVector3(attrib.vertices[shape.mesh.indices[i].vertex_index + 0], 
                                                      attrib.vertices[shape.mesh.indices[i].vertex_index + 1], 
                                                      attrib.vertices[shape.mesh.indices[i].vertex_index + 2]),
                                            btVector3(attrib.vertices[shape.mesh.indices[i + 1].vertex_index + 0], 
                                                      attrib.vertices[shape.mesh.indices[i + 1].vertex_index + 1], 
                                                      attrib.vertices[shape.mesh.indices[i + 1].vertex_index + 2]),
                                            btVector3(attrib.vertices[shape.mesh.indices[i + 2].vertex_index + 0], 
                                                      attrib.vertices[shape.mesh.indices[i + 2].vertex_index + 1], 
                                                      attrib.vertices[shape.mesh.indices[i + 2].vertex_index + 2]));
            }
            
            auto new_shape = std::make_shared<KineverseBvhTriangleMeshShape>(mesh_interface, true, true);
            m_flat_shape_cache.push_back(new_shape);
        }
        
        if (shapes.size() == 1) {
            shape_ptr = m_flat_shape_cache[m_flat_shape_cache.size() - 1];
        } else {
            auto compound_ptr = std::make_shared<KineverseCompoundShape>();
            for (int i = m_flat_shape_cache.size() - shapes.size(); i < m_flat_shape_cache.size(); i++)
                compound_ptr->addChildShape(btTransform::getIdentity(), m_flat_shape_cache[i]);
            shape_ptr = compound_ptr;
        }
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".stl") {
        auto attrib = load_stl_mesh(filename);
        auto mesh_interface = std::make_shared<btTriangleMesh>();

        for (int i = 0; i < attrib.vertices.size(); i += 9) 
            mesh_interface->addTriangle(btVector3(attrib.vertices[i + 0], 
                                                  attrib.vertices[i + 1], 
                                                  attrib.vertices[i + 2]),
                                        btVector3(attrib.vertices[i + 3], 
                                                  attrib.vertices[i + 4], 
                                                  attrib.vertices[i + 5]),
                                        btVector3(attrib.vertices[i + 6], 
                                                  attrib.vertices[i + 7], 
                                                  attrib.vertices[i + 8]));
        shape_ptr = std::make_shared<KineverseBvhTriangleMeshShape>(mesh_interface, true, true);
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".dae") {
        throw std::runtime_error(string_format(".dae export is not yet supported. File: %s", filename).c_str());
    } else {
        throw std::runtime_error(string_format("File '%s' is not .obj, .stl, or .dae", filename.c_str()));
    }

    m_trimesh_shape_cache[filename] = shape_ptr;
    collision_shape_filenames[shape_ptr.get()] = filename;
    return shape_ptr;
}

std::string get_shape_filename(const btCollisionShape* shape) {
    if (collision_shape_filenames.find(shape) != collision_shape_filenames.end())
        return collision_shape_filenames[shape];
    return "";
}
