#include <algorithm>

#include <btBulletCollisionCommon.h>

#include "kineverse_mesh_loader.h"
#include "kineverse_utils.h"

std::shared_ptr<tinyobj::mesh_t> load_stl_mesh(const std::string& filename) {
    FILE* file = fopen(filename.c_str(),"rb");
    std::shared_ptr<tinyobj::mesh_t> out(new tinyobj::mesh_t());
    if (file) {
        int size=0;
        if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {
            throw std::runtime_error(string_format("Error: Cannot access file to determine size of %s\n", filename.c_str()));
        } else if (size) {
            //b3Warning("Open STL file of %d bytes\n",size);
            char* memoryBuffer = new char[size+1];
            int actualBytesRead = fread(memoryBuffer,1,size,file);
            if (actualBytesRead!=size) {
                throw std::runtime_error(string_format("Error reading from file %s", filename.c_str()));
            } else {
                int numTriangles = *(int*)&memoryBuffer[80];
                
                if (numTriangles) {
                    //perform a sanity check instead of crashing on invalid triangles/STL files
                    int expectedBinaryFileSize = numTriangles* 50 + 84;
                    if (expectedBinaryFileSize != size) {
                        delete[] memoryBuffer;
                        throw std::runtime_error(string_format("STL file has wrong size! Expected size is %d actual size is %d", expectedBinaryFileSize, size));
                    }

                    out->positions.reserve(numTriangles * 3 * 3);
                    out->normals.reserve(numTriangles * 3);
                    for (int i=0;i<numTriangles;i++) {
                        char* curPtr = &memoryBuffer[84+i*50];
                        float* float_ptr = (float*)curPtr;
                        //MySTLTriangle tmp;
                        //memcpy(&tmp,curPtr,sizeof(MySTLTriangle));
                        
                        for (int v=0; v<3; v++)
                            out->normals.push_back(*(float_ptr + v));

                        for (int v=3; v < 12; v++)
                            out->positions.push_back(*(float_ptr + v));
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
std::unordered_map<const btCollisionShape*, std::string> collision_shape_filenames;

std::shared_ptr<btCollisionShape> load_convex_shape(std::string filename, bool use_cache) {
    if (use_cache && m_convex_shape_cache.find(filename) != m_convex_shape_cache.end())
        return m_convex_shape_cache[filename];


    std::string lc_filename = filename;
    std::transform(lc_filename.begin(), lc_filename.end(), lc_filename.begin(), [](unsigned char c) { return std::tolower(c); });

    std::shared_ptr<btCollisionShape> shape_ptr;
    if (lc_filename.substr(lc_filename.size() - 4) == ".obj") {
        std::vector<tinyobj::shape_t> shapes;
        std::string error_msg = tinyobj::LoadObj(shapes, filename.c_str());
        for (tinyobj::shape_t& shape : shapes) {
            std::shared_ptr<btConvexHullShape> new_shape(new btConvexHullShape());
            for (int i = 0; i < shape.mesh.positions.size(); i += 3) 
                new_shape->addPoint(btVector3(shape.mesh.positions[i], shape.mesh.positions[i + 1], shape.mesh.positions[i + 2]), i == shape.mesh.positions.size() - 3);
            new_shape->optimizeConvexHull();
            m_flat_shape_cache.push_back(new_shape);
        }
        
        if (shapes.size() == 1) {
            shape_ptr = m_flat_shape_cache[m_flat_shape_cache.size() - 1];
        } else {
            std::shared_ptr<btCompoundShape> compound_ptr(new btCompoundShape());
            for (int i = m_flat_shape_cache.size() - shapes.size(); i < m_flat_shape_cache.size(); i++)
                compound_ptr->addChildShape(btTransform::getIdentity(), m_flat_shape_cache[i].get());
            shape_ptr = compound_ptr;
        }
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".stl") {
        auto stl_mesh = load_stl_mesh(filename);
        std::shared_ptr<btConvexHullShape> hull_ptr(new btConvexHullShape());
        for (int i = 0; i < stl_mesh->positions.size(); i += 3) 
            hull_ptr->addPoint(btVector3(stl_mesh->positions[i], stl_mesh->positions[i + 1], stl_mesh->positions[i + 2]), i == stl_mesh->positions.size() - 3);
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

std::string get_shape_filename(const btCollisionShape* shape) {
    if (collision_shape_filenames.find(shape) != collision_shape_filenames.end())
        return collision_shape_filenames[shape];
    return "";
}
