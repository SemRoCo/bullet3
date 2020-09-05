#pragma once
#include <Wavefront/tiny_obj_loader.h>

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "kineverse_settings.h"

class btCollisionShape;

tinyobj::attrib_t load_stl_mesh(const std::string& filename);

std::shared_ptr<btCollisionShape> load_convex_shape(std::string filename,
                                                    bool single_shape = true,
                                                    bool use_cache = true,
                                                    btScalar shape_margin = KINEVERSE_COLLISION_MARGIN,
                                                    btVector3 scaling = btVector3(1, 1, 1));
// std::shared_ptr<btCollisionShape> load_trimesh_shape(std::string filename, bool use_cache = true);

std::pair<std::string, btVector3> get_shape_filename_and_scale(const btCollisionShape* shape);
