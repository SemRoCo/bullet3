#pragma once

#include <memory>
#include <vector>

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"


class KineverseBvhTriangleMeshShape : public btBvhTriangleMeshShape {
 public:
    KineverseBvhTriangleMeshShape(std::shared_ptr<btStridingMeshInterface> meshInterface, 
                                  bool useQuantizedAabbCompression, 
                                  bool buildBvh = true);

    KineverseBvhTriangleMeshShape(std::shared_ptr<btStridingMeshInterface> meshInterface, 
                                  bool useQuantizedAabbCompression, 
                                  const btVector3& bvhAabbMin, 
                                  const btVector3& bvhAabbMax, 
                                  bool buildBvh = true);

 private:
    std::shared_ptr<btStridingMeshInterface> m_mesh_interface_ptr;
};
