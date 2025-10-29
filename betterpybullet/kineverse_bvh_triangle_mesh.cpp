#include "kineverse_bvh_triangle_mesh.h"

KineverseBvhTriangleMeshShape::KineverseBvhTriangleMeshShape(std::shared_ptr<btStridingMeshInterface> meshInterface,
                                                             bool useQuantizedAabbCompression, 
                                                             bool buildBvh)
: btBvhTriangleMeshShape(meshInterface.get(),
                         useQuantizedAabbCompression,
                         buildBvh)
, m_mesh_interface_ptr(meshInterface) {}

KineverseBvhTriangleMeshShape::KineverseBvhTriangleMeshShape(std::shared_ptr<btStridingMeshInterface> meshInterface, 
                                                             bool useQuantizedAabbCompression, 
                                                             const btVector3& bvhAabbMin, 
                                                             const btVector3& bvhAabbMax, 
                                                             bool buildBvh)
: btBvhTriangleMeshShape(meshInterface.get(), 
                         useQuantizedAabbCompression,
                         bvhAabbMin,
                         bvhAabbMax,
                         buildBvh)
, m_mesh_interface_ptr(meshInterface) {}
