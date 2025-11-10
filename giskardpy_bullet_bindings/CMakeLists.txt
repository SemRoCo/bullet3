IF(BUILD_PYBULLET)
    include(FetchContent)

    FetchContent_Declare(
        pybind11
        URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
        URL_HASH SHA256=e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20
    )
    FetchContent_MakeAvailable(pybind11)

    INCLUDE_DIRECTORIES( 
        ${BULLET_PHYSICS_SOURCE_DIR}/src/
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ 
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/ 
        ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ 
        ${BULLET_PHYSICS_SOURCE_DIR}/Extras/VHACD/inc/ 
        ${BULLET_PHYSICS_SOURCE_DIR}/Extras/VHACD/public/
        ${PYTHON_INCLUDE_DIR}
    )

    add_library(kineverse SHARED
                kineverse_utils.cpp
                kineverse_mesh_loader.cpp
                # kineverse_query.cpp
                kineverse_world.cpp
                kineverse_collision_object.cpp
                kineverse_compound_shape.cpp
                kineverse_bvh_triangle_mesh.cpp
                ../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
                ../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
                ../examples/Utils/b3ResourcePath.cpp
                ../Extras/VHACD/src/VHACD.cpp
                ../Extras/VHACD/src/vhacdMesh.cpp
                ../Extras/VHACD/src/vhacdICHull.cpp
                ../Extras/VHACD/src/vhacdManifoldMesh.cpp
                ../Extras/VHACD/src/vhacdVolume.cpp
                ../examples/SharedMemory/PhysicsClientC_API.cpp
                ../examples/Utils/b3Clock.cpp
                ../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp)
    target_include_directories(kineverse PRIVATE ${pybind11_INCLUDE_DIRS})
    target_link_libraries(kineverse PUBLIC BulletCollision BulletDynamics Bullet3Common LinearMath)

#     add_executable(kineverse_test kineverse_test.cpp)
#     target_link_libraries(kineverse_test kineverse)

    # add_executable(kineverse_test2 kineverse_test2.cpp)
    # target_link_libraries(kineverse_test2 kineverse)

    # add_executable(no_pybullet_dist_query no_pybullet_dist_query.cpp)
    # target_link_libraries(no_pybullet_dist_query kineverse BulletRobotics)

    # add_executable(bpb_pybullet_comparison bpb_pybullet_comparison.cpp)
    # target_link_libraries(bpb_pybullet_comparison kineverse BulletRobotics)


    SET(bpb_SRCS
        bind_collisions.cpp)

    PYBIND11_ADD_MODULE(_giskardpy_bullet_bindings SHARED ${bpb_SRCS} ${Root_HDRS})
    TARGET_LINK_LIBRARIES(_giskardpy_bullet_bindings PRIVATE kineverse)# BulletCollision BulletDynamics Bullet3Common LinearMath)

    SET_TARGET_PROPERTIES(_giskardpy_bullet_bindings PROPERTIES INSTALL_RPATH "\$ORIGIN/lib")
    SET_TARGET_PROPERTIES(kineverse PROPERTIES INSTALL_RPATH $ORIGIN)

    INSTALL(TARGETS kineverse DESTINATION "lib")
    INSTALL(TARGETS _giskardpy_bullet_bindings DESTINATION .)

ENDIF(BUILD_PYBULLET)
