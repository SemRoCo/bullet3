IF(BUILD_PYBULLET)
    FIND_PACKAGE(pybind11 REQUIRED)
    INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs ${BULLET_PHYSICS_SOURCE_DIR}/examples/ ${PYTHON_INCLUDE_DIR})

    add_library(kineverse SHARED
                kineverse_utils.cpp
                kineverse_mesh_loader.cpp
                kineverse_query.cpp
                kineverse_world.cpp
                kineverse_collision_object.cpp
                kineverse_compound_shape.cpp
                ../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
                ../examples/Utils/b3ResourcePath.cpp)
    target_link_libraries(kineverse BulletCollision BulletDynamics Bullet3Common LinearMath)

    add_executable(kineverse_test kineverse_test.cpp)
    target_link_libraries(kineverse_test kineverse)

    SET(bpb_SRCS
        bind_collisions.cpp)

    PYBIND11_ADD_MODULE(betterpybullet SHARED ${bpb_SRCS} ${Root_HDRS})
    TARGET_LINK_LIBRARIES(betterpybullet PRIVATE kineverse)# BulletCollision BulletDynamics Bullet3Common LinearMath)

ENDIF(BUILD_PYBULLET)
