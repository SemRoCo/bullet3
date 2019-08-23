IF(BUILD_PYBULLET)
    FIND_PACKAGE(pybind11 REQUIRED)
    INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs )

    SET(bpb_SRCS
        bind_collisions.cpp
        ../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp)    

    PYBIND11_ADD_MODULE(betterpybullet SHARED ${bpb_SRCS} ${Root_HDRS})
    TARGET_LINK_LIBRARIES(betterpybullet PRIVATE BulletCollision BulletDynamics Bullet3Common LinearMath)

ENDIF(BUILD_PYBULLET)