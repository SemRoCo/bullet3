#include "kineverse_collision_object.h"


void KineverseCollisionObject::setCollisionShape(btCollisionShape* collisionShape) {
    throw std::runtime_error("DO NOT SET COLLISION SHAPES OF KINEVERSE OBJECTS USING RAW-POINTERS.");
}

void KineverseCollisionObject::setCollisionShape(std::shared_ptr<btCollisionShape> collisionShape) {
    btCollisionObject::setCollisionShape(collisionShape.get());

    m_collision_shape_ptr = collisionShape;
}
