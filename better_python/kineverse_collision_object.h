#pragma once

#include <memory>

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class KineverseCollisionObject : public btCollisionObject {

 public:
    virtual void setCollisionShape(btCollisionShape* collisionShape);
    virtual void setCollisionShape(std::shared_ptr<btCollisionShape> collisionShape);

    inline std::shared_ptr<btCollisionShape> getCollisionShape() const {
        return m_collision_shape_ptr;
    }

 private:
    std::shared_ptr<btCollisionShape> m_collision_shape_ptr;
};
