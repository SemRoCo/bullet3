#pragma once

#include <memory>
#include <vector>

#include "BulletCollision/CollisionShapes/btCompoundShape.h"


class KineverseCompoundShape : public btCompoundShape {

 public:

    void addChildShape(const btTransform& localTransform, std::shared_ptr<btCollisionShape> shape);

    /// Remove all children shapes that contain the specified shape
    virtual void removeChildShape(std::shared_ptr<btCollisionShape> shape);

    void removeChildShapeByIndex(int childShapeindex);

    inline std::shared_ptr<btCollisionShape> getChildShape(int index) {
        return m_shape_ptrs[index];
    }
    
    inline std::shared_ptr<const btCollisionShape> getChildShape(int index) const {
        return std::const_pointer_cast<const btCollisionShape>(m_shape_ptrs[index]);
    }


    inline std::vector<std::shared_ptr<btCollisionShape>> get_child_shapes() {
        return m_shape_ptrs;
    }

 private:
    std::vector<std::shared_ptr<btCollisionShape>> m_shape_ptrs;
};
