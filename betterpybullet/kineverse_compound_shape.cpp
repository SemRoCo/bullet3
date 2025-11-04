#include "kineverse_compound_shape.h"


void KineverseCompoundShape::addChildShape(const btTransform& localTransform, std::shared_ptr<btCollisionShape> shape) {
    btCompoundShape::addChildShape(localTransform, shape.get());
    m_shape_ptrs.push_back(shape);
}

/// Remove all children shapes that contain the specified shape
void KineverseCompoundShape::removeChildShape(std::shared_ptr<btCollisionShape> shape) {
    btCompoundShape::removeChildShape(shape.get());
    auto it = m_shape_ptrs.begin();
    while(it != m_shape_ptrs.end()) {
        if (*it == shape)
            m_shape_ptrs.erase(it);
        else
            it++;
    }
}

void KineverseCompoundShape::removeChildShapeByIndex(int childShapeindex) {
    btCompoundShape::removeChildShapeByIndex(childShapeindex);
    m_shape_ptrs.erase(m_shape_ptrs.begin() + childShapeindex);
}
