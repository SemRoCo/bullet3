#include "kineverse_query.h"

ContactPair::ContactPair() { }

ContactPair::ContactPair(std::shared_ptr<const KineverseCollisionObject> obj_a,
                         std::shared_ptr<const KineverseCollisionObject> obj_b) 
: m_obj_a(obj_a)
, m_obj_b(obj_b) {}  

bool ContactPair::needsCollision(btBroadphaseProxy* proxy0) const { 
    return true; 
}

btScalar ContactPair::addSingleResult(btManifoldPoint& cp, 
                                      const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                                      const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    m_points.push_back(ContactPoint(cp.m_localPointA,
                                    cp.m_localPointB,
                                    cp.m_normalWorldOnB, 
                                    cp.getDistance()));

    return 1;
}

ClosestPair::ClosestPair()
: ContactPair() { }

ClosestPair::ClosestPair(std::shared_ptr<const KineverseCollisionObject> obj_a,
                         std::shared_ptr<const KineverseCollisionObject> obj_b) 
: ContactPair(obj_a, obj_b) { }


btScalar ClosestPair::addSingleResult(btManifoldPoint& cp, 
                                      const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                                      const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    if (m_points.size() == 0) {
        return ContactPair::addSingleResult(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1);
    } else if (cp.getDistance() < m_points[0].m_distance) {
        m_points[0].m_pointOnA     = cp.m_localPointA;
        m_points[0].m_pointOnB     = cp.m_localPointB;
        m_points[0].m_normalWorldB = cp.m_normalWorldOnB; 
        m_points[0].m_distance     = cp.getDistance();
    }

    return 1;
}
