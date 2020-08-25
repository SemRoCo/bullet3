#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include <btBulletCollisionCommon.h>

#include "kineverse_collision_object.h"


struct ContactPoint {
    ContactPoint()
    : m_distance(0.0f) {}

    ContactPoint(const btVector3& pointA, 
                 const btVector3& pointB, 
                 const btVector3& world_normalB, 
                 btScalar distance) 
    : m_pointOnA(pointA)
    , m_pointOnB(pointB)
    , m_normalWorldB(world_normalB)
    , m_distance(distance) {}

    btVector3 m_pointOnA;
    btVector3 m_pointOnB;
    btVector3 m_normalWorldB;
    btScalar m_distance;

    inline bool operator< (const ContactPoint& other) {
        return m_distance < other.m_distance;
    }
};

struct ContactPair : public btCollisionWorld::ContactResultCallback {
    ContactPair();

    ContactPair(std::shared_ptr<const KineverseCollisionObject> obj_a,
                std::shared_ptr<const KineverseCollisionObject> obj_b);  

    virtual ~ContactPair() = default;

    virtual bool needsCollision(btBroadphaseProxy* proxy0) const;
    
    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);

    std::shared_ptr<const KineverseCollisionObject> m_obj_a;
    std::shared_ptr<const KineverseCollisionObject> m_obj_b;
    std::vector<ContactPoint> m_points;
};

struct ClosestPair : public ContactPair {
    ClosestPair();

    ClosestPair(std::shared_ptr<const KineverseCollisionObject> obj_a,
                std::shared_ptr<const KineverseCollisionObject> obj_b);

    virtual ~ClosestPair() = default;

    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);
};


template <typename A>
struct PairAccumulator : public btCollisionWorld::ContactResultCallback {
    PairAccumulator(std::unordered_map<const btCollisionObject*, 
                    std::shared_ptr<KineverseCollisionObject>>& ptr_map)
    : m_ptr_map(ptr_map) {}
    
    PairAccumulator(std::shared_ptr<const KineverseCollisionObject> obj, 
                    std::unordered_map<const btCollisionObject*, std::shared_ptr<KineverseCollisionObject>>& ptr_map) 
    : m_obj(obj)
    , m_ptr_map(ptr_map) {}
    virtual ~PairAccumulator() = default;

    virtual bool needsCollision(btBroadphaseProxy* proxy0) const { return true; }
    
    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
        if (m_obj.get() != colObj0Wrap->m_collisionObject) {
            if (m_obj.get() == colObj1Wrap->m_collisionObject) {
                auto buffer = colObj0Wrap;
                colObj0Wrap = colObj1Wrap;
                colObj1Wrap = buffer;
                auto buffer_pointA = cp.m_localPointA;
                cp.m_localPointA = cp.m_localPointB;
                cp.m_localPointB = buffer_pointA;
                cp.m_normalWorldOnB = -cp.m_normalWorldOnB;
            } else {
                return 0;
            }
        }

        if (m_ptr_map.find(colObj1Wrap->m_collisionObject) == m_ptr_map.end())
            return 1;

        auto obj_ptr = m_ptr_map[colObj1Wrap->m_collisionObject];

        if (m_obj_map.find(obj_ptr) == m_obj_map.end())
            m_obj_map[obj_ptr] = A(m_obj, obj_ptr);


        m_obj_map[obj_ptr].addSingleResult(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1);

        return 1;
    }

    inline int size() {
        return m_obj_map.size();
    }

    std::unordered_map<const btCollisionObject*, std::shared_ptr<KineverseCollisionObject>>& m_ptr_map;
    std::unordered_map<std::shared_ptr<const KineverseCollisionObject>, A> m_obj_map;
    std::shared_ptr<const KineverseCollisionObject> m_obj;
};

struct AABBBroadphaseCallback : public btBroadphaseAabbCallback {
    AABBBroadphaseCallback(std::unordered_map<const btCollisionObject*, std::shared_ptr<KineverseCollisionObject>>& ptr_map) 
    : m_ptr_map(ptr_map) {}

    virtual bool process(const btBroadphaseProxy* proxy) {
        auto it = m_ptr_map.find((btCollisionObject*)proxy->m_clientObject);
        if (it != m_ptr_map.end())
            m_objects.push_back(it->second);
        return true;
    }

    std::unordered_map<const btCollisionObject*, std::shared_ptr<KineverseCollisionObject>>& m_ptr_map;
    std::vector<std::shared_ptr<KineverseCollisionObject>> m_objects;
};
