#pragma once
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <btBulletCollisionCommon.h>

#include "kineverse_query.h"
#include "kineverse_collision_object.h"

typedef std::shared_ptr<btDispatcher>             Dispatcher_ptr;
typedef std::shared_ptr<btBroadphaseInterface>    Broadphase_ptr;
typedef std::shared_ptr<btCollisionConfiguration> CollisionConfiguration_ptr;

typedef std::shared_ptr<KineverseCollisionObject> CollisionObjectPtr;

class KineverseWorld : public btCollisionWorld {
private:
    btDbvtBroadphase         m_bpinterface;
    btDefaultCollisionConfiguration m_collision_configuration;
    std::shared_ptr<btDispatcher> m_dispatcher;

    std::unordered_set<CollisionObjectPtr> m_collision_object_set;
    std::unordered_map<const btCollisionObject*, CollisionObjectPtr> m_collision_object_ptr_map;

public:
    KineverseWorld();

    virtual ~KineverseWorld();

    template<typename T>
    T closest_ray_test(const btVector3& from, const btVector3& to) {
        T out(from, to);
        rayTest(from, to, out);
        return out;
    }

    template<typename T>
    std::vector<T> closest_ray_batch_test(const std::vector<btVector3>& from, const std::vector<btVector3>& to) {
        std::vector<T> out;
        out.reserve(from.size());
        for (int i = 0; i < from.size(); i++) {
            out.push_back(T(from[i], to[i]));
            rayTest(from[i], to[i], out[i]);
        }
        return out;
    }

    inline auto& get_object_ptr_map() { return m_collision_object_ptr_map; }

    void addCollisionObject(btCollisionObject* collisionObject, 
                            int collisionFilterGroup=btBroadphaseProxy::DefaultFilter, 
                            int collisionFilterMask=btBroadphaseProxy::AllFilter);

    void addCollisionObject(CollisionObjectPtr collisionObject, 
                            int collisionFilterGroup=btBroadphaseProxy::DefaultFilter, 
                            int collisionFilterMask=btBroadphaseProxy::AllFilter);

    void removeCollisionObject(btCollisionObject* collisionObject);

    void removeCollisionObject(CollisionObjectPtr collisionObject);

    std::vector<ContactPair> get_contacts();

    std::vector<ContactPoint> get_distance(CollisionObjectPtr obj_a, CollisionObjectPtr obj_b, btScalar max_distance = 1.0);

    std::vector<ClosestPair> get_closest(CollisionObjectPtr obj, btScalar max_distance=1.f);

    std::unordered_map<CollisionObjectPtr, std::vector<ClosestPair>> get_closest_batch(std::unordered_map<CollisionObjectPtr, btScalar> params);

    std::vector<ClosestPair> get_closest_filtered(CollisionObjectPtr obj, const std::vector<CollisionObjectPtr>& other_objects, btScalar max_distance = 1.0);

    std::unordered_map<CollisionObjectPtr, std::vector<ClosestPair>> get_closest_filtered_batch(const std::unordered_map<CollisionObjectPtr, std::pair<std::vector<CollisionObjectPtr>, btScalar>>& query);
    
    std::vector<ClosestPair> get_closest_filtered_POD(CollisionObjectPtr obj, const std::vector<std::pair<CollisionObjectPtr, btScalar>>& other_objects);

    std::unordered_map<CollisionObjectPtr, std::vector<ClosestPair>> get_closest_filtered_POD_batch(const std::unordered_map<CollisionObjectPtr, std::vector<std::pair<CollisionObjectPtr, btScalar>>>& query);

    std::vector<CollisionObjectPtr> overlap_aabb(const btVector3& aabb_min, const btVector3& aabb_max);

    inline std::vector<CollisionObjectPtr> get_collision_objects() const { 
        return std::vector<CollisionObjectPtr>(m_collision_object_set.begin(), m_collision_object_set.end()); 
    }
};
