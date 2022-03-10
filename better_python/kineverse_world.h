#pragma once
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <btBulletCollisionCommon.h>

#include "kineverse_query.h"
#include "kineverse_collision_object.h"
#include "kineverse_utils.h"

typedef std::shared_ptr<btDispatcher>             Dispatcher_ptr;
typedef std::shared_ptr<btBroadphaseInterface>    Broadphase_ptr;
typedef std::shared_ptr<btCollisionConfiguration> CollisionConfiguration_ptr;

typedef std::shared_ptr<KineverseCollisionObject> CollisionObjectPtr;

using btContactPoint = ContactPoint<btVector3, btVector3>;
using btContactPair  = ContactPair<btVector3, btVector3>;
using btClosestPair  = ClosestPair<btVector3, btVector3>;

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

    template<typename TResult, typename TPoint>
    std::vector<TResult> get_contacts() {
        btDispatcher* dispatcher = getDispatcher();

        if (!dispatcher) {
            throw std::runtime_error("Can not fetch contacts since dispatcher is NULL.");
        }

        int n_manifolds = dispatcher->getNumManifolds();
        std::vector<TResult> out(n_manifolds);
        //std::cout << "About to iterate over manifolds. Number: " << n_manifolds << std::endl;

        for (int i = 0; i < n_manifolds; i++) {
            //std::cout << "Manifold iteration: " << i << std::endl;
            const btPersistentManifold* manifold = dispatcher->getInternalManifoldPointer()[i];
            //std::cout << "Gotten manifold pointer: " << manifold << std::endl;
            const btCollisionObject* body_a = manifold->getBody0();
            const btCollisionObject* body_b = manifold->getBody1();
            if (!body_a || !body_b) {
                throw std::runtime_error(string_format("At least one of the body pointers is zero. BodyA: %d BodyB: %d", body_a, body_b).c_str());
            }

            auto body_a_ptr = m_collision_object_ptr_map[body_a];
            auto body_b_ptr = m_collision_object_ptr_map[body_b];

            out.push_back(TResult(body_a_ptr, body_b_ptr));
            TResult& pair = out[i];
            //std::cout << "Created contact pair" << std::endl;
            pair.m_points.reserve(manifold->getNumContacts());
            //std::cout << "Reserved contact space. Number of contacts: " << manifold->getNumContacts() << std::endl;
            for (int p = 0; p < manifold->getNumContacts(); p++) {
                //std::cout << "Iterating over contacts. At: " << p << std::endl;
                const btManifoldPoint& pt = manifold->getContactPoint(p);
                pair.m_points.push_back(TPoint(pt.m_localPointA, pt.m_localPointB, pt.m_normalWorldOnB, pt.getDistance()));
            }
        }
        return out;   
    }

    template<typename TResult, typename TPoint>
    std::vector<TPoint> get_distance(CollisionObjectPtr obj_a, CollisionObjectPtr obj_b, btScalar max_distance = 1.0) {
        if (obj_a != obj_b) {
            TResult pair(obj_a, obj_b);
            pair.m_closestDistanceThreshold = max_distance;
            contactPairTest(obj_a.get(), obj_b.get(), pair);
            std::sort(pair.m_points.begin(), pair.m_points.end());
            return pair.m_points;
        }
        return {};
    }

    std::vector<btClosestPair> get_closest(CollisionObjectPtr obj, btScalar max_distance=1.f);

    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> get_closest_batch(std::unordered_map<CollisionObjectPtr, btScalar> params);

    std::vector<btClosestPair> get_closest_filtered(CollisionObjectPtr obj, const std::vector<CollisionObjectPtr>& other_objects, btScalar max_distance = 1.0);

    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> get_closest_filtered_batch(const std::unordered_map<CollisionObjectPtr, std::pair<std::vector<CollisionObjectPtr>, btScalar>>& query);
    
    std::vector<btClosestPair> get_closest_filtered_POD(CollisionObjectPtr obj, const std::vector<std::pair<CollisionObjectPtr, btScalar>>& other_objects);

    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> get_closest_filtered_POD_batch(const std::unordered_map<CollisionObjectPtr, std::vector<std::pair<CollisionObjectPtr, btScalar>>>& query);

    std::vector<CollisionObjectPtr> overlap_aabb(const btVector3& aabb_min, const btVector3& aabb_max);

    inline std::vector<CollisionObjectPtr> get_collision_objects() const { 
        return std::vector<CollisionObjectPtr>(m_collision_object_set.begin(), m_collision_object_set.end()); 
    }
};
