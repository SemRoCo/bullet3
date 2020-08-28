#include <algorithm>
#include <iostream>

#include "kineverse_world.h"
#include "kineverse_utils.h"

KineverseWorld::KineverseWorld()
: btCollisionWorld(0, 0, 0) {        
    m_dispatcher = Dispatcher_ptr(new btCollisionDispatcher(&m_collision_configuration));
    m_broadphasePairCache = &m_bpinterface;
    m_dispatcher1 = m_dispatcher.get();
}

KineverseWorld::~KineverseWorld() {
    for (const auto object_ptr: m_collision_object_set) {
        btCollisionWorld::removeCollisionObject(object_ptr.get());    
    }
}

void KineverseWorld::addCollisionObject(btCollisionObject* collisionObject, 
                                        int collisionFilterGroup, 
                                        int collisionFilterMask) {
    throw std::runtime_error("DO NOT USE THE RAW-POINTER FUNCTION FOR ADDING OBJECTS TO THE KINEVERSE WORLD!");
}

void KineverseWorld::removeCollisionObject(btCollisionObject* collisionObject) {
    throw std::runtime_error("DO NOT USE THE RAW-POINTER FUNCTION FOR REMOVING OBJECTS FROM THE KINEVERSE WORLD!");
}

void KineverseWorld::addCollisionObject(CollisionObjectPtr collisionObject, 
                        int collisionFilterGroup, 
                        int collisionFilterMask) {
    if (m_collision_object_set.find(collisionObject) != m_collision_object_set.end())
        return;
    btCollisionWorld::addCollisionObject(collisionObject.get(), collisionFilterGroup, collisionFilterMask);
    m_collision_object_set.insert(collisionObject);
    m_collision_object_ptr_map[collisionObject.get()] = collisionObject;
}

void KineverseWorld::removeCollisionObject(CollisionObjectPtr collisionObject) {
    if (m_collision_object_set.find(collisionObject) == m_collision_object_set.end())
        return;
    btCollisionWorld::removeCollisionObject(collisionObject.get());
    m_collision_object_set.erase(collisionObject);
    m_collision_object_ptr_map.erase(collisionObject.get());
}

std::vector<ContactPair> KineverseWorld::get_contacts() {
    btDispatcher* dispatcher = getDispatcher();

    if (!dispatcher) {
        throw std::runtime_error("Can not fetch contacts since dispatcher is NULL.");
    }

    int n_manifolds = dispatcher->getNumManifolds();
    std::vector<ContactPair> out(n_manifolds);
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

        out.push_back(ContactPair(body_a_ptr, body_b_ptr));
        ContactPair& pair = out[i];
        //std::cout << "Created contact pair" << std::endl;
        pair.m_points.reserve(manifold->getNumContacts());
        //std::cout << "Reserved contact space. Number of contacts: " << manifold->getNumContacts() << std::endl;
        for (int p = 0; p < manifold->getNumContacts(); p++) {
            //std::cout << "Iterating over contacts. At: " << p << std::endl;
            const btManifoldPoint& pt = manifold->getContactPoint(p);
            pair.m_points.push_back(ContactPoint(pt.m_localPointA, pt.m_localPointB, pt.m_normalWorldOnB, pt.getDistance()));
        }
    }
    return out;   
}

std::vector<ContactPoint> KineverseWorld::get_distance(CollisionObjectPtr obj_a, 
                                                       CollisionObjectPtr obj_b, btScalar max_distance) {
    if (obj_a != obj_b) {
        ContactPair pair(obj_a, obj_b);
        pair.m_closestDistanceThreshold = max_distance;
        contactPairTest(obj_a.get(), obj_b.get(), pair);
        std::sort(pair.m_points.begin(), pair.m_points.end());
        return pair.m_points;
    }
    return {};
}

std::vector<ClosestPair> KineverseWorld::get_closest(CollisionObjectPtr obj, btScalar max_distance) {
    PairAccumulator<ClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), m_collision_object_ptr_map);
    pair.m_closestDistanceThreshold = max_distance;
    btScalar sq_distance = max_distance * max_distance;
    for (auto other_ptr: m_collision_object_set) {
        if (other_ptr != obj) {
            //printf("obj: %x other_ptr: %x\n", obj, other_ptr);
            contactPairTest(obj.get(), other_ptr.get(), pair);
        }
    }

    std::vector<ClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;
}

std::unordered_map<CollisionObjectPtr, std::vector<ClosestPair>> KineverseWorld::get_closest_batch(std::unordered_map<CollisionObjectPtr, btScalar> params) {
    std::unordered_map<CollisionObjectPtr, std::vector<ClosestPair>> out;
    for (auto kv : params) {
        out[kv.first] = get_closest(kv.first, kv.second);
    }
    return out;
}

std::vector<ClosestPair> KineverseWorld::get_closest_filtered(CollisionObjectPtr obj, const std::vector<CollisionObjectPtr>& other_objects, btScalar max_distance) {
    PairAccumulator<ClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), m_collision_object_ptr_map);
    pair.m_closestDistanceThreshold = max_distance;

    for (auto other_ptr: other_objects) {
        contactPairTest(obj.get(), other_ptr.get(), pair);
    }

    std::vector<ClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;   
}

std::vector<CollisionObjectPtr> KineverseWorld::overlap_aabb(const btVector3& aabb_min, const btVector3& aabb_max) {
    AABBBroadphaseCallback accumulator(m_collision_object_ptr_map);
    m_broadphasePairCache->aabbTest(aabb_min, aabb_max, accumulator);
    return accumulator.m_objects;
}
