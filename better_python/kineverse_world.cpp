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

std::vector<btClosestPair> KineverseWorld::get_closest(CollisionObjectPtr obj, btScalar max_distance) {
    PairAccumulator<btClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), m_collision_object_ptr_map);
    pair.m_closestDistanceThreshold = max_distance;
    btScalar sq_distance = max_distance * max_distance;
    for (auto other_ptr: m_collision_object_set) {
        if (other_ptr != obj) {
            //printf("obj: %x other_ptr: %x\n", obj, other_ptr);
            contactPairTest(obj.get(), other_ptr.get(), pair);
        }
    }

    std::vector<btClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;
}

std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> KineverseWorld::get_closest_batch(std::unordered_map<CollisionObjectPtr, btScalar> params) {
    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> out;
    for (auto kv : params) {
        out[kv.first] = get_closest(kv.first, kv.second);
    }
    return out;
}

std::vector<btClosestPair> KineverseWorld::get_closest_filtered(CollisionObjectPtr obj, const std::vector<CollisionObjectPtr>& other_objects, btScalar max_distance) {
    PairAccumulator<btClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), m_collision_object_ptr_map);
    pair.m_closestDistanceThreshold = max_distance;

    for (auto other_ptr: other_objects) {
        contactPairTest(obj.get(), other_ptr.get(), pair);
    }

    std::vector<btClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;
}

std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> KineverseWorld::get_closest_filtered_batch(const std::unordered_map<CollisionObjectPtr, std::pair<std::vector<CollisionObjectPtr>, btScalar>>& query) {
    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> out;
    for (auto kv : query) {
        out[kv.first] = get_closest_filtered(kv.first, kv.second.first, kv.second.second);
    }
    return out;   
}

std::vector<btClosestPair> KineverseWorld::get_closest_filtered_POD(CollisionObjectPtr obj, const std::vector<std::pair<CollisionObjectPtr, btScalar>>& other_objects) {

    PairAccumulator<btClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), m_collision_object_ptr_map);

    for (auto query_pair: other_objects) {
        pair.m_closestDistanceThreshold = query_pair.second;
        contactPairTest(obj.get(), query_pair.first.get(), pair);
    }

    std::vector<btClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;
}

std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> KineverseWorld::get_closest_filtered_POD_batch(const std::unordered_map<CollisionObjectPtr, std::vector<std::pair<CollisionObjectPtr, btScalar>>>& query) {
    std::unordered_map<CollisionObjectPtr, std::vector<btClosestPair>> out;
    for (auto kv : query) {
        out[kv.first] = get_closest_filtered_POD(kv.first, kv.second);
    }
    return out;      
}

std::vector<CollisionObjectPtr> KineverseWorld::overlap_aabb(const btVector3& aabb_min, const btVector3& aabb_max) {
    AABBBroadphaseCallback accumulator(m_collision_object_ptr_map);
    m_broadphasePairCache->aabbTest(aabb_min, aabb_max, accumulator);
    return accumulator.m_objects;
}
