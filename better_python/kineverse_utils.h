#pragma once

#include <string>
#include <vector>

#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

// #include <LinearMath/btMatrix3x3.h>


std::string string_format(const std::string fmt, ...);


std::string toString(const btTransform& transform);

std::string toString(const btMatrix3x3& matrix);

std::string toString(const btVector3& vector);

std::string toString(const btQuaternion& quat);

template <typename T>
std::vector<T> aligned_array_to_stl(const btAlignedObjectArray<T>& array) {
    std::vector<T> out;
    out.reserve(array.size());
    for (int i = 0; i < array.size(); i++)
        out.push_back(array[i]);
    return out;
}

inline btTransform create_translation(btScalar x, btScalar y, btScalar z) {
    return btTransform(btQuaternion::getIdentity(), btVector3(x,y,z));
}

btScalar minAabbDistanceSq(const btVector3& minAabb_a, const btVector3& maxAabb_a,
                           const btVector3& minAabb_b, const btVector3& maxAabb_b);

inline btScalar minAabbDistance(const btVector3& minAabb_a, const btVector3& maxAabb_a,
                                const btVector3& minAabb_b, const btVector3& maxAabb_b) {
    return sqrt(minAabbDistanceSq(minAabb_a,
                                  maxAabb_a,
                                  minAabb_b,
                                  maxAabb_b));
}
