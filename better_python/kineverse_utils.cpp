#include <string>
#include <stdarg.h>  // For va_start, etc.
#include <stdarg.h>  // For va_start, etc.

#include <LinearMath/btTransform.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btQuaternion.h>

#include "kineverse_utils.h"

std::string string_format(const std::string fmt, ...) {
    int size = ((int)fmt.size()) * 2 + 50;   // Use a rubric appropriate for your code
    std::string str;
    va_list ap;
    while (1) {     // Maximum two passes on a POSIX system...
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.data(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {  // Everything worked
            str.resize(n);
            return str;
        }
        if (n > -1)  // Needed size returned
            size = n + 1;   // For null char
        else
            size *= 2;      // Guess at a larger size (OS specific)
    }
    return str;
}


std::string toString(const btTransform& transform) {
    const btMatrix3x3& basis = transform.getBasis();
    const btVector3& row_x   = basis[0];
    const btVector3& row_y   = basis[1];
    const btVector3& row_z   = basis[2];
    const btVector3& origin  = transform.getOrigin();
    return string_format("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f", 
                            row_x.x(), row_x.y(), row_x.z(), origin.x(),
                            row_y.x(), row_y.y(), row_y.z(), origin.y(),
                            row_z.x(), row_z.y(), row_z.z(), origin.z(),
                                  0.f,       0.f,       0.f, origin.w());
}

std::string toString(const btMatrix3x3& matrix) {
    const btVector3& row_x   = matrix[0];
    const btVector3& row_y   = matrix[1];
    const btVector3& row_z   = matrix[2];
    return string_format("%f %f %f\n%f %f %f\n%f %f %f", 
                            row_x.x(), row_x.y(), row_x.z(),
                            row_y.x(), row_y.y(), row_y.z(),
                            row_z.x(), row_z.y(), row_z.z());
}

std::string toString(const btVector3& vector) {
    return string_format("%f\n%f\n%f\n%f", vector.x(), vector.y(), vector.z(), vector.w());
}

std::string toString(const btQuaternion& quat) {
    return string_format("%f\n%f\n%f\n%f", quat.x(), quat.y(), quat.z(), quat.w());
}

btScalar minAabbDistanceSq(const btVector3& minAabb_a, const btVector3& maxAabb_a,
                           const btVector3& minAabb_b, const btVector3& maxAabb_b) {
    btScalar x_delta = std::max(minAabb_a.m_floats[0] - maxAabb_b.m_floats[0], 
                                minAabb_b.m_floats[0] - maxAabb_a.m_floats[0]);
    btScalar y_delta = std::max(minAabb_a.m_floats[1] - maxAabb_b.m_floats[1], 
                                minAabb_b.m_floats[1] - maxAabb_a.m_floats[1]);
    btScalar z_delta = std::max(minAabb_a.m_floats[2] - maxAabb_b.m_floats[2], 
                                minAabb_b.m_floats[2] - maxAabb_a.m_floats[2]);

    if (x_delta < 0 && y_delta < 0 && z_delta < 0) { // Overlap
        return 0;
    }
    
    // Return squared shortest distance, weeding out overlap
    return std::max(x_delta, static_cast<btScalar>(0)) * std::max(x_delta, static_cast<btScalar>(0)) +
           std::max(y_delta, static_cast<btScalar>(0)) * std::max(y_delta, static_cast<btScalar>(0)) +
           std::max(z_delta, static_cast<btScalar>(0)) * std::max(z_delta, static_cast<btScalar>(0));
}
