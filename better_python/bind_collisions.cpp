#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <btBulletCollisionCommon.h>

#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr

#include <stdarg.h>  // For va_start, etc.

#include <iostream>

#include "kineverse_world.h"
#include "kineverse_query.h"
#include "kineverse_utils.h"
#include "kineverse_compound_shape.h"
#include "kineverse_mesh_loader.h"

namespace py = pybind11;

void batch_set_transforms(const std::vector<CollisionObjectPtr>& objects, const py::array_t<btScalar>& poses) {
    if (poses.ndim() != 2)
        throw std::runtime_error("Passed array is not 2D");
    
    if (poses.shape(0) != objects.size() * 4 || poses.shape(1) != 4)
        throw std::runtime_error(string_format("Passed array has wrong size. Array should be of size %d x 4, but is of shape %d x %d", objects.size() * 4, poses.shape(0), poses.shape(1)));

    auto r = poses.unchecked<2>();
    for (ssize_t i = 0; i < objects.size(); i++) {
        ssize_t row = i * 4;
        objects[i]->setWorldTransform(btTransform(
                        btMatrix3x3(r(row,     0), r(row,     1), r(row,     2),
                                    r(row + 1, 0), r(row + 1, 1), r(row + 1, 2),
                                    r(row + 2, 0), r(row + 2, 1), r(row + 2, 2)),
                        btVector3(r(row, 3), r(row + 1, 3), r(row + 2, 3))));
    }
}

using ShapePtr = std::shared_ptr<btCollisionShape>;


PYBIND11_MODULE(betterpybullet, m) {
    m.doc() = "Attempt at exposing bullet's collision functionality.";


// MESH LOADING

    m.def("load_convex_shape", &load_convex_shape, "Loads mesh file as a convex collision shape. Supported file types .obj, .stl, .dae.", py::arg("filename"), py::arg("use_cache") = true);

    m.def("get_shape_filename", &get_shape_filename, "Returns the name of the mesh file for a given shape, "
                                                     "if the shape is a mesh.", py::arg("shape"));

// VECTORIZATION

    m.def("batch_set_transforms", &batch_set_transforms, "Sets the transforms of a list of matrix by extracting it from a stacked numpy matrix.", py::arg("objects"), py::arg("np_pose_matrix"));

// MATH


    py::class_<btMatrix3x3>(m, "Matrix3")
        .def(py::init<>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&, const btScalar&, const btScalar&, const btScalar&, const btScalar&, const btScalar&, const btScalar&>())
        .def(py::init<const btQuaternion&>())
        .def(py::init<const btMatrix3x3&>())
        .def("get_col", &btMatrix3x3::getColumn)
        .def("get_row", &btMatrix3x3::getRow)
        .def("set_value", &btMatrix3x3::setValue)
        .def("set_identity", &btMatrix3x3::setIdentity)
        .def("set_euler_ypr", &btMatrix3x3::setEulerYPR)
        .def("set_euler_zyx", &btMatrix3x3::setEulerZYX)
        .def("scaled", &btMatrix3x3::scaled)
        .def_static("identity", &btMatrix3x3::getIdentity)
        .def_property_readonly("determinant", &btMatrix3x3::determinant)
        .def_property_readonly("adjoint", &btMatrix3x3::adjoint)
        .def_property_readonly("absolute", &btMatrix3x3::absolute)
        .def_property_readonly("T", &btMatrix3x3::transpose)
        .def_property_readonly("inv", &btMatrix3x3::inverse)
        .def_property("rotation", &btMatrix3x3::getRotation, &btMatrix3x3::setRotation)
        .def("__repr__", (std::string (*)(const btMatrix3x3&)) &toString)
        .def(py::self *= py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self * py::self)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * float())
        .def(py::self * btVector3())
        .def(py::self == py::self);


    py::class_<btVector3>(m, "Vector3")
        .def(py::init<>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&>())
        .def("dot", &btVector3::dot)
        .def("distance", &btVector3::distance)
        .def("distance_sq", &btVector3::distance2)
        .def("normalize",   &btVector3::normalize)
        .def("normalized",  &btVector3::normalized)
        .def("rotate",      &btVector3::rotate)
        .def("angle",       &btVector3::angle)
        .def("abs",         &btVector3::absolute)
        .def("cross",       &btVector3::cross)
        .def("triple",      &btVector3::triple)
        .def("min_axis",    &btVector3::minAxis)
        .def("max_axis",    &btVector3::maxAxis)
        .def("furthest_axis",   &btVector3::furthestAxis)
        .def("closest_axis",    &btVector3::closestAxis)
        .def("set_interpolate", &btVector3::setInterpolate3)
        .def("lerp",            &btVector3::lerp)
        .def("set_max",         &btVector3::setMax)
        .def("set_min",         &btVector3::setMin)
        .def("set_value",       &btVector3::setValue)
        .def("set_zero",        &btVector3::setZero)
        .def_property_readonly("isZero",  &btVector3::isZero)
        .def_property_readonly("norm_sq", &btVector3::length2)
        .def_property_readonly("norm", &btVector3::length)
        .def_property("x", &btVector3::x, &btVector3::setX)
        .def_property("y", &btVector3::y, &btVector3::setY)
        .def_property("z", &btVector3::z, &btVector3::setZ)
        .def_property("w", &btVector3::w, &btVector3::setW)
        .def("__len__", [](const btVector3& q) { return 4; })
        .def("__repr__", (std::string (*)(const btVector3&)) &toString)
        .def("__getitem__", [](const btVector3& v, int idx) {
            switch(idx) {
                case 0: return v.x();
                case 1: return v.y();
                case 2: return v.z();
                case 3: return v.w();
                default: throw std::out_of_range(string_format("Index out of range: %d", idx));
            }
        })
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= float())
        .def(py::self /= float())
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self - py::self)
        .def(-py::self)
        .def(py::self * float())
        .def(float()  * py::self)
        .def(py::self / float())
        .def(py::self / py::self);


    py::class_<btQuadWord>(m, "QuadWord")
        .def(py::init<>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&, const btScalar&>())
        .def_property("x", &btQuadWord::x, &btQuadWord::setX)
        .def_property("y", &btQuadWord::y, &btQuadWord::setY)
        .def_property("z", &btQuadWord::z, &btQuadWord::setZ)
        .def_property("w", &btQuadWord::w, &btQuadWord::setW)
        .def("__len__", [](const btQuaternion& q) { return 4; })
        .def("set_value", (void (btQuadWord::*)(const btScalar&, const btScalar&, const btScalar&)) &btQuadWord::setValue)
        .def("set_value", (void (btQuadWord::*)(const btScalar&, const btScalar&, const btScalar&, const btScalar&)) &btQuadWord::setValue)
        .def("set_max", &btQuadWord::setMax)
        .def("set_min", &btQuadWord::setMin);

    py::class_<btQuaternion, btQuadWord>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&, const btScalar&>())
        .def(py::init<const btVector3&, const btScalar&>())
        .def(py::init<const btScalar&, const btScalar&, const btScalar&>())
        .def("set_rotation", &btQuaternion::setRotation)
        .def("set_euler", &btQuaternion::setEuler)
        .def("set_euler_zyx", &btQuaternion::setEulerZYX)
        .def("dot", &btQuaternion::dot)
        .def("normalize", &btQuaternion::normalize)
        .def("normalized", &btQuaternion::normalized)
        .def("get_angle", &btQuaternion::angle)
        .def("get_angle_shortest_path", &btQuaternion::angleShortestPath)
        .def("inverse", &btQuaternion::inverse)
        .def("nearest", &btQuaternion::nearest)
        .def("farthest", &btQuaternion::farthest)
        .def("slerp", &btQuaternion::slerp)
        .def_static("identity", &btQuaternion::getIdentity)
        .def_property_readonly("w", &btQuaternion::getW)
        .def_property_readonly("axis", &btQuaternion::getAxis)
        .def_property_readonly("angle", &btQuaternion::getAngle)
        .def_property_readonly("shortest_angle", &btQuaternion::getAngleShortestPath)
        .def_property_readonly("euler_zyx", &btQuaternion::getEulerZYX)
        .def_property_readonly("norm_sq", &btQuaternion::length2)
        .def_property_readonly("norm", &btQuaternion::length)
        .def("__repr__", (std::string (*)(const btQuaternion&)) &toString)
        .def("__getitem__", [](const btQuaternion& v, int idx) {
            switch(idx) {
                case 0: return v.x();
                case 1: return v.y();
                case 2: return v.z();
                case 3: return v.w();
                default: throw std::out_of_range(string_format("Index out of range: %d", idx));
            }
        })
        .def(py::self * float())
        .def(py::self / float())
        .def(py::self *= float())
        .def(py::self /= float())
        .def(-py::self)
        .def(py::self * py::self)
        .def(py::self * btVector3())
        .def(btVector3() * py::self);

    py::class_<btTransform>(m, "Transform")
        .def(py::init<>())
        .def("__init__", [](btTransform& self, btScalar x, btScalar y, btScalar z) {
            self.setRotation(btQuaternion::getIdentity());
            self.setOrigin(btVector3(x,y,z));
        }, py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init<const btQuaternion&>())
        .def(py::init<const btQuaternion&, const btVector3&>())
        .def(py::init<const btMatrix3x3&>())
        .def(py::init<const btMatrix3x3&, const btVector3&>())
        .def(py::init<const btTransform&>())
        .def_property("rotation", &btTransform::getRotation, &btTransform::setRotation)
        .def_property("origin", (btVector3& (btTransform::*)()) &btTransform::getOrigin, &btTransform::setOrigin)
        .def_property("basis", (btMatrix3x3& (btTransform::*)()) &btTransform::getBasis, &btTransform::setBasis)
        .def("set_identity", &btTransform::setIdentity)
        .def("inv", &btTransform::inverse)
        .def_static("identity", &btTransform::getIdentity)
        .def("__repr__", (std::string (*)(const btTransform&)) &toString)
        .def(py::self *  btVector3())
        .def(py::self *  btQuaternion())
        .def(py::self *  py::self)
        .def(py::self *= py::self);



    py::class_<KineverseCollisionObject, CollisionObjectPtr> collision_object(m, "CollisionObject");

    collision_object.def(py::init<>())
        .def("set_contact_stiffness_and_damping", &btCollisionObject::setContactStiffnessAndDamping)
        .def("set_anisotropic_friction", &btCollisionObject::setAnisotropicFriction)
        .def("has_anisotropic_friction", &btCollisionObject::hasAnisotropicFriction)
        .def("set_ignore_collision", [](KineverseCollisionObject& self, CollisionObjectPtr other, bool ignore) {
                self.setIgnoreCollisionCheck(other.get(), ignore);
            }, py::arg("other_object"), py::arg("ignore") = true)
        .def("check_collsion_override", &btCollisionObject::checkCollideWithOverride)
        .def("activate", &btCollisionObject::activate)
        .def("force_activation_state", &btCollisionObject::forceActivationState)
        .def_property("transform", (btTransform& (btCollisionObject::*)()) &btCollisionObject::getWorldTransform, &btCollisionObject::setWorldTransform)
        .def_property("deactivation_time", &btCollisionObject::getDeactivationTime, &btCollisionObject::setDeactivationTime)
        .def_property("activation_state", &btCollisionObject::getActivationState, &btCollisionObject::setActivationState)
        .def_property("contact_processing_threshold", &btCollisionObject::getContactProcessingThreshold, &btCollisionObject::setContactProcessingThreshold)
        .def_property("collision_shape", &KineverseCollisionObject::getCollisionShape, 
                                         (void (KineverseCollisionObject:: *)(ShapePtr)) &KineverseCollisionObject::setCollisionShape)
        .def_property("friction",          &btCollisionObject::getFriction, &btCollisionObject::setFriction)
        .def_property("restitution",       &btCollisionObject::getRestitution, &btCollisionObject::setRestitution)
        .def_property("rolling_friction",  &btCollisionObject::getRollingFriction, &btCollisionObject::setRollingFriction)
        .def_property("spinning_friction", &btCollisionObject::getSpinningFriction, &btCollisionObject::setSpinningFriction)
        .def_property("collision_flags",   &btCollisionObject::getCollisionFlags, &btCollisionObject::setCollisionFlags)
        .def_property("ccd_swept_sphere_radius", &btCollisionObject::getCcdSweptSphereRadius, &btCollisionObject::setCcdSweptSphereRadius)
        .def_property_readonly("isActive", &btCollisionObject::isActive)
        .def_property_readonly("isStaticObject", &btCollisionObject::isStaticObject)
        .def_property_readonly("isKinematicObject", &btCollisionObject::isKinematicObject)
        .def_property_readonly("isStaticOrKinematic", &btCollisionObject::isStaticOrKinematicObject)
        .def_property_readonly("has_contact_response", &btCollisionObject::hasContactResponse)
        .def_property_readonly("merges_islands", &btCollisionObject::mergesSimulationIslands)
        .def_property_readonly("contact_stiffness", &btCollisionObject::getContactStiffness)
        .def_property_readonly("contact_damping",   &btCollisionObject::getContactDamping)
        .def_property_readonly("anisotropic_friction", &btCollisionObject::getAnisotropicFriction)
        .def_property_readonly("aabb", [](KineverseCollisionObject& self) {
            btVector3 min, max;
            self.getCollisionShape()->getAabb(self.getWorldTransform(), min, max);
            return py::make_tuple(min, max);
        });

    py::enum_<btCollisionObject::CollisionFlags>(collision_object, "CollisionFlags", py::arithmetic())
        .value("StaticObject",                  btCollisionObject::CollisionFlags::CF_STATIC_OBJECT)
        .value("KinematicObject",               btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT)
        .value("NoContactResponse",             btCollisionObject::CollisionFlags::CF_NO_CONTACT_RESPONSE)
        .value("CustomMaterialCallback",        btCollisionObject::CollisionFlags::CF_CUSTOM_MATERIAL_CALLBACK)
        .value("CharacterObject",               btCollisionObject::CollisionFlags::CF_CHARACTER_OBJECT)
        .value("DisableVisualizeObject",        btCollisionObject::CollisionFlags::CF_DISABLE_VISUALIZE_OBJECT)
        .value("DisableSpuCollisionProcessing", btCollisionObject::CollisionFlags::CF_DISABLE_SPU_COLLISION_PROCESSING)
        .value("HasContactStiffnessDamping",    btCollisionObject::CollisionFlags::CF_HAS_CONTACT_STIFFNESS_DAMPING)
        .value("HasCustomDebugRenderingColor",  btCollisionObject::CollisionFlags::CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR)
        .value("HasFrictionAnchor",             btCollisionObject::CollisionFlags::CF_HAS_FRICTION_ANCHOR)
        .value("HasCollisionSoundTrigger",      btCollisionObject::CollisionFlags::CF_HAS_COLLISION_SOUND_TRIGGER)
        .export_values();

    py::enum_<btCollisionObject::CollisionObjectTypes>(collision_object, "CollisionObjectTypes", py::arithmetic())
        .value("CollisionObject",  btCollisionObject::CollisionObjectTypes::CO_COLLISION_OBJECT)
        .value("RigidBody",        btCollisionObject::CollisionObjectTypes::CO_RIGID_BODY)
        .value("GhostObject",      btCollisionObject::CollisionObjectTypes::CO_GHOST_OBJECT)
        .value("SoftBody",         btCollisionObject::CollisionObjectTypes::CO_SOFT_BODY)
        .value("HfFluid",          btCollisionObject::CollisionObjectTypes::CO_HF_FLUID)
        .value("UserType",         btCollisionObject::CollisionObjectTypes::CO_USER_TYPE)
        .value("FeatherstoneLink", btCollisionObject::CollisionObjectTypes::CO_FEATHERSTONE_LINK)
        .export_values();

    py::enum_<btCollisionObject::AnisotropicFrictionFlags>(collision_object, "AnisotropicFrictionFlags", py::arithmetic())
        .value("AnisotropicFrictionDisabled", btCollisionObject::AnisotropicFrictionFlags::CF_ANISOTROPIC_FRICTION_DISABLED)
        .value("AnisotropicFriction", btCollisionObject::AnisotropicFrictionFlags::CF_ANISOTROPIC_FRICTION)
        .value("AnisotropicRollingFriction", btCollisionObject::AnisotropicFrictionFlags::CF_ANISOTROPIC_ROLLING_FRICTION)
        .export_values();

    py::class_<btCollisionShape, ShapePtr>(m, "CollisionShape")
        .def_property_readonly("isPolyhedral", &btCollisionShape::isPolyhedral)
        .def_property_readonly("isConvex2d", &btCollisionShape::isConvex2d)
        .def_property_readonly("isConvex", &btCollisionShape::isConvex)
        .def_property_readonly("isNonMoving", &btCollisionShape::isNonMoving)
        .def_property_readonly("isConcave", &btCollisionShape::isConcave)
        .def_property_readonly("isCompound", &btCollisionShape::isCompound)
        .def_property_readonly("isSoftBody", &btCollisionShape::isSoftBody)
        .def_property_readonly("isInfinite", &btCollisionShape::isInfinite)
        .def_property_readonly("shape_type", &btCollisionShape::getShapeType)
        .def_property_readonly("angular_motion_disc", &btCollisionShape::getAngularMotionDisc)
        .def_property_readonly("anisotropic_rolling_friction_direction", &btCollisionShape::getAnisotropicRollingFrictionDirection)
        .def_property("margin", &btCollisionShape::getMargin, &btCollisionShape::setMargin)
#ifndef __SPU__
        .def_property("local_scaling", &btCollisionShape::getLocalScaling, &btCollisionShape::setLocalScaling)
        .def("calculate_local_inertia", &btCollisionShape::calculateLocalInertia)
#endif
        .def("get_contact_breaking_threshold", &btCollisionShape::getContactBreakingThreshold)
        .def("get_aabb", [](btCollisionShape& self, const btTransform& t) {
            btVector3 min, max;
            self.getAabb(t, min, max);
            return py::make_tuple(min, max);
        }, py::arg("transform"))
        .def("get_bounding_sphere", &btCollisionShape::getBoundingSphere);


    py::class_<btCollisionWorld::LocalShapeInfo>(m, "LocalShapeInfo")
        .def_readonly("part", &btCollisionWorld::LocalShapeInfo::m_shapePart)
        .def_readonly("triangle_index", &btCollisionWorld::LocalShapeInfo::m_triangleIndex);

    py::class_<btCollisionWorld::LocalRayResult>(m, "LocalRayResult")
        .def_readonly("collision_object", &btCollisionWorld::LocalRayResult::m_collisionObject)
        .def_readonly("shape_info", &btCollisionWorld::LocalRayResult::m_localShapeInfo)
        .def_readonly("hit_normal_local", &btCollisionWorld::LocalRayResult::m_hitNormalLocal)
        .def_readonly("hit_fraction", &btCollisionWorld::LocalRayResult::m_hitFraction)
        .def_property_readonly("has_hit", [](const btCollisionWorld::LocalRayResult& r){
            return r.m_collisionObject != 0;
        });

    py::class_<btCollisionWorld::RayResultCallback>(m, "RayResult")
        .def_readwrite("collision_object", &btCollisionWorld::RayResultCallback::m_collisionObject)
        .def_readwrite("filter_group", &btCollisionWorld::RayResultCallback::m_collisionFilterGroup)
        .def_readwrite("filter_mask", &btCollisionWorld::RayResultCallback::m_collisionFilterMask)
        .def_readwrite("hit_fraction", &btCollisionWorld::RayResultCallback::m_closestHitFraction)
        .def_property_readonly("has_hit", &btCollisionWorld::RayResultCallback::hasHit);

    py::class_<btCollisionWorld::ClosestRayResultCallback, btCollisionWorld::RayResultCallback>(m, "ClosestRayResult")
        .def(py::init<const btVector3&, const btVector3&>())
        .def_readonly("from", &btCollisionWorld::ClosestRayResultCallback::m_rayFromWorld)
        .def_readonly("to", &btCollisionWorld::ClosestRayResultCallback::m_rayToWorld)
        .def_readonly("hit_normal", &btCollisionWorld::ClosestRayResultCallback::m_hitNormalWorld)
        .def_readonly("hit_point", &btCollisionWorld::ClosestRayResultCallback::m_hitPointWorld);

    py::class_<btCollisionWorld::AllHitsRayResultCallback, btCollisionWorld::RayResultCallback>(m, "AllHitsRayResult")
        .def(py::init<const btVector3&, const btVector3&>())
        .def_readonly("from", &btCollisionWorld::AllHitsRayResultCallback::m_rayFromWorld)
        .def_readonly("to", &btCollisionWorld::AllHitsRayResultCallback::m_rayToWorld)
        .def_readonly("hit_normal", &btCollisionWorld::AllHitsRayResultCallback::m_hitNormalWorld)
        .def_readonly("hit_point", &btCollisionWorld::AllHitsRayResultCallback::m_hitPointWorld)
        .def_readonly("hit_fractions", &btCollisionWorld::AllHitsRayResultCallback::m_hitFractions);

    py::class_<btCollisionWorld::LocalConvexResult>(m, "LocalConvexResult")
        .def_readonly("collision_object", &btCollisionWorld::LocalConvexResult::m_hitCollisionObject)
        .def_readonly("shape_info", &btCollisionWorld::LocalConvexResult::m_localShapeInfo)
        .def_readonly("hit_normal_local", &btCollisionWorld::LocalConvexResult::m_hitNormalLocal)
        .def_readonly("hit_point_local", &btCollisionWorld::LocalConvexResult::m_hitPointLocal)
        .def_readonly("hit_fraction", &btCollisionWorld::LocalConvexResult::m_hitFraction);

    py::class_<btCollisionWorld::ConvexResultCallback>(m, "ConvexResult")
        .def_readonly("closest_hit_fraction", &btCollisionWorld::ConvexResultCallback::m_closestHitFraction)
        .def_readonly("filter_group", &btCollisionWorld::ConvexResultCallback::m_collisionFilterGroup)
        .def_readonly("filter_mask", &btCollisionWorld::ConvexResultCallback::m_collisionFilterMask)
        .def_property_readonly("has_hit", &btCollisionWorld::ConvexResultCallback::hasHit);

    py::class_<btCollisionWorld::ClosestConvexResultCallback, btCollisionWorld::ConvexResultCallback>(m, "ClosestConvexResult")
        .def(py::init<const btVector3&, const btVector3&>())
        .def_readonly("collision_object", &btCollisionWorld::ClosestConvexResultCallback::m_hitCollisionObject)
        .def_readonly("from", &btCollisionWorld::ClosestConvexResultCallback::m_convexFromWorld)
        .def_readonly("to", &btCollisionWorld::ClosestConvexResultCallback::m_convexToWorld)
        .def_readonly("hit_normal", &btCollisionWorld::ClosestConvexResultCallback::m_hitNormalWorld)
        .def_readonly("hit_point", &btCollisionWorld::ClosestConvexResultCallback::m_hitPointWorld);

    py::class_<btCollisionWorld::ContactResultCallback>(m, "ContactResult")
        .def_readwrite("filter_group", &btCollisionWorld::ContactResultCallback::m_collisionFilterGroup)
        .def_readwrite("filter_mask", &btCollisionWorld::ContactResultCallback::m_collisionFilterMask)
        .def_readwrite("closest_distance_threshold", &btCollisionWorld::ContactResultCallback::m_closestDistanceThreshold);

    py::class_<KineverseWorld>(m, "KineverseWorld")
        .def(py::init<>())
        .def("closest_ray_test", &KineverseWorld::closest_ray_test<btCollisionWorld::ClosestRayResultCallback>, py::arg("from"), py::arg("to"))
        .def("closest_ray_test_batch", &KineverseWorld::closest_ray_batch_test<btCollisionWorld::ClosestRayResultCallback>, py::arg("from"), py::arg("to"))
        .def("get_closest", &KineverseWorld::get_closest, py::arg("object"), py::arg("max_distance"))
        .def("get_distance", &KineverseWorld::get_distance, py::arg("object_a"), py::arg("object_b"))
        .def("get_contacts", &KineverseWorld::get_contacts)
        .def("get_closest_batch", &KineverseWorld::get_closest_batch, "Performs a batched determination of closest object. Parameter maps objects to their max distances", py::arg("max_distances"))
        .def("overlap_aabb", &KineverseWorld::overlap_aabb, py::arg("aabb_min"), py::arg("aabb_max"))
        .def("update_single_aabb", &btCollisionWorld::updateSingleAabb)
        .def("update_aabbs", &btCollisionWorld::updateAabbs)
        .def("compute_overlapping_pairs", &btCollisionWorld::computeOverlappingPairs)
        .def("debug_draw_world", &btCollisionWorld::debugDrawWorld)
        .def("debug_draw_object", &btCollisionWorld::debugDrawObject)
        .def("num_collision_objects", &btCollisionWorld::getNumCollisionObjects)
        .def("add_collision_object", (void (KineverseWorld::*)(CollisionObjectPtr, int, int)) &KineverseWorld::addCollisionObject, 
                                     py::arg("object"), py::arg("filter_group") = 1, py::arg("filter_mask") = -1)
        .def("remove_collision_object", (void (KineverseWorld::*)(CollisionObjectPtr)) &KineverseWorld::removeCollisionObject)
        .def("perform_discrete_collision_detection", &btCollisionWorld::performDiscreteCollisionDetection)
        .def_property("force_update_all_aabbs", &btCollisionWorld::getForceUpdateAllAabbs, 
                                                &btCollisionWorld::setForceUpdateAllAabbs)
        .def_property_readonly("collision_objects", &KineverseWorld::get_collision_objects)
        .def("add_collision_object", [](KineverseWorld& self, ShapePtr shape, const btTransform& t) {
            CollisionObjectPtr out = std::make_shared<KineverseCollisionObject>();
            out->setCollisionShape(shape);
            out->setWorldTransform(t);
            self.addCollisionObject(out);
            return out;
        }, py::arg("shape"), py::arg("transform") = btTransform::getIdentity());;

    py::class_<ContactPoint>(m, "ContactPoint")
        .def(py::init<const btVector3&, const btVector3&, const btVector3&, btScalar>())
        .def_readonly("point_a", &ContactPoint::m_pointOnA)
        .def_readonly("point_b", &ContactPoint::m_pointOnB)
        .def_readonly("normal_world_b", &ContactPoint::m_normalWorldB)
        .def_readonly("distance", &ContactPoint::m_distance);

    py::class_<ContactPair>(m, "ContactPair")
        .def(py::init<std::shared_ptr<const KineverseCollisionObject>, 
                      std::shared_ptr<const KineverseCollisionObject>>())
        .def_readonly("obj_a", &ContactPair::m_obj_a)
        .def_readonly("obj_b", &ContactPair::m_obj_b)
        .def_readonly("points", &ContactPair::m_points);

    py::class_<ClosestPair, ContactPair>(m, "ClosestPair");

//////// SHAPES

    py::class_<btConvexShape, btCollisionShape, std::shared_ptr<btConvexShape>>(m, "ConvexShape")
        .def("local_supporting_vertex", &btConvexShape::localGetSupportingVertex)
        .def("local_supporting_vertex_no_margin", &btConvexShape::localGetSupportingVertexWithoutMargin)
        .def_property("margin", &btConvexShape::getMargin, &btConvexShape::setMargin)
        .def_property("scaling", &btConvexShape::getLocalScaling, &btConvexShape::setLocalScaling);


    py::class_<btPolyhedralConvexShape, btConvexShape, std::shared_ptr<btPolyhedralConvexShape>>(m, "PolyedralConvexShape")
        .def_property_readonly("nvertices", &btPolyhedralConvexShape::getNumVertices)
        .def_property_readonly("nedges", &btPolyhedralConvexShape::getNumEdges)
        .def_property_readonly("nplanes", &btPolyhedralConvexShape::getNumPlanes)
        .def("is_inside", &btPolyhedralConvexShape::isInside);

    py::class_<btBoxShape, btPolyhedralConvexShape, std::shared_ptr<btBoxShape>>(m, "BoxShape")
        .def(py::init<const btVector3&>())
        .def_property_readonly("extents", [](const btBoxShape& self) {
            return self.getHalfExtentsWithoutMargin() * 2.f;
        });

    py::class_<btCylinderShape, btConvexShape, std::shared_ptr<btCylinderShape>>(m, "CylinderShape")
        .def_property_readonly("axis", &btCylinderShape::getUpAxis)
        .def_property_readonly("radius", &btCylinderShape::getRadius)
        .def_property_readonly("height", [](const btCylinderShape& self) {
            switch (self.getUpAxis()) {
                case 0: return self.getHalfExtentsWithoutMargin().getX() * 2.f;
                case 1: return self.getHalfExtentsWithoutMargin().getY() * 2.f;
                case 2: return self.getHalfExtentsWithoutMargin().getZ() * 2.f;
                default: throw std::out_of_range(string_format("getUpAxis is %d, that should never have happened!", self.getUpAxis()));
            }
        });

    py::class_<btCylinderShapeX, btCylinderShape, std::shared_ptr<btCylinderShapeX>>(m, "CylinderShapeX")
        .def(py::init<const btVector3&>());

    py::class_<btCylinderShapeZ, btCylinderShape, std::shared_ptr<btCylinderShapeZ>>(m, "CylinderShapeZ")
        .def(py::init<const btVector3&>());

    py::class_<btSphereShape, btConvexShape, std::shared_ptr<btSphereShape>>(m, "SphereShape")
        .def(py::init<btScalar>(), py::arg("radius"))
        .def_property("radius", &btSphereShape::getRadius, &btSphereShape::setUnscaledRadius);

    py::class_<btCapsuleShape, btConvexShape, std::shared_ptr<btCapsuleShape>>(m, "CapsuleShape")
        .def(py::init<btScalar, btScalar>(), py::arg("radius"), py::arg("height"))
        .def_property_readonly("radius", &btCapsuleShape::getRadius)
        .def_property_readonly("height", [](const btCapsuleShape& c) {
            return c.getHalfHeight() * 2;
        })
        .def_property_readonly("axis", &btCapsuleShape::getUpAxis);

    py::class_<btConeShape, btConvexShape, std::shared_ptr<btConeShape>>(m, "ConeShape")
        .def(py::init<btScalar, btScalar>(), py::arg("radius"), py::arg("height"))
        .def_property("radius", &btConeShape::getRadius, &btConeShape::setRadius)
        .def_property("height", &btConeShape::getHeight, &btConeShape::setHeight);

    py::class_<KineverseCompoundShape, btCollisionShape, std::shared_ptr<KineverseCompoundShape>>(m, "CompoundShape")
        .def(py::init<>())
        .def("add_child", &KineverseCompoundShape::addChildShape, py::arg("transform"), py::arg("shape"))
        .def("remove_child", &KineverseCompoundShape::removeChildShape, py::arg("shape"))
        .def("get_child", (ShapePtr (KineverseCompoundShape::*)(int)) &KineverseCompoundShape::getChildShape, 
                          py::arg("index"))
        .def("get_child_transform", (btTransform& (btCompoundShape::*)(int)) &btCompoundShape::getChildTransform, py::arg("index"))
        .def("update_child_transform", &btCompoundShape::updateChildTransform, py::arg("index"), py::arg("transform"), py::arg("recompute_aabb"))
        .def_property_readonly("nchildren", &btCompoundShape::getNumChildShapes)
        .def_property_readonly("file_path", [](ShapePtr shape) {
            return get_shape_filename(shape.get());
        })
        .def_property_readonly("child_shapes", &KineverseCompoundShape::get_child_shapes);

    py::class_<btConvexHullShape, btPolyhedralConvexShape, std::shared_ptr<btConvexHullShape>>(m, "ConvexHullShape")
        .def(py::init<>())
        .def("add_point", &btConvexHullShape::addPoint)
        .def("optimize_hull", &btConvexHullShape::optimizeConvexHull)
        .def_property_readonly("npoints", &btConvexHullShape::getNumPoints)
        .def_property_readonly("file_path", [](btCollisionShape* shape) {
            return get_shape_filename(shape);
        });
}
