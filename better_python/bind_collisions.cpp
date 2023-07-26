#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <btBulletCollisionCommon.h>
#include "../examples/SharedMemory/PhysicsClientC_API.h"

#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr

#include <stdarg.h>  // For va_start, etc.

#include <iostream>

#include "kineverse_world.h"
#include "kineverse_query.h"
#include "kineverse_utils.h"
#include "kineverse_compound_shape.h"
#include "kineverse_mesh_loader.h"
#include "kineverse_settings.h"

namespace py = pybind11;

class np_point : public py::array_t<btScalar> {
 public:
    np_point()
    : py::array_t<btScalar>({4}) {
        auto r = mutable_unchecked<1>();
        r(0) = 0;
        r(1) = 0;
        r(2) = 0;
        r(3) = 1;
    } 
    np_point(const btVector3& vec)
    : py::array_t<btScalar>({4}) {
        auto r = mutable_unchecked<1>();
        r(0) = vec.getX();
        r(1) = vec.getY();
        r(2) = vec.getZ();
        r(3) = 1;
    }  
};

class np_vector : public py::array_t<btScalar> {
 public:
    np_vector()
    : py::array_t<btScalar>({4}) {
        auto r = mutable_unchecked<1>();
        r(0) = 0;
        r(1) = 0;
        r(2) = 0;
        r(3) = 0;
    }
    np_vector(const btVector3& vec)
    : py::array_t<btScalar>({4}) {
        auto r = mutable_unchecked<1>();
        r(0) = vec.getX();
        r(1) = vec.getY();
        r(2) = vec.getZ();
        r(3) = 0;
    }
};

template<typename T>
py::array_t<btScalar> to_np(const T& a);

template<>
py::array_t<btScalar> to_np(const btVector3& a) {
    btScalar buffer[4] = {a.getX(), a.getY(), a.getZ(), 0};
    return py::array_t<btScalar>({4, 1}, buffer);
}

template<>
py::array_t<btScalar> to_np(const btVector4& a) {
    btScalar buffer[4] = {a.getX(), a.getY(), a.getZ(), a.getW()};
    return py::array_t<btScalar>({4, 1}, buffer);
}

py::array_t<btScalar> to_np_point(const btVector3& a) {
    btScalar buffer[4] = {a.getX(), a.getY(), a.getZ(), 1};
    return py::array_t<btScalar>({4, 1}, buffer);   
}

template<>
py::array_t<btScalar> to_np(const btQuaternion& a) {
    btScalar buffer[4] = {a.getX(), a.getY(), a.getZ(), a.getW()};
    return py::array_t<btScalar>({4, 1}, buffer);
}

template<>
py::array_t<btScalar> to_np(const btMatrix3x3& a) {
    btScalar buffer[9] = {a[0][0], a[0][1], a[0][2],
                          a[1][0], a[1][1], a[1][2],
                          a[2][0], a[2][1], a[2][2],};
    return py::array_t<btScalar>({3, 3}, buffer);
}

template<>
py::array_t<btScalar> to_np(const btTransform& a) {
    const btMatrix3x3& basis = a.getBasis();
    const btVector3&   origin = a.getOrigin();
    btScalar buffer[16] = {basis[0][0], basis[0][1], basis[0][2], origin[0],
                           basis[1][0], basis[1][1], basis[1][2], origin[1],
                           basis[2][0], basis[2][1], basis[2][2], origin[2],
                                  0,        0,        0,         1};
    return py::array_t<btScalar>({4, 4}, buffer);
}

template<typename T>
T from_np(py::array_t<btScalar> a);

template<>
btTransform from_np(py::array_t<btScalar> a) {
    if (a.ndim() != 2)
        throw std::runtime_error(string_format("Numpy array needs to be 2D, but is %dD", a.ndim()).c_str());

    if (a.shape(0) != 4 || a.shape(1) != 4)
        throw std::runtime_error(string_format("Numpy array is of wrong shape. "
                                               "Expected is (4, 4) got (%d, %d)", a.shape(0), a.shape(1)).c_str());
    auto r = a.unchecked<2>();
    return btTransform(btMatrix3x3(r(0, 0), r(0, 1), r(0, 2),
                                   r(1, 0), r(1, 1), r(1, 2),
                                   r(2, 0), r(2, 1), r(2, 2)),
                       btVector3(r(0, 3), r(1, 3), r(2, 3)));    
}

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

using npContactPoint = ContactPoint<np_point, np_vector>;
using npContactPair  = ContactPair<np_point, np_vector>;
using npClosestPair  = ClosestPair<np_point, np_vector>;

using btContactPoint = ContactPoint<btVector3, btVector3>;
using btContactPair  = ContactPair<btVector3, btVector3>;
using btClosestPair  = ClosestPair<btVector3, btVector3>;

py::list py_get_closest(KineverseWorld& world, 
                        CollisionObjectPtr obj, btScalar max_distance = 1.0) {
    PairAccumulator<npClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), world.get_object_ptr_map());
    pair.m_closestDistanceThreshold = max_distance;

    const auto& collision_objects = world.getCollisionObjectArray();
    for (size_t i = 0; i < collision_objects.size(); i++) {
        auto other = collision_objects.at(i);
        if (obj.get() != other)
            world.contactPairTest(obj.get(), other, pair);
    }

    py::list out(pair.size()); // Let's try to avoid memory reallocation
    int idx = 0;
    for (auto kv : pair.m_obj_map) {
        out[idx] = py::cast(kv.second);
        idx++;
    }
    return out;
}

py::dict py_get_closest_batch(KineverseWorld& world, py::dict query) {
    py::dict out;

    for (auto item : query) {
        CollisionObjectPtr obj = item.first.cast<CollisionObjectPtr>();
        auto max_distance = item.second.cast<btScalar>();
        out[item.first] = py_get_closest(world, obj, max_distance);
    }
    return out;
}

py::list py_get_closest_filtered(KineverseWorld& world, 
                                 CollisionObjectPtr obj, py::list other_objects, btScalar max_distance = 1.0) {
    PairAccumulator<npClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), world.get_object_ptr_map());
    pair.m_closestDistanceThreshold = max_distance;

    for (py::handle py_other: other_objects) {
        KineverseCollisionObject* other = py_other.cast<KineverseCollisionObject*>();
        world.contactPairTest(obj.get(), other, pair);
    }

    py::list out(pair.size()); // Let's try to avoid memory reallocation
    int idx = 0;
    for (auto kv : pair.m_obj_map) {
        out[idx] = py::cast(kv.second);
        idx++;
    }
    return out;
}

py::dict py_get_closest_filtered_batch(KineverseWorld& world, py::dict query) {
    py::dict out;

    for (auto item : query) {
        CollisionObjectPtr obj = item.first.cast<CollisionObjectPtr>();
        py::tuple query_tuple = item.second.cast<py::tuple>();
        if (query_tuple.size() != 2)
            throw std::runtime_error(string_format("Query tuple associated with '%s' in query batch "
                                                   "should be of length 2 but is of length %d", 
                                                   item.first.attr("__str__")().cast<std::string>().c_str(), 
                                                   query_tuple.size()).c_str());
        out[item.first] = py_get_closest_filtered(world, obj, query_tuple[0], query_tuple[1].cast<btScalar>());
    }
    return out;
}

py::list py_get_closest_filtered_POD(KineverseWorld& world, 
                                     CollisionObjectPtr obj, py::list query) {
    PairAccumulator<npClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), world.get_object_ptr_map());

    for (py::handle item: query) {
        auto q_tuple = item.cast<py::tuple>();
        pair.m_closestDistanceThreshold = q_tuple[1].cast<btScalar>();
        KineverseCollisionObject* other = q_tuple[0].cast<KineverseCollisionObject*>();
        world.contactPairTest(obj.get(), other, pair);
    }

    py::list out(pair.size()); // Let's try to avoid memory reallocation
    int idx = 0;
    for (auto kv : pair.m_obj_map) {
        out[idx] = py::cast(kv.second);
        idx++;
    }
    return out;
}

py::dict py_get_closest_filtered_POD_batch(KineverseWorld& world, py::dict query) {
    py::dict out;

    for (auto item : query) {
        CollisionObjectPtr obj = item.first.cast<CollisionObjectPtr>();
        py::list query_list = item.second.cast<py::list>();
        out[item.first] = py_get_closest_filtered_POD(world, obj, query_list);
    }
    return out;
}

std::vector<btClosestPair> py_check_collision(KineverseWorld& world, CollisionObjectPtr obj, CollisionObjectPtr obj2, btScalar max_distance = 1.0) {
    PairAccumulator<btClosestPair> pair(std::const_pointer_cast<const KineverseCollisionObject>(obj), world.get_object_ptr_map());
    pair.m_closestDistanceThreshold = max_distance;

    world.contactPairTest(obj.get(), obj2.get(), pair);

    std::vector<btClosestPair> out;
    out.reserve(pair.size());
    for (auto kv : pair.m_obj_map) {
        out.push_back(kv.second);
    }
    return out;
}

// input:  Dict[Tuple[CollisionObjectPtr, CollisionObjectPtr], float]
// output: List[Collision]
struct Collision {

    Collision(btContactPoint contact_point, 
        std::shared_ptr<const KineverseCollisionObject> obj_a, std::shared_ptr<const KineverseCollisionObject> obj_b) 
    : m_a_P_pa(np_point(contact_point.m_pointOnA))
    , m_b_P_pb(np_point(contact_point.m_pointOnB))
    , m_world_V_n(np_vector(contact_point.m_normalWorldB))
    , m_contact_distance(contact_point.m_distance)
    , m_obj_a(obj_a)
    , m_obj_b(obj_b)
    {
        btTransform world_T_a = m_obj_a->getWorldTransform();
        m_map_P_pa = np_point(world_T_a * contact_point.m_pointOnA);

        btTransform world_T_b = m_obj_b->getWorldTransform();
        m_map_P_pb = np_point(world_T_b * contact_point.m_pointOnB);

    }

    Collision(np_point a_P_pa, np_point b_P_pb, np_vector world_V_n, btScalar contact_distance, 
        std::shared_ptr<const KineverseCollisionObject> obj_a, std::shared_ptr<const KineverseCollisionObject> obj_b, 
        np_point map_P_pa, np_point map_P_pb) 
    : m_a_P_pa(a_P_pa)
    , m_b_P_pb(b_P_pb)
    , m_world_V_n(world_V_n)
    , m_contact_distance(contact_distance)
    , m_obj_a(obj_a)
    , m_obj_b(obj_b)
    , m_map_P_pb(map_P_pb)
    , m_map_P_pa(map_P_pa) {}

    np_point m_a_P_pa;
    np_point m_b_P_pb;
    np_vector m_world_V_n;
    btScalar m_contact_distance;
    std::shared_ptr<const KineverseCollisionObject> m_obj_a;
    std::shared_ptr<const KineverseCollisionObject> m_obj_b;
    np_point m_map_P_pa;
    np_point m_map_P_pb;
    py::str link_a;
    py::str link_b;
    py::str original_link_a;
    py::str original_link_b;
    bool is_external;
    np_point new_a_P_pa;
    np_point new_b_P_pb;
    np_vector new_b_V_n;

    Collision reverse(){
        return Collision(m_a_P_pa, m_b_P_pb, m_world_V_n, m_contact_distance, m_obj_a, m_obj_b, m_map_P_pa, m_map_P_pb);
    }

};

py::list py_get_closest_filtered_map_batch(KineverseWorld& world, py::dict query) {
    py::list out;

    for (auto item : query) {
        py::tuple key = item.first.cast<py::tuple>();
        CollisionObjectPtr obj_a = key[0].cast<CollisionObjectPtr>();
        CollisionObjectPtr obj_b = key[1].cast<CollisionObjectPtr>();
        std::vector<btClosestPair> closest_pair_list = py_check_collision(world, obj_a, obj_b, item.second.cast<btScalar>());
        for (auto closest_pair : closest_pair_list) {
            for (auto contact_point : closest_pair.m_points) {
                Collision c(contact_point, obj_a, obj_b);
                out.append(c);
            }
        }
    }
    return out;
}

using ShapePtr = std::shared_ptr<btCollisionShape>;


PYBIND11_MODULE(betterpybullet, m) {
    m.doc() = "Attempt at exposing bullet's collision functionality.";
    m.def("get_version", []() {
        return string_format("Better PyBullet. Built: %s %s", __DATE__, __TIME__);
    });
    m.attr("__version__") = "1.0.0";

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
        .def("to_np", &to_np<btMatrix3x3>)
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
        .def("to_np", &to_np<btVector3>)
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
        .def("inv", &btQuaternion::inverse)
        .def("to_np", &to_np<btQuaternion>)
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
        .def("to_np", &to_np<btTransform>)
        .def_static("identity", &btTransform::getIdentity)
        .def_static("from_np", from_np<btTransform>)
        .def("__repr__", (std::string (*)(const btTransform&)) &toString)
        .def(py::self *  btVector3())
        .def(py::self *  btQuaternion())
        .def(py::self *  py::self)
        .def(py::self *= py::self);

// MESH LOADING

    m.def("load_convex_shape", &load_convex_shape,
          "Loads mesh file as a convex collision shape. Supported file types .obj, .stl, .dae.",
          py::arg("filename"), 
          py::arg("single_shape") = true, 
          py::arg("use_cache") = true, 
          py::arg("shape_margin") = 0.001,
          py::arg("scaling") = btVector3(1, 1, 1));

    m.def("get_shape_filename_and_scale", &get_shape_filename_and_scale, "Returns a tuple consisting of mesh file and scale vector for a given shape, "
                                                     "if the shape is a mesh.", py::arg("shape"));
    m.def("get_shape_filename", [](const btCollisionShape* shape) {
        auto temp = get_shape_filename_and_scale(shape);
        return temp.first;
    }, "Returns a tuple consisting of mesh file "
       "and scale vector for a given shape, "
       "if the shape is a mesh.", py::arg("shape"));

    m.def("vhacd", &b3VHACD,
          "vhacd",
          py::arg("fileNameIn"), 
          py::arg("fileNameOut"), 
          py::arg("fileNameLogging") = 0,
          py::arg("concavity") = -1,
          py::arg("alpha") = -1,
          py::arg("beta") = -1,
          py::arg("gamma") = -1,
          py::arg("minVolumePerCH") = -1,
          py::arg("resolution") = -1,
          py::arg("maxNumVerticesPerCH") = -1,
          py::arg("depth") = -1,
          py::arg("planeDownsampling") = -1,
          py::arg("convexhullDownsampling") = -1,
          py::arg("pca") = -1,
          py::arg("mode") = -1,
          py::arg("convexhullApproximation") = -1);

// COLLISION OBJECTS

    py::class_<KineverseCollisionObject, CollisionObjectPtr> collision_object(m, "CollisionObject");

    collision_object.def(py::init<>())
        .def(py::init<py::object&>())
        .def("set_contact_stiffness_and_damping", &btCollisionObject::setContactStiffnessAndDamping)
        .def("set_anisotropic_friction", &btCollisionObject::setAnisotropicFriction)
        .def("has_anisotropic_friction", &btCollisionObject::hasAnisotropicFriction)
        .def("set_ignore_collision", [](KineverseCollisionObject& self, CollisionObjectPtr other, bool ignore) {
                self.setIgnoreCollisionCheck(other.get(), ignore);
            }, py::arg("other_object"), py::arg("ignore") = true)
        .def("check_collsion_override", &btCollisionObject::checkCollideWithOverride)
        .def("activate", &btCollisionObject::activate)
        .def("force_activation_state", &btCollisionObject::forceActivationState)
        .def("__str__", [](const KineverseCollisionObject& obj) {
            return py::str(obj.name);
        })
        .def("__repr__", [](const KineverseCollisionObject& obj) {
            return py::str(obj.name);
        })
        .def("__eq__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name.equal(other.name);
        })
        .def("__ne__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name.not_equal(other.name);
        })
        .def("__le__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name <= other.name;
        })
        .def("__ge__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name >= other.name;
        })
        .def("__lt__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name < other.name;
        })
        .def("__gt__", [](const KineverseCollisionObject& self, const KineverseCollisionObject& other) {
            return self.name > other.name;
        })
        .def("__hash__", [](const KineverseCollisionObject& self) {
            return py::hash(self.name);
        })
        .def_property("transform", (btTransform& (btCollisionObject::*)()) &btCollisionObject::getWorldTransform, &btCollisionObject::setWorldTransform)
        .def_property("np_transform", [](const KineverseCollisionObject& o) {
            return to_np(o.getWorldTransform());
        }, [](KineverseCollisionObject& o, py::array_t<btScalar> np_pose) {
            o.setWorldTransform(from_np<btTransform>(np_pose));
        })
        .def("compound_transform", [](const KineverseCollisionObject& self, int shape_idx) {
            py::module geometry_msgs = py::module::import("geometry_msgs.msg");
            std::shared_ptr<btCompoundShape> shape = std::dynamic_pointer_cast<btCompoundShape>(self.getCollisionShape());
            btTransform o_T_geo = shape->getChildTransform(shape_idx);
            btTransform map_T_o = self.getWorldTransform();
            btTransform map_T_geo = map_T_o * o_T_geo;
            btVector3& p = map_T_geo.getOrigin();
            btQuaternion q = map_T_geo.getRotation();

            py::object position = geometry_msgs.attr("Point")(p.x(), p.y(), p.z());
            py::object orientation = geometry_msgs.attr("Quaternion")(q.x(), q.y(), q.z(), q.w());
            return geometry_msgs.attr("Pose")(position, orientation);
        }, py::arg("shape_idx") = 0)
        .def_property_readonly("np_inv_transform", [](const KineverseCollisionObject& o) {
            return to_np(o.getWorldTransform().inverse());
        })
        .def_property("deactivation_time", &btCollisionObject::getDeactivationTime, &btCollisionObject::setDeactivationTime)
        .def_property("activation_state", &btCollisionObject::getActivationState, &btCollisionObject::setActivationState)
        .def_property("contact_processing_threshold", &btCollisionObject::getContactProcessingThreshold, &btCollisionObject::setContactProcessingThreshold)
        .def_property("collision_shape", &KineverseCollisionObject::getCollisionShape, 
                                         (void (KineverseCollisionObject:: *)(ShapePtr)) &KineverseCollisionObject::setCollisionShape)
        .def_property("friction",          &btCollisionObject::getFriction, &btCollisionObject::setFriction)
        .def_readwrite("name", &KineverseCollisionObject::name)
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
        .def("get_closest", &py_get_closest, py::arg("object"), py::arg("max_distance"))
        .def("get_distance", &KineverseWorld::get_distance<npContactPair, npContactPoint>, py::arg("object_a"), py::arg("object_b"), py::arg("max_distance") = 1.0)
        .def("get_contacts", &KineverseWorld::get_contacts<npContactPair, npContactPoint>)
        .def("get_closest_batch", &py_get_closest_batch, "Performs a batched determination of closest object. Parameter maps objects to their max distances", py::arg("max_distances"))
        .def("get_closest_filtered", &py_get_closest_filtered, "Performs a distance check of the first given object against a list of other objects.", py::arg("obj_a"), py::arg("objects"), py::arg("max_distance"))
        .def("get_closest_filtered_batch", &py_get_closest_filtered_batch, "Batches the checks for closest objects w.r.t to a set of given objects.", py::arg("query"))
        .def("get_closest_filtered_POD", &py_get_closest_filtered_POD, "Performs a distance check of the first given object against a list of other objects.", py::arg("obj_a"), py::arg("object_queries"))
        .def("get_closest_filtered_POD_batch", &py_get_closest_filtered_POD_batch, "Batches the checks for closest objects w.r.t to a set of given objects.", py::arg("query"))
        .def("get_closest_filtered_map_batch", &py_get_closest_filtered_map_batch, "Batches the checks for closest objects w.r.t to a set of given objects.", py::arg("query"))
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

    py::class_<npContactPoint>(m, "ContactPoint")
        .def(py::init<const btVector3&, const btVector3&, const btVector3&, btScalar>())
        .def_readonly("point_a", &npContactPoint::m_pointOnA)
        .def_readonly("point_b", &npContactPoint::m_pointOnB)
        .def_readonly("normal_world_b", &npContactPoint::m_normalWorldB)
        .def_readonly("distance", &npContactPoint::m_distance);

    py::class_<npContactPair>(m, "ContactPair")
        .def(py::init<std::shared_ptr<const KineverseCollisionObject>, 
                      std::shared_ptr<const KineverseCollisionObject>>())
        .def_readonly("obj_a", &npContactPair::m_obj_a)
        .def_readonly("obj_b", &npContactPair::m_obj_b)
        .def_readonly("points", &npContactPair::m_points);

    py::class_<npClosestPair, npContactPair>(m, "ClosestPair");

    py::class_<Collision>(m, "Collision")
        .def("reverse", &Collision::reverse)
        .def_readonly("a_P_pa", &Collision::m_a_P_pa)
        .def_readonly("b_P_pb", &Collision::m_b_P_pb)
        .def_readonly("world_V_n", &Collision::m_world_V_n)
        .def_readonly("obj_a", &Collision::m_obj_a)
        .def_readonly("obj_b", &Collision::m_obj_b)
        .def_readonly("map_P_pa", &Collision::m_map_P_pa)
        .def_readonly("map_P_pb", &Collision::m_map_P_pb)
        .def_readonly("link_a", &Collision::link_a)
        .def_readonly("link_b", &Collision::link_b)
        .def_readonly("original_link_a", &Collision::original_link_a)
        .def_readonly("original_link_b", &Collision::original_link_b)
        .def_readonly("is_external", &Collision::is_external)
        .def_readonly("new_a_P_pa", &Collision::new_a_P_pa)
        .def_readonly("new_b_P_pb", &Collision::new_b_P_pb)
        .def_readonly("new_b_V_n", &Collision::new_b_V_n)
        .def_readonly("contact_distance", &Collision::m_contact_distance);

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
            return get_shape_filename_and_scale(shape.get()).first;
        })
        .def_property_readonly("scaling", [](ShapePtr shape) {
            return get_shape_filename_and_scale(shape.get()).second;
        })
        .def_property_readonly("child_shapes", &KineverseCompoundShape::get_child_shapes);

    py::class_<btConvexHullShape, btPolyhedralConvexShape, std::shared_ptr<btConvexHullShape>>(m, "ConvexHullShape")
        .def(py::init<>())
        .def("add_point", &btConvexHullShape::addPoint)
        .def("optimize_hull", &btConvexHullShape::optimizeConvexHull)
        .def_property_readonly("npoints", &btConvexHullShape::getNumPoints)
        .def_property_readonly("points", [](const btConvexHullShape& shape) {
            std::vector<btVector3> out;
            out.resize(shape.getNumVertices());

            for (int i = 0; i < shape.getNumVertices(); i++)
                shape.getVertex(i, out[i]);

            return out;
        })
        .def_property_readonly("edges", [](const btConvexHullShape& shape) {
            std::vector<btVector3> out;
            out.resize(shape.getNumEdges() * 2);

            for (int i = 0; i < shape.getNumEdges(); i++)
                shape.getEdge(i, out[i * 2], out[i * 2 + 1]);

            return out;
        })
        .def_property_readonly("file_path", [](btCollisionShape* shape) {
            return get_shape_filename_and_scale(shape).first;
        });
}
