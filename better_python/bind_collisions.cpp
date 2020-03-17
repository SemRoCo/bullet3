#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <btBulletCollisionCommon.h>
#include <Wavefront/tiny_obj_loader.h>

#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr

#include <stdarg.h>  // For va_start, etc.

#include <iostream>


typedef std::shared_ptr<btCollisionShape>         Shape_ptr;
typedef std::shared_ptr<btCollisionObject>        Object_ptr;
typedef std::shared_ptr<btDispatcher>             Dispatcher_ptr;
typedef std::shared_ptr<btBroadphaseInterface>    Broadphase_ptr;
typedef std::shared_ptr<btCollisionConfiguration> CollisionConfiguration_ptr;

//typedef std::shared_ptr<btCollisionShape> Shape_ptr;

class Smart_btCollisionObject : public btCollisionObject {
protected:
    Shape_ptr m_smart_collisionShape;
    std::unordered_set<Object_ptr> m_smart_ignoreset;

public:
    virtual void setCollisionShape(Shape_ptr collisionShape) {
        m_smart_collisionShape = collisionShape;
        btCollisionObject::setCollisionShape(m_smart_collisionShape.get());
    }

    virtual Shape_ptr getCollisionShape() {
        return m_smart_collisionShape;
    }

    void setIgnoreCollisionCheck(Object_ptr co, bool ignoreCollisionCheck) {
        if (ignoreCollisionCheck)
            m_smart_ignoreset.insert(co);
        else
            m_smart_ignoreset.erase(co);
        btCollisionObject::setIgnoreCollisionCheck(co.get(), ignoreCollisionCheck);
    }
};


class Smart_btCollisionWorld : public btCollisionWorld {
protected:
    std::unordered_set<Object_ptr> m_smart_collision_objects;
    Dispatcher_ptr m_smart_dispatcher;
    Broadphase_ptr m_smart_broadphase_pair_cache;
    CollisionConfiguration_ptr m_smart_configuration;

public:
    Smart_btCollisionWorld(Dispatcher_ptr dispatcher, Broadphase_ptr broadphase, CollisionConfiguration_ptr configuration) 
    : m_smart_dispatcher(dispatcher)
    , m_smart_broadphase_pair_cache(broadphase)
    , m_smart_configuration(configuration)
    , btCollisionWorld(dispatcher.get(), broadphase.get(), configuration.get()) {}

    void setBroadphase(Broadphase_ptr pairCache)
    {
        m_smart_broadphase_pair_cache = pairCache;
    }

    const Broadphase_ptr getBroadphase() const
    {
        return m_smart_broadphase_pair_cache;
    }

    Broadphase_ptr getBroadphase()
    {
        return m_smart_broadphase_pair_cache;
    }

    Dispatcher_ptr   getDispatcher()
    {
        return m_smart_dispatcher;
    }

    const Dispatcher_ptr getDispatcher() const
    {
        return m_smart_dispatcher;
    }

    std::vector<Object_ptr> get_collision_objects() {
        return std::vector<Object_ptr>(m_smart_collision_objects.begin(), m_smart_collision_objects.end());
    }

    virtual void addCollisionObject(Object_ptr collisionObject, int collisionFilterGroup=btBroadphaseProxy::DefaultFilter, int collisionFilterMask=btBroadphaseProxy::AllFilter) {
        m_smart_collision_objects.insert(collisionObject);
        btCollisionWorld::addCollisionObject(collisionObject.get(), collisionFilterGroup, collisionFilterMask);
    }

    virtual void removeCollisionObject(Object_ptr collisionObject) {
        m_smart_collision_objects.erase(collisionObject);
        btCollisionWorld::removeCollisionObject(collisionObject.get());
    }
};

class Smart_btCompoundShape : public btCompoundShape {
protected:
    std::vector<Shape_ptr> m_smart_children;

public:

    void addChildShape(const btTransform& localTransform, Shape_ptr shape) {
        m_smart_children.push_back(shape);
        btCompoundShape::addChildShape(localTransform, shape.get());
    }

    virtual void removeChildShape(Shape_ptr shape) {
        btCompoundShape::removeChildShape(shape.get());
        std::remove(m_smart_children.begin(), m_smart_children.end(), shape);
    }

    void removeChildShapeByIndex(int childShapeindex) {
        btCompoundShape::removeChildShapeByIndex(childShapeindex);
        m_smart_children.erase(m_smart_children.begin() + childShapeindex);
    }

    Shape_ptr getChildShape(int index) {
        return m_smart_children[index];
    }

    const Shape_ptr getChildShape(int index) const {
        return m_smart_children[index];
    }
};

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

std::shared_ptr<tinyobj::mesh_t> load_stl_mesh(const std::string& filename) {
    FILE* file = fopen(filename.c_str(),"rb");
    std::shared_ptr<tinyobj::mesh_t> out(new tinyobj::mesh_t());
    if (file) {
        int size=0;
        if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {
            throw std::runtime_error(string_format("Error: Cannot access file to determine size of %s\n", filename.c_str()));
        } else if (size) {
            //b3Warning("Open STL file of %d bytes\n",size);
            char* memoryBuffer = new char[size+1];
            int actualBytesRead = fread(memoryBuffer,1,size,file);
            if (actualBytesRead!=size) {
                throw std::runtime_error(string_format("Error reading from file %s", filename.c_str()));
            } else {
                int numTriangles = *(int*)&memoryBuffer[80];
                
                if (numTriangles) {
                    //perform a sanity check instead of crashing on invalid triangles/STL files
                    int expectedBinaryFileSize = numTriangles* 50 + 84;
                    if (expectedBinaryFileSize != size) {
                        delete[] memoryBuffer;
                        throw std::runtime_error(string_format("STL file has wrong size! Expected size is %d actual size is %d", expectedBinaryFileSize, size));
                    }

                    out->positions.reserve(numTriangles * 3 * 3);
                    out->normals.reserve(numTriangles * 3);
                    for (int i=0;i<numTriangles;i++) {
                        char* curPtr = &memoryBuffer[84+i*50];
                        float* float_ptr = (float*)curPtr;
                        //MySTLTriangle tmp;
                        //memcpy(&tmp,curPtr,sizeof(MySTLTriangle));
                        
                        for (int v=0; v<3; v++)
                            out->normals.push_back(*(float_ptr + v));

                        for (int v=3; v < 12; v++)
                            out->positions.push_back(*(float_ptr + v));
                    }
                }
            }
            
            delete[] memoryBuffer;
        }
        fclose(file);
    }
    return out;
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

template <typename T>
std::vector<T> aligned_array_to_stl(const btAlignedObjectArray<T>& array) {
    std::vector<T> out;
    out.reserve(array.size());
    for (int i = 0; i < array.size(); i++)
        out.push_back(array[i]);
    return out;
}

namespace py = pybind11;

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
};


struct ContactPair : public btCollisionWorld::ContactResultCallback {
    ContactPair()
    : m_obj_a(0)
    , m_obj_b(0)
    {}

    ContactPair(const btCollisionObject* obj_a,
                const btCollisionObject* obj_b) 
    : m_obj_a(obj_a)
    , m_obj_b(obj_b) {}  

    virtual bool needsCollision(btBroadphaseProxy* proxy0) const { return true; }
    
    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
        m_points.push_back(ContactPoint(cp.m_localPointA, cp.m_localPointB, cp.m_normalWorldOnB, cp.getDistance()));

        return 1;
    }

    const btCollisionObject* m_obj_a;
    const btCollisionObject* m_obj_b;
    std::vector<ContactPoint> m_points;
};

struct ClosestPair : public ContactPair {
    ClosestPair()
    : ContactPair()
    {}

    ClosestPair(const btCollisionObject* obj_a,
                const btCollisionObject* obj_b) 
    : ContactPair(obj_a, obj_b) {}

    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
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
};

template <typename A>
struct PairAccumulator : public btCollisionWorld::ContactResultCallback {
    PairAccumulator() : m_obj(0) {}
    PairAccumulator(const btCollisionObject* obj) : m_obj(obj) {}

    virtual bool needsCollision(btBroadphaseProxy* proxy0) const { return true; }
    
    virtual btScalar addSingleResult(btManifoldPoint& cp, 
                        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
                        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
        
        if (m_obj != colObj0Wrap->m_collisionObject) {
            if (m_obj == colObj1Wrap->m_collisionObject) {
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

        if (m_obj_map.find(colObj1Wrap->m_collisionObject) == m_obj_map.end())
            m_obj_map[colObj1Wrap->m_collisionObject] = A(m_obj, colObj1Wrap->m_collisionObject);


        m_obj_map[colObj1Wrap->m_collisionObject].addSingleResult(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1);

        return 1;
    }

    int size() {
        return m_obj_map.size();
    }

    std::unordered_map<const btCollisionObject*, A> m_obj_map;
    const btCollisionObject* m_obj;
};


struct AABBBroadphaseCallback : public btBroadphaseAabbCallback {
    virtual bool process(const btBroadphaseProxy* proxy) {
        m_objects.push_back((btCollisionObject*)proxy->m_clientObject);
        return true;
    }

    std::vector<btCollisionObject*> m_objects;
};


class KineverseWorld : public btCollisionWorld {
private:
    btDbvtBroadphase         m_bpinterface;
    btDefaultCollisionConfiguration m_collision_configuration;
    std::shared_ptr<btDispatcher> m_dispatcher;

public:
    // KineverseWorld()
    // : Smart_btCollisionWorld(
    //     Dispatcher_ptr(), 
    //     Broadphase_ptr(new btDbvtBroadphase()), 
    //     CollisionConfiguration_ptr(new btDefaultCollisionConfiguration())) {
        
    //     m_smart_dispatcher = Dispatcher_ptr(new btCollisionDispatcher(m_smart_configuration.get()));
    //     m_dispatcher1 = m_smart_dispatcher.get();
    // }

    KineverseWorld()
    : btCollisionWorld(0, 0, 0) {        
        m_dispatcher = Dispatcher_ptr(new btCollisionDispatcher(&m_collision_configuration));
        m_broadphasePairCache = &m_bpinterface;
        m_dispatcher1 = m_dispatcher.get();
    }

    template<typename T>
    T closest_ray_test(const btVector3& from, const btVector3& to) {
        T out(from, to);
        rayTest(from, to, out);
        return out;
    }

    template<typename T>
    std::vector<T> closest_ray_batch_test(std::vector<btVector3> from, std::vector<btVector3> to) {
        std::vector<T> out;
        out.reserve(from.size());
        for (int i = 0; i < from.size(); i++) {
            out.push_back(T(from[i], to[i]));
            rayTest(from[i], to[i], out[i]);
        }
        return out;
    }

    std::vector<ContactPair> get_contacts() {
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
                std::cout << "At least one of the body pointers is zero. BodyA: " << body_a << " BodyB: " << body_b << std::endl;
            }

            out.push_back(ContactPair(body_a, body_b));
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

    std::vector<ClosestPair> get_closest(btCollisionObject* obj, btScalar max_distance=1.f) {
        PairAccumulator<ClosestPair> pair(obj);
        pair.m_closestDistanceThreshold = max_distance;
        for (int i = 0; i < m_collisionObjects.size(); i++) {
            btCollisionObject* other_object = m_collisionObjects[i];
            if (other_object != obj) {
                //printf("obj: %x other_object: %x\n", obj, other_object);
                contactPairTest(obj, other_object, pair);
            }
        }

        std::vector<ClosestPair> out;
        out.reserve(pair.size());
        for (auto kv : pair.m_obj_map) {
            out.push_back(kv.second);
        }
        return out;
    }

    std::unordered_map<btCollisionObject*, std::vector<ClosestPair>> get_closest_batch(std::unordered_map<btCollisionObject*, btScalar> params) {
        std::unordered_map<btCollisionObject*, std::vector<ClosestPair>> out;
        for (auto kv : params) {
            out[kv.first] = get_closest(kv.first, kv.second);
        }
        return out;
    }

    
    std::vector<btCollisionObject*> overlap_aabb(const btVector3& aabb_min, const btVector3& aabb_max) {
        AABBBroadphaseCallback accumulator;
        m_broadphasePairCache->aabbTest(aabb_min, aabb_max, accumulator);
        return accumulator.m_objects;
    }

    void batch_set_transforms(std::vector<btCollisionObject*> objects, py::array_t<btScalar> poses) {
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
};


btTransform create_translation(btScalar x, btScalar y, btScalar z) {
    return btTransform(btQuaternion::getIdentity(), btVector3(x,y,z));
}


std::vector<std::shared_ptr<btCollisionShape>> m_flat_shape_cache;
std::unordered_map<std::string, std::shared_ptr<btCollisionShape>> m_convex_shape_cache;
std::unordered_map<const btCollisionShape*, std::string> collision_shape_filenames;

btCollisionShape* load_convex_shape(std::string filename, bool use_cache = true) {
    if (use_cache && m_convex_shape_cache.find(filename) != m_convex_shape_cache.end())
        return m_convex_shape_cache[filename].get();


    std::string lc_filename = filename;
    std::transform(lc_filename.begin(), lc_filename.end(), lc_filename.begin(), [](unsigned char c) { return std::tolower(c); });

    std::shared_ptr<btCollisionShape> shape_ptr;
    if (lc_filename.substr(lc_filename.size() - 4) == ".obj") {
        std::vector<tinyobj::shape_t> shapes;
        std::string error_msg = tinyobj::LoadObj(shapes, filename.c_str());
        for (tinyobj::shape_t& shape : shapes) {
            std::shared_ptr<btConvexHullShape> new_shape(new btConvexHullShape());
            for (int i = 0; i < shape.mesh.positions.size(); i += 3) 
                new_shape->addPoint(btVector3(shape.mesh.positions[i], shape.mesh.positions[i + 1], shape.mesh.positions[i + 2]), i == shape.mesh.positions.size() - 3);
            new_shape->optimizeConvexHull();
            m_flat_shape_cache.push_back(new_shape);
        }
        
        if (shapes.size() == 1) {
            shape_ptr = m_flat_shape_cache[m_flat_shape_cache.size() - 1];
        } else {
            std::shared_ptr<btCompoundShape> compound_ptr(new btCompoundShape());
            for (int i = m_flat_shape_cache.size() - shapes.size(); i < m_flat_shape_cache.size(); i++)
                compound_ptr->addChildShape(btTransform::getIdentity(), m_flat_shape_cache[i].get());
            shape_ptr = compound_ptr;
        }
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".stl") {
        auto stl_mesh = load_stl_mesh(filename);
        std::shared_ptr<btConvexHullShape> hull_ptr(new btConvexHullShape());
        for (int i = 0; i < stl_mesh->positions.size(); i += 3) 
            hull_ptr->addPoint(btVector3(stl_mesh->positions[i], stl_mesh->positions[i + 1], stl_mesh->positions[i + 2]), i == stl_mesh->positions.size() - 3);
        hull_ptr->optimizeConvexHull();
        shape_ptr = hull_ptr;
    } else if (lc_filename.substr(lc_filename.size() - 4) == ".dae") {
        throw std::runtime_error(string_format(".dae export is not yet supported. File: %s", filename).c_str());
    } else {
        throw std::runtime_error(string_format("File '%s' is not .obj, .stl, or .dae", filename.c_str()));
    }

    m_convex_shape_cache[filename] = shape_ptr;
    collision_shape_filenames[shape_ptr.get()] = filename;
    return shape_ptr.get();
}

std::string get_shape_filename(const btCollisionShape* shape) {
    if (collision_shape_filenames.find(shape) != collision_shape_filenames.end())
        return collision_shape_filenames[shape];
    return "";
}


PYBIND11_MODULE(betterpybullet, m) {
    m.doc() = "Attempt at exposing bullet's collision functionality.";


// MESH LOADING

    m.def("load_convex_shape", &load_convex_shape, "Loads mesh file as a convex collision shape. Supported file types .obj, .stl, .dae.", py::arg("filename"), py::arg("use_cache") = true);

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
        .def("__getitem__", [](const btVector3& v, int idx) {
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



    py::class_<btCollisionObject> collision_object(m, "CollisionObject");

    collision_object.def(py::init<>())
        .def("set_collision_shape", &btCollisionObject::setCollisionShape)
        .def("set_contact_stiffness_and_damping", &btCollisionObject::setContactStiffnessAndDamping)
        .def("set_anisotropic_friction", &btCollisionObject::setAnisotropicFriction)
        .def("has_anisotropic_friction", &btCollisionObject::hasAnisotropicFriction)
        .def("set_ignore_collision", &btCollisionObject::setIgnoreCollisionCheck, py::arg("other_object"), py::arg("ignore") = true)
        .def("check_collsion_override", &btCollisionObject::checkCollideWithOverride)
        .def("activate", &btCollisionObject::activate)
        .def("force_activation_state", &btCollisionObject::forceActivationState)
        .def_property("transform", (btTransform& (btCollisionObject::*)()) &btCollisionObject::getWorldTransform, &btCollisionObject::setWorldTransform)
        .def_property("deactivation_time", &btCollisionObject::getDeactivationTime, &btCollisionObject::setDeactivationTime)
        .def_property("activation_state", &btCollisionObject::getActivationState, &btCollisionObject::setActivationState)
        .def_property("contact_processing_threshold", &btCollisionObject::getContactProcessingThreshold, &btCollisionObject::setContactProcessingThreshold)
        .def_property("collision_shape", (btCollisionShape* (btCollisionObject::*)()) &btCollisionObject::getCollisionShape, &btCollisionObject::setCollisionShape)
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
        .def_property_readonly("aabb", [](btCollisionObject& self) {
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

    py::class_<btCollisionShape>(m, "CollisionShape")
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


    py::class_<btCollisionWorld> collision_world(m, "CollisionWorld");

    collision_world
        .def("update_single_aabb", &btCollisionWorld::updateSingleAabb)
        .def("update_aabbs", &btCollisionWorld::updateAabbs)
        .def("compute_overlapping_pairs", &btCollisionWorld::computeOverlappingPairs)
        .def("debug_draw_world", &btCollisionWorld::debugDrawWorld)
        .def("debug_draw_object", &btCollisionWorld::debugDrawObject)
        .def("num_collision_objects", &btCollisionWorld::getNumCollisionObjects)
        .def("add_collision_object", &btCollisionWorld::addCollisionObject, py::arg("object"), py::arg("filter_group") = 1, py::arg("filter_mask") = -1)
        .def("remove_collision_object", &btCollisionWorld::removeCollisionObject)
        .def("perform_discrete_collision_detection", &btCollisionWorld::performDiscreteCollisionDetection)
        .def_property("force_update_all_aabbs", &btCollisionWorld::getForceUpdateAllAabbs, &btCollisionWorld::setForceUpdateAllAabbs)
        .def_property_readonly("collision_objects", [](const btCollisionWorld& w) {
            return aligned_array_to_stl(w.getCollisionObjectArray());
        })
        .def("add_collision_object", [](btCollisionWorld& self, btCollisionShape* shape, const btTransform& t) {
            btCollisionObject* out = new btCollisionObject();
            out->setCollisionShape(shape);
            out->setWorldTransform(t);
            self.addCollisionObject(out);
            return out;
        }, py::arg("shape"), py::arg("transform") = btTransform::getIdentity());


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

    py::class_<KineverseWorld, btCollisionWorld>(m, "KineverseWorld")
        .def(py::init<>())
        .def("closest_ray_test", &KineverseWorld::closest_ray_test<btCollisionWorld::ClosestRayResultCallback>, py::arg("from"), py::arg("to"))
        .def("closest_ray_test_batch", &KineverseWorld::closest_ray_batch_test<btCollisionWorld::ClosestRayResultCallback>, py::arg("from"), py::arg("to"))
        .def("get_closest", &KineverseWorld::get_closest, py::arg("object"), py::arg("max_distance"))
        .def("get_contacts", &KineverseWorld::get_contacts)
        .def("get_closest_batch", &KineverseWorld::get_closest_batch, "Performs a batched determination of closest object. Parameter maps objects to their max distances", py::arg("max_distances"))
        .def("batch_set_transforms", &KineverseWorld::batch_set_transforms, py::arg("objects"), py::arg("transforms_matrix"))
        .def("overlap_aabb", &KineverseWorld::overlap_aabb, py::arg("aabb_min"), py::arg("aabb_max"));

    py::class_<ContactPoint>(m, "ContactPoint")
        .def(py::init<const btVector3&, const btVector3&, const btVector3&, btScalar>())
        .def_readonly("point_a", &ContactPoint::m_pointOnA)
        .def_readonly("point_b", &ContactPoint::m_pointOnB)
        .def_readonly("normal_world_b", &ContactPoint::m_normalWorldB)
        .def_readonly("distance", &ContactPoint::m_distance);

    py::class_<ContactPair>(m, "ContactPair")
        .def(py::init<const btCollisionObject*, const btCollisionObject*>())
        .def_readonly("obj_a", &ContactPair::m_obj_a)
        .def_readonly("obj_b", &ContactPair::m_obj_b)
        .def_readonly("points", &ContactPair::m_points);

    py::class_<ClosestPair, ContactPair>(m, "ClosestPair");

//////// SHAPES

    py::class_<btConvexShape, btCollisionShape>(m, "ConvexShape")
        .def("local_supporting_vertex", &btConvexShape::localGetSupportingVertex)
        .def("local_supporting_vertex_no_margin", &btConvexShape::localGetSupportingVertexWithoutMargin)
        .def_property("margin", &btConvexShape::getMargin, &btConvexShape::setMargin)
        .def_property("scaling", &btConvexShape::getLocalScaling, &btConvexShape::setLocalScaling);


    py::class_<btPolyhedralConvexShape, btConvexShape>(m, "PolyedralConvexShape")
        .def_property_readonly("nvertices", &btPolyhedralConvexShape::getNumVertices)
        .def_property_readonly("nedges", &btPolyhedralConvexShape::getNumEdges)
        .def_property_readonly("nplanes", &btPolyhedralConvexShape::getNumPlanes)
        .def("is_inside", &btPolyhedralConvexShape::isInside);

    py::class_<btBoxShape, btPolyhedralConvexShape>(m, "BoxShape")
        .def(py::init<const btVector3&>())
        .def_property_readonly("extents", [](const btBoxShape& self) {
            return self.getHalfExtentsWithoutMargin() * 2.f;
        });

    py::class_<btCylinderShape, btConvexShape>(m, "CylinderShape")
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

    py::class_<btCylinderShapeX, btCylinderShape>(m, "CylinderShapeX")
        .def(py::init<const btVector3&>());

    py::class_<btCylinderShapeZ, btCylinderShape>(m, "CylinderShapeZ")
        .def(py::init<const btVector3&>());

    py::class_<btSphereShape, btConvexShape>(m, "SphereShape")
        .def(py::init<btScalar>(), py::arg("radius"))
        .def_property("radius", &btSphereShape::getRadius, &btSphereShape::setUnscaledRadius);

    py::class_<btCapsuleShape, btConvexShape>(m, "CapsuleShape")
        .def(py::init<btScalar, btScalar>(), py::arg("radius"), py::arg("height"))
        .def_property_readonly("radius", &btCapsuleShape::getRadius)
        .def_property_readonly("height", [](const btCapsuleShape& c) {
            return c.getHalfHeight() * 2;
        })
        .def_property_readonly("axis", &btCapsuleShape::getUpAxis);

    py::class_<btConeShape, btConvexShape>(m, "ConeShape")
        .def(py::init<btScalar, btScalar>(), py::arg("radius"), py::arg("height"))
        .def_property("radius", &btConeShape::getRadius, &btConeShape::setRadius)
        .def_property("height", &btConeShape::getHeight, &btConeShape::setHeight);

    py::class_<btCompoundShape, btCollisionShape>(m, "CompoundShape")
        .def(py::init<>())
        .def("add_child", &btCompoundShape::addChildShape, py::arg("transform"), py::arg("shape"))
        .def("remove_child", &btCompoundShape::removeChildShape, py::arg("shape"))
        .def("get_child", (btCollisionShape* (btCompoundShape::*)(int)) &btCompoundShape::getChildShape, py::arg("index"))
        .def("get_child_transform", (btTransform& (btCompoundShape::*)(int)) &btCompoundShape::getChildTransform, py::arg("index"))
        .def("update_child_transform", &btCompoundShape::updateChildTransform, py::arg("index"), py::arg("transform"), py::arg("recompute_aabb"))
        .def_property_readonly("nchildren", &btCompoundShape::getNumChildShapes)
        .def_property_readonly("file_path", [](btCollisionShape* shape) {
            return get_shape_filename(shape);
        });

    py::class_<btConvexHullShape, btPolyhedralConvexShape>(m, "ConvexHullShape")
        .def(py::init<>())
        .def("add_point", &btConvexHullShape::addPoint)
        .def("optimize_hull", &btConvexHullShape::optimizeConvexHull)
        .def_property_readonly("npoints", &btConvexHullShape::getNumPoints)
        .def_property_readonly("file_path", [](btCollisionShape* shape) {
            return get_shape_filename(shape);
        });
}
