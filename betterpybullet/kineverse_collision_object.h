#pragma once

#include <memory>

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class KineverseCollisionObject : public btCollisionObject {

 public:
   KineverseCollisionObject() : btCollisionObject() {
      // name = py::str('asdf');
   }

   KineverseCollisionObject(py::object name_) : btCollisionObject() {
      name = name_;
   }

   virtual void setCollisionShape(btCollisionShape* collisionShape);
   virtual void setCollisionShape(std::shared_ptr<btCollisionShape> collisionShape);

   inline std::shared_ptr<btCollisionShape> getCollisionShape() const {
        return m_collision_shape_ptr;
   }


   py::object name;
   
 private:
   std::shared_ptr<btCollisionShape> m_collision_shape_ptr;
};
