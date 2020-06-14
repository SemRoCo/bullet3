#include <iostream>
#include <memory>

#include "kineverse_collision_object.h"
#include "kineverse_query.h"
#include "kineverse_world.h"
#include "kineverse_mesh_loader.h"
#include "kineverse_utils.h"

std::ostream& operator<<(std::ostream& s, const ContactPoint& p) {
    s << "    on A:\n" << toString(p.m_pointOnA) << '\n'
      << "    on B:\n" << toString(p.m_pointOnB) << '\n'
      << "  normal:\n" << toString(p.m_normalWorldB) << '\n'
      << "distance: " << p.m_distance;
    return s;
}

std::ostream& operator<<(std::ostream& s, const ContactPair& p) {
    s << "Contact between " << p.m_obj_a << " and " << p.m_obj_b << '\n';
    for (const auto& cp: p.m_points)
        s << cp << '\n';
    return s;
}

template<typename A>
std::ostream& operator<<(std::ostream& s, const PairAccumulator<A>& pa) {
    s << "Contacts of " << pa.m_obj << '\n';
    for (const auto& cp: pa.m_obj_map)
        s << cp.second << '\n';
    return s;
}


int main(int argc, char const *argv[]) {
    
    if (argc < 2) {
        std::cout << "Need path to mesh directory" << std::endl;
        return 0;
    }

    ContactPair pair1;
    ClosestPair pair2;

    std::cout << "lol" << std::endl;

    KineverseWorld world;

    auto shape_box        = std::make_shared<btBoxShape>(btVector3(1.0f, 1.0f, 1.0f));
    auto shape_cylinder_z = std::make_shared<btCylinderShapeZ>(btVector3(1.0f, 1.0f, 1.0f));
    auto shape_cone       = std::make_shared<btConeShape>(0.5f, 1.0f);
    auto shape_ball_joint = load_convex_shape(string_format("%s/%s", argv[1], "ball_joint.obj").c_str());
    auto shape_piston     = load_convex_shape(string_format("%s/%s", argv[1], "piston.obj").c_str());

    auto obj_box = std::make_shared<KineverseCollisionObject>();
    obj_box->setCollisionShape(shape_box);
    obj_box->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(1, 1, 0)));

    auto obj_cylinder = std::make_shared<KineverseCollisionObject>();
    obj_cylinder->setCollisionShape(shape_cylinder_z);    
    obj_cylinder->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(-1, 1, 0)));

    auto obj_cone = std::make_shared<KineverseCollisionObject>();
    obj_cone->setCollisionShape(shape_cone);
    obj_cone->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(1, -1, 0)));

    auto obj_ball_joint = std::make_shared<KineverseCollisionObject>();
    obj_ball_joint->setCollisionShape(shape_ball_joint);
    obj_ball_joint->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(-1, -1, 0)));

    auto obj_piston = std::make_shared<KineverseCollisionObject>();
    obj_piston->setCollisionShape(shape_piston);
    obj_piston->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(-1, -1, 2)));

    world.addCollisionObject(obj_box);
    world.addCollisionObject(obj_cylinder);
    world.addCollisionObject(obj_cone);
    world.addCollisionObject(obj_ball_joint);
    world.addCollisionObject(obj_piston);
    world.updateAabbs();

    for (int i = 0; i < 100; i++) {
        auto closest_pairs = world.get_closest_batch({{obj_piston,     4.0f},
                                                      {obj_ball_joint, 4.0f},
                                                      {obj_box,        4.0f}});
        for (const auto& pair: closest_pairs)
            for (const auto& cp: pair.second)
                std::cout << cp << std::endl;
    }

    return 0;
}
