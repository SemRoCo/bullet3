#include <iostream>
#include <memory>

#include "kineverse_collision_object.h"
#include "kineverse_compound_shape.h"
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

    auto shape_empty_compound = std::make_shared<KineverseCompoundShape>();

    auto shape_Drawer_100_29    = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_100_29.obj");
    auto shape_Drawer_60_14     = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_60_14.obj");
    auto shape_Drawer_60_29     = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_60_29.obj");
    auto shape_Drawer_60_58     = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_60_58.obj");
    auto shape_Drawer_80_14     = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_80_14.obj");
    auto shape_Drawer_80_29     = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/Drawer_80_29.obj");
    auto shape_VDrawer          = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/drawers/VDrawer.obj");
    auto shape_Handle100        = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/handles/Handle100.obj");
    auto shape_Handle60         = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/handles/Handle60.obj");
    auto shape_Handle80         = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/handles/Handle80.obj");
    auto shape_VHandle130       = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/handles/VHandle130.obj");
    auto shape_VHandle90        = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/handles/VHandle90.obj");
    auto shape_DishWasherDoor   = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/misc/DishWasherDoor.obj");
    auto shape_FridgeDoor       = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/misc/FridgeDoor.obj");
    auto shape_Knob             = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/oven/Knob.obj");
    auto shape_OvenDoor         = load_convex_shape("/home/adrian/robots_ws/src/iai_maps/iai_kitchen/meshes/oven/OvenDoor.obj");
    auto shape_caster_L         = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/base_v0/caster_L.stl");
    auto shape_forearm          = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/forearm_v0/forearm.stl");
    auto shape_wrist_flex       = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/forearm_v0/wrist_flex.stl");
    auto shape_wrist_roll_L     = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/forearm_v0/wrist_roll_L.stl");
    auto shape_gripper_palm     = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/gripper_v0/gripper_palm.stl");
    auto shape_l_finger         = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/gripper_v0/l_finger.stl");
    auto shape_l_finger_tip     = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/gripper_v0/l_finger_tip.stl");
    auto shape_head_pan_L       = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/head_v0/head_pan_L.stl");
    auto shape_head_tilt_L      = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/head_v0/head_tilt_L.stl");
    auto shape_shoulder_lift    = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/shoulder_v0/shoulder_lift.stl");
    auto shape_shoulder_pan     = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/shoulder_v0/shoulder_pan.stl");
    auto shape_upper_arm_roll_L = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/shoulder_v0/upper_arm_roll_L.stl");
    auto shape_tilting_hokuyo_L = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo_L.stl");
    auto shape_torso_lift_L     = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/torso_v0/torso_lift_L.stl");
    auto shape_elbow_flex       = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/upper_arm_v0/elbow_flex.stl");
    auto shape_forearm_roll_L   = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/upper_arm_v0/forearm_roll_L.stl");
    auto shape_upper_arm        = load_convex_shape("/opt/ros/melodic/share/pr2_description/meshes/upper_arm_v0/upper_arm.stl");

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

    auto obj_empty_compound = std::make_shared<KineverseCollisionObject>();
    obj_empty_compound->setCollisionShape(shape_empty_compound);
    obj_empty_compound->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(-1, 1, 2)));    

    // world.addCollisionObject(obj_box);
    // world.addCollisionObject(obj_cylinder);
    // world.addCollisionObject(obj_cone);
    // world.addCollisionObject(obj_ball_joint);
    // world.addCollisionObject(obj_piston);
    // world.addCollisionObject(obj_empty_compound);

    auto kitchen_iai_fridge_door_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_iai_fridge_door_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.185000, -0.790000, 0.985000)));
    kitchen_iai_fridge_door_handle->setCollisionShape(shape_VHandle90);
    auto kitchen_kitchen_island_middle_lower_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_middle_lower_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 1.719200, 0.245000)));
    kitchen_kitchen_island_middle_lower_drawer_main->setCollisionShape(shape_Drawer_100_29);
    auto pr2_r_wrist_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_wrist_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.771000, -0.188000, 0.790675)));
    pr2_r_wrist_roll_link->setCollisionShape(shape_wrist_roll_L);
    auto kitchen_kitchen_island_left_upper_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_left_upper_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 0.919200, 0.645000)));
    kitchen_kitchen_island_left_upper_drawer_handle->setCollisionShape(shape_Handle60);
    auto pr2_r_forearm_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_forearm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, -0.188000, 0.790675)));
    pr2_r_forearm_link->setCollisionShape(shape_forearm);
    auto kitchen_oven_area_oven_knob_oven = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_knob_oven->setWorldTransform(btTransform(btQuaternion(0.707107, 0.000000, 0.707107, -0.000000), btVector3(1.245000, 1.740000, 1.414900)));
    kitchen_oven_area_oven_knob_oven->setCollisionShape(shape_Knob);
    auto kitchen_oven_area_area_right_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_right_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.515000, 1.450000, 0.815000)));
    kitchen_oven_area_area_right_drawer_main->setCollisionShape(shape_VDrawer);
    auto pr2_l_elbow_flex_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_elbow_flex_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, 0.188000, 0.790675)));
    pr2_l_elbow_flex_link->setCollisionShape(shape_elbow_flex);
    auto kitchen_oven_area_oven_knob_stove_2 = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_knob_stove_2->setWorldTransform(btTransform(btQuaternion(0.707107, 0.000000, 0.707107, -0.000000), btVector3(1.245000, 2.040000, 1.414900)));
    kitchen_oven_area_oven_knob_stove_2->setCollisionShape(shape_Knob);
    auto pr2_r_gripper_l_finger_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_gripper_l_finger_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.847910, -0.178000, 0.790675)));
    pr2_r_gripper_l_finger_link->setCollisionShape(shape_l_finger);
    auto kitchen_sink_area_left_middle_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_middle_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.192500, 0.870000, 0.630000)));
    kitchen_sink_area_left_middle_drawer_handle->setCollisionShape(shape_Handle80);
    auto pr2_r_upper_arm_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_upper_arm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, -0.188000, 0.790675)));
    pr2_r_upper_arm_link->setCollisionShape(shape_upper_arm);
    auto kitchen_kitchen_island_middle_upper_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_middle_upper_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 1.719200, 0.535000)));
    kitchen_kitchen_island_middle_upper_drawer_main->setCollisionShape(shape_Drawer_100_29);
    auto kitchen_sink_area_left_bottom_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_bottom_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.525000, 0.870000, 0.230000)));
    kitchen_sink_area_left_bottom_drawer_main->setCollisionShape(shape_Drawer_80_29);
    auto pr2_l_shoulder_pan_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_shoulder_pan_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.050000, 0.188000, 0.790675)));
    pr2_l_shoulder_pan_link->setCollisionShape(shape_shoulder_pan);
    auto kitchen_sink_area_dish_washer_door_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_dish_washer_door_handle->setWorldTransform(btTransform(btQuaternion(-0.000000, 1.000000, -0.000000, 0.000000), btVector3(1.192500, 0.170000, 0.752500)));
    kitchen_sink_area_dish_washer_door_handle->setCollisionShape(shape_Handle60);
    auto pr2_fl_caster_rotation_link = std::make_shared<KineverseCollisionObject>();
    pr2_fl_caster_rotation_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.224600, 0.224600, 0.079200)));
    pr2_fl_caster_rotation_link->setCollisionShape(shape_caster_L);
    auto kitchen_kitchen_island_right_lower_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_right_lower_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 2.519200, 0.245000)));
    kitchen_kitchen_island_right_lower_drawer_main->setCollisionShape(shape_Drawer_60_29);
    auto pr2_l_wrist_flex_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_wrist_flex_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.771000, 0.188000, 0.790675)));
    pr2_l_wrist_flex_link->setCollisionShape(shape_wrist_flex);
    auto kitchen_oven_area_area_middle_upper_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_middle_upper_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.182500, 1.900000, 0.817500)));
    kitchen_oven_area_area_middle_upper_drawer_handle->setCollisionShape(shape_Handle60);
    auto pr2_l_upper_arm_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_upper_arm_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, 0.188000, 0.790675)));
    pr2_l_upper_arm_roll_link->setCollisionShape(shape_upper_arm_roll_L);
    auto pr2_l_shoulder_lift_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_shoulder_lift_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, 0.188000, 0.790675)));
    pr2_l_shoulder_lift_link->setCollisionShape(shape_shoulder_lift);
    auto kitchen_kitchen_island_left_upper_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_left_upper_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 0.919200, 0.535000)));
    kitchen_kitchen_island_left_upper_drawer_main->setCollisionShape(shape_Drawer_60_29);
    auto pr2_r_gripper_r_finger_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_gripper_r_finger_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.847910, -0.198000, 0.790675)));
    pr2_r_gripper_r_finger_link->setCollisionShape(shape_l_finger);
    auto kitchen_oven_area_area_right_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_right_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.162500, 1.450000, 0.815000)));
    kitchen_oven_area_area_right_drawer_handle->setCollisionShape(shape_VHandle130);
    auto pr2_head_tilt_link = std::make_shared<KineverseCollisionObject>();
    pr2_head_tilt_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.000930, 0.000000, 1.172125)));
    pr2_head_tilt_link->setCollisionShape(shape_head_tilt_L);
    auto pr2_r_shoulder_pan_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_shoulder_pan_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.050000, -0.188000, 0.790675)));
    pr2_r_shoulder_pan_link->setCollisionShape(shape_shoulder_pan);
    auto pr2_r_shoulder_lift_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_shoulder_lift_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, -0.188000, 0.790675)));
    pr2_r_shoulder_lift_link->setCollisionShape(shape_shoulder_lift);
    auto kitchen_oven_area_area_left_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_left_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.162500, 2.350000, 0.815000)));
    kitchen_oven_area_area_left_drawer_handle->setCollisionShape(shape_VHandle130);
    auto kitchen_fridge_area_lower_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_fridge_area_lower_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.535000, -1.060000, 0.322500)));
    kitchen_fridge_area_lower_drawer_main->setCollisionShape(shape_Drawer_60_29);
    auto pr2_head_pan_link = std::make_shared<KineverseCollisionObject>();
    pr2_head_pan_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.067070, 0.000000, 1.172125)));
    pr2_head_pan_link->setCollisionShape(shape_head_pan_L);
    auto kitchen_kitchen_island_middle_upper_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_middle_upper_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 1.719200, 0.645000)));
    kitchen_kitchen_island_middle_upper_drawer_handle->setCollisionShape(shape_Handle100);
    auto pr2_l_gripper_r_finger_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_gripper_r_finger_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.883510, 0.178000, 0.790675)));
    pr2_l_gripper_r_finger_link->setCollisionShape(shape_l_finger);
    auto kitchen_oven_area_oven_door_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_door_handle->setWorldTransform(btTransform(btQuaternion(-0.000000, 1.000000, -0.000000, 0.000000), btVector3(1.182540, 1.900000, 1.306630)));
    kitchen_oven_area_oven_door_handle->setCollisionShape(shape_Handle60);
    auto pr2_br_caster_rotation_link = std::make_shared<KineverseCollisionObject>();
    pr2_br_caster_rotation_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.224600, -0.224600, 0.079200)));
    pr2_br_caster_rotation_link->setCollisionShape(shape_caster_L);
    auto kitchen_oven_area_area_middle_lower_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_middle_lower_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.515000, 1.900000, 0.440000)));
    kitchen_oven_area_area_middle_lower_drawer_main->setCollisionShape(shape_Drawer_60_58);
    auto pr2_r_elbow_flex_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_elbow_flex_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, -0.188000, 0.790675)));
    pr2_r_elbow_flex_link->setCollisionShape(shape_elbow_flex);
    auto kitchen_oven_area_oven_door = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_door->setWorldTransform(btTransform(btQuaternion(0.000000, -0.707107, 0.707107, -0.000000), btVector3(1.241300, 1.900000, 0.908600)));
    kitchen_oven_area_oven_door->setCollisionShape(shape_OvenDoor);
    auto pr2_l_upper_arm_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_upper_arm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, 0.188000, 0.790675)));
    pr2_l_upper_arm_link->setCollisionShape(shape_upper_arm);
    auto pr2_l_gripper_palm_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_gripper_palm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.806600, 0.188000, 0.790675)));
    pr2_l_gripper_palm_link->setCollisionShape(shape_gripper_palm);
    auto kitchen_sink_area_left_upper_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_upper_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.192500, 0.870000, 0.752500)));
    kitchen_sink_area_left_upper_drawer_handle->setCollisionShape(shape_Handle80);
    auto kitchen_kitchen_island_left_lower_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_left_lower_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 0.919200, 0.245000)));
    kitchen_kitchen_island_left_lower_drawer_main->setCollisionShape(shape_Drawer_60_29);
    auto kitchen_kitchen_island_left_lower_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_left_lower_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 0.919200, 0.355000)));
    kitchen_kitchen_island_left_lower_drawer_handle->setCollisionShape(shape_Handle60);
    auto kitchen_fridge_area_lower_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_fridge_area_lower_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.202500, -1.060000, 0.432500)));
    kitchen_fridge_area_lower_drawer_handle->setCollisionShape(shape_Handle60);
    auto kitchen_kitchen_island_right_upper_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_right_upper_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 2.519200, 0.645000)));
    kitchen_kitchen_island_right_upper_drawer_handle->setCollisionShape(shape_Handle60);
    auto kitchen_kitchen_island_middle_lower_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_middle_lower_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 1.719200, 0.355000)));
    kitchen_kitchen_island_middle_lower_drawer_handle->setCollisionShape(shape_Handle100);
    auto pr2_l_gripper_r_finger_tip_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_gripper_r_finger_tip_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.974880, 0.173050, 0.790675)));
    pr2_l_gripper_r_finger_tip_link->setCollisionShape(shape_l_finger_tip);
    auto pr2_torso_lift_link = std::make_shared<KineverseCollisionObject>();
    pr2_torso_lift_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.050000, 0.000000, 0.790675)));
    pr2_torso_lift_link->setCollisionShape(shape_torso_lift_L);
    auto kitchen_oven_area_area_middle_lower_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_middle_lower_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.182500, 1.900000, 0.677500)));
    kitchen_oven_area_area_middle_lower_drawer_handle->setCollisionShape(shape_Handle60);
    auto pr2_r_upper_arm_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_upper_arm_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.050000, -0.188000, 0.790675)));
    pr2_r_upper_arm_roll_link->setCollisionShape(shape_upper_arm_roll_L);
    auto pr2_laser_tilt_mount_link = std::make_shared<KineverseCollisionObject>();
    pr2_laser_tilt_mount_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.048930, 0.000000, 1.017675)));
    pr2_laser_tilt_mount_link->setCollisionShape(shape_tilting_hokuyo_L);
    auto pr2_bl_caster_rotation_link = std::make_shared<KineverseCollisionObject>();
    pr2_bl_caster_rotation_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.224600, 0.224600, 0.079200)));
    pr2_bl_caster_rotation_link->setCollisionShape(shape_caster_L);
    auto kitchen_sink_area_trash_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_trash_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.192500, -0.430000, 0.612500)));
    kitchen_sink_area_trash_drawer_handle->setCollisionShape(shape_Handle60);
    auto pr2_r_gripper_r_finger_tip_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_gripper_r_finger_tip_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.939280, -0.202950, 0.790675)));
    pr2_r_gripper_r_finger_tip_link->setCollisionShape(shape_l_finger_tip);
    auto kitchen_sink_area_left_bottom_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_bottom_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.192500, 0.870000, 0.340000)));
    kitchen_sink_area_left_bottom_drawer_handle->setCollisionShape(shape_Handle80);
    auto kitchen_kitchen_island_right_lower_drawer_handle = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_right_lower_drawer_handle->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-0.732500, 2.519200, 0.355000)));
    kitchen_kitchen_island_right_lower_drawer_handle->setCollisionShape(shape_Handle60);
    auto kitchen_sink_area_dish_washer_door = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_dish_washer_door->setWorldTransform(btTransform(btQuaternion(0.000000, -0.707107, 0.707107, -0.000000), btVector3(1.255000, 0.170000, 0.120000)));
    kitchen_sink_area_dish_washer_door->setCollisionShape(shape_DishWasherDoor);
    auto kitchen_iai_fridge_door = std::make_shared<KineverseCollisionObject>();
    kitchen_iai_fridge_door->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.245000, -1.360000, 0.985000)));
    kitchen_iai_fridge_door->setCollisionShape(shape_FridgeDoor);
    auto pr2_r_forearm_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_forearm_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, -0.188000, 0.790675)));
    pr2_r_forearm_roll_link->setCollisionShape(shape_forearm_roll_L);
    auto pr2_r_wrist_flex_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_wrist_flex_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.771000, -0.188000, 0.790675)));
    pr2_r_wrist_flex_link->setCollisionShape(shape_wrist_flex);
    auto pr2_l_gripper_l_finger_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_gripper_l_finger_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.883510, 0.198000, 0.790675)));
    pr2_l_gripper_l_finger_link->setCollisionShape(shape_l_finger);
    auto pr2_l_forearm_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_forearm_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, 0.188000, 0.790675)));
    pr2_l_forearm_roll_link->setCollisionShape(shape_forearm_roll_L);
    auto pr2_l_gripper_l_finger_tip_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_gripper_l_finger_tip_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.974880, 0.202950, 0.790675)));
    pr2_l_gripper_l_finger_tip_link->setCollisionShape(shape_l_finger_tip);
    auto pr2_r_gripper_l_finger_tip_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_gripper_l_finger_tip_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.939280, -0.173050, 0.790675)));
    pr2_r_gripper_l_finger_tip_link->setCollisionShape(shape_l_finger_tip);
    auto kitchen_oven_area_area_left_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_left_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.515000, 2.350000, 0.815000)));
    kitchen_oven_area_area_left_drawer_main->setCollisionShape(shape_VDrawer);
    auto kitchen_sink_area_trash_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_trash_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.525000, -0.430000, 0.375000)));
    kitchen_sink_area_trash_drawer_main->setCollisionShape(shape_Drawer_60_58);
    auto kitchen_sink_area_left_middle_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_middle_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.525000, 0.870000, 0.520000)));
    kitchen_sink_area_left_middle_drawer_main->setCollisionShape(shape_Drawer_80_29);
    auto pr2_l_wrist_roll_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_wrist_roll_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.771000, 0.188000, 0.790675)));
    pr2_l_wrist_roll_link->setCollisionShape(shape_wrist_roll_L);
    auto kitchen_oven_area_oven_knob_stove_3 = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_knob_stove_3->setWorldTransform(btTransform(btQuaternion(0.707107, 0.000000, 0.707107, -0.000000), btVector3(1.245000, 1.970000, 1.414900)));
    kitchen_oven_area_oven_knob_stove_3->setCollisionShape(shape_Knob);
    auto pr2_fr_caster_rotation_link = std::make_shared<KineverseCollisionObject>();
    pr2_fr_caster_rotation_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.224600, -0.224600, 0.079200)));
    pr2_fr_caster_rotation_link->setCollisionShape(shape_caster_L);
    auto kitchen_oven_area_oven_knob_stove_1 = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_knob_stove_1->setWorldTransform(btTransform(btQuaternion(0.707107, 0.000000, 0.707107, -0.000000), btVector3(1.245000, 2.110000, 1.414900)));
    kitchen_oven_area_oven_knob_stove_1->setCollisionShape(shape_Knob);
    auto kitchen_kitchen_island_right_upper_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_kitchen_island_right_upper_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(-1.065000, 2.519200, 0.535000)));
    kitchen_kitchen_island_right_upper_drawer_main->setCollisionShape(shape_Drawer_60_29);
    auto pr2_l_forearm_link = std::make_shared<KineverseCollisionObject>();
    pr2_l_forearm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.450000, 0.188000, 0.790675)));
    pr2_l_forearm_link->setCollisionShape(shape_forearm);
    auto kitchen_oven_area_oven_knob_stove_4 = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_oven_knob_stove_4->setWorldTransform(btTransform(btQuaternion(0.707107, 0.000000, 0.707107, -0.000000), btVector3(1.245000, 1.900000, 1.414900)));
    kitchen_oven_area_oven_knob_stove_4->setCollisionShape(shape_Knob);
    auto pr2_r_gripper_palm_link = std::make_shared<KineverseCollisionObject>();
    pr2_r_gripper_palm_link->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 0.000000, 1.000000), btVector3(0.771000, -0.188000, 0.790675)));
    pr2_r_gripper_palm_link->setCollisionShape(shape_gripper_palm);
    auto kitchen_oven_area_area_middle_upper_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_oven_area_area_middle_upper_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.515000, 1.900000, 0.800000)));
    kitchen_oven_area_area_middle_upper_drawer_main->setCollisionShape(shape_Drawer_60_14);
    auto kitchen_sink_area_left_upper_drawer_main = std::make_shared<KineverseCollisionObject>();
    kitchen_sink_area_left_upper_drawer_main->setWorldTransform(btTransform(btQuaternion(0.000000, 0.000000, 1.000000, -0.000000), btVector3(1.525000, 0.870000, 0.735000)));
    kitchen_sink_area_left_upper_drawer_main->setCollisionShape(shape_Drawer_80_14);

    world.addCollisionObject(kitchen_iai_fridge_door_handle);
    world.addCollisionObject(kitchen_kitchen_island_middle_lower_drawer_main);
    world.addCollisionObject(pr2_r_wrist_roll_link);
    world.addCollisionObject(kitchen_kitchen_island_left_upper_drawer_handle);
    world.addCollisionObject(pr2_r_forearm_link);
    world.addCollisionObject(kitchen_oven_area_oven_knob_oven);
    world.addCollisionObject(kitchen_oven_area_area_right_drawer_main);
    world.addCollisionObject(pr2_l_elbow_flex_link);
    world.addCollisionObject(kitchen_oven_area_oven_knob_stove_2);
    world.addCollisionObject(pr2_r_gripper_l_finger_link);
    world.addCollisionObject(kitchen_sink_area_left_middle_drawer_handle);
    world.addCollisionObject(pr2_r_upper_arm_link);
    world.addCollisionObject(kitchen_kitchen_island_middle_upper_drawer_main);
    world.addCollisionObject(kitchen_sink_area_left_bottom_drawer_main);
    world.addCollisionObject(pr2_l_shoulder_pan_link);
    world.addCollisionObject(kitchen_sink_area_dish_washer_door_handle);
    world.addCollisionObject(pr2_fl_caster_rotation_link);
    world.addCollisionObject(kitchen_kitchen_island_right_lower_drawer_main);
    world.addCollisionObject(pr2_l_wrist_flex_link);
    world.addCollisionObject(kitchen_oven_area_area_middle_upper_drawer_handle);
    world.addCollisionObject(pr2_l_upper_arm_roll_link);
    world.addCollisionObject(pr2_l_shoulder_lift_link);
    world.addCollisionObject(kitchen_kitchen_island_left_upper_drawer_main);
    world.addCollisionObject(pr2_r_gripper_r_finger_link);
    world.addCollisionObject(kitchen_oven_area_area_right_drawer_handle);
    world.addCollisionObject(pr2_head_tilt_link);
    world.addCollisionObject(pr2_r_shoulder_pan_link);
    world.addCollisionObject(pr2_r_shoulder_lift_link);
    world.addCollisionObject(kitchen_oven_area_area_left_drawer_handle);
    world.addCollisionObject(kitchen_fridge_area_lower_drawer_main);
    world.addCollisionObject(pr2_head_pan_link);
    world.addCollisionObject(kitchen_kitchen_island_middle_upper_drawer_handle);
    world.addCollisionObject(pr2_l_gripper_r_finger_link);
    world.addCollisionObject(kitchen_oven_area_oven_door_handle);
    world.addCollisionObject(pr2_br_caster_rotation_link);
    world.addCollisionObject(kitchen_oven_area_area_middle_lower_drawer_main);
    world.addCollisionObject(pr2_r_elbow_flex_link);
    world.addCollisionObject(kitchen_oven_area_oven_door);
    world.addCollisionObject(pr2_l_upper_arm_link);
    world.addCollisionObject(pr2_l_gripper_palm_link);
    world.addCollisionObject(kitchen_sink_area_left_upper_drawer_handle);
    world.addCollisionObject(kitchen_kitchen_island_left_lower_drawer_main);
    world.addCollisionObject(kitchen_kitchen_island_left_lower_drawer_handle);
    world.addCollisionObject(kitchen_fridge_area_lower_drawer_handle);
    world.addCollisionObject(kitchen_kitchen_island_right_upper_drawer_handle);
    world.addCollisionObject(kitchen_kitchen_island_middle_lower_drawer_handle);
    world.addCollisionObject(pr2_l_gripper_r_finger_tip_link);
    world.addCollisionObject(pr2_torso_lift_link);
    world.addCollisionObject(kitchen_oven_area_area_middle_lower_drawer_handle);
    world.addCollisionObject(pr2_r_upper_arm_roll_link);
    world.addCollisionObject(pr2_laser_tilt_mount_link);
    world.addCollisionObject(pr2_bl_caster_rotation_link);
    world.addCollisionObject(kitchen_sink_area_trash_drawer_handle);
    world.addCollisionObject(pr2_r_gripper_r_finger_tip_link);
    world.addCollisionObject(kitchen_sink_area_left_bottom_drawer_handle);
    world.addCollisionObject(kitchen_kitchen_island_right_lower_drawer_handle);
    world.addCollisionObject(kitchen_sink_area_dish_washer_door);
    world.addCollisionObject(kitchen_iai_fridge_door);
    world.addCollisionObject(pr2_r_forearm_roll_link);
    world.addCollisionObject(pr2_r_wrist_flex_link);
    world.addCollisionObject(pr2_l_gripper_l_finger_link);
    world.addCollisionObject(pr2_l_forearm_roll_link);
    world.addCollisionObject(pr2_l_gripper_l_finger_tip_link);
    world.addCollisionObject(pr2_r_gripper_l_finger_tip_link);
    world.addCollisionObject(kitchen_oven_area_area_left_drawer_main);
    world.addCollisionObject(kitchen_sink_area_trash_drawer_main);
    world.addCollisionObject(kitchen_sink_area_left_middle_drawer_main);
    world.addCollisionObject(pr2_l_wrist_roll_link);
    world.addCollisionObject(kitchen_oven_area_oven_knob_stove_3);
    world.addCollisionObject(pr2_fr_caster_rotation_link);
    world.addCollisionObject(kitchen_oven_area_oven_knob_stove_1);
    world.addCollisionObject(kitchen_kitchen_island_right_upper_drawer_main);
    world.addCollisionObject(pr2_l_forearm_link);
    world.addCollisionObject(kitchen_oven_area_oven_knob_stove_4);
    world.addCollisionObject(pr2_r_gripper_palm_link);
    world.addCollisionObject(kitchen_oven_area_area_middle_upper_drawer_main);
    world.addCollisionObject(kitchen_sink_area_left_upper_drawer_main);

    world.updateAabbs();

    std::unordered_map<CollisionObjectPtr, btScalar> qbatch =  {
        {pr2_l_upper_arm_roll_link, 4.0f},
        {pr2_r_gripper_r_finger_tip_link, 4.0f},
        {pr2_r_forearm_link, 4.0f},
        {pr2_l_upper_arm_link, 4.0f},
        {pr2_r_gripper_r_finger_link, 4.0f},
        {pr2_head_tilt_link, 4.0f},
        {pr2_l_gripper_palm_link, 4.0f},
        {pr2_l_gripper_r_finger_tip_link, 4.0f},
        {pr2_r_shoulder_pan_link, 4.0f},
        {pr2_r_forearm_roll_link, 4.0f},
        {pr2_r_elbow_flex_link, 4.0f},
        {pr2_r_shoulder_lift_link, 4.0f},
        {pr2_l_forearm_roll_link, 4.0f},
        {pr2_r_gripper_l_finger_tip_link, 4.0f},
        {pr2_l_elbow_flex_link, 4.0f},
        {pr2_l_shoulder_lift_link, 4.0f},
        {pr2_l_gripper_l_finger_tip_link, 4.0f},
        {pr2_r_wrist_roll_link, 4.0f},
        {pr2_head_pan_link, 4.0f},
        {pr2_l_gripper_l_finger_link, 4.0f},
        {pr2_r_gripper_l_finger_link, 4.0f},
        {pr2_r_wrist_flex_link, 4.0f},
        {pr2_l_gripper_r_finger_link, 4.0f},
        {pr2_r_upper_arm_link, 4.0f},
        {pr2_torso_lift_link, 4.0f},
        {pr2_l_wrist_roll_link, 4.0f},
        {pr2_l_shoulder_pan_link, 4.0f},
        {pr2_br_caster_rotation_link, 4.0f},
        {pr2_r_upper_arm_roll_link, 4.0f},
        {pr2_l_forearm_link, 4.0f},
        {pr2_r_gripper_palm_link, 4.0f},
        {pr2_fl_caster_rotation_link, 4.0f},
        {pr2_fr_caster_rotation_link, 4.0f},
        {pr2_bl_caster_rotation_link, 4.0f},
        {pr2_l_wrist_flex_link, 4.0f},
        {pr2_laser_tilt_mount_link, 4.0f}
    };

    for (int i = 0; i < 1; i++) {
        auto closest_pairs = world.get_closest_batch(qbatch);
        // for (const auto& pair: closest_pairs)
        //     for (const auto& cp: pair.second)
        //         std::cout << cp << std::endl;
    }

    std::cout << "Done with distance checks." << std::endl;

    return 0;
}
