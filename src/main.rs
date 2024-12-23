use std::path::Path;

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};

mod kinematics;
use kinematics::kinematics::{fwd_kin, inv_kin};

fn main() {    
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut colliders = ColliderSet::new();

    let urdf_path = Path::new("assets/fr3.urdf");
    let urdf_options = UrdfLoaderOptions {
        create_colliders_from_collision_shapes: false,
        create_colliders_from_visual_shapes: false,
        apply_imported_mass_props: true,
        make_roots_fixed: true,
        ..Default::default()
    };
    let (mut robot, _) = UrdfRobot::from_file(
        urdf_path,
        urdf_options,
        None
    ).unwrap();

    println!("robot: {:#?}, njoints {:?}, nlinks {:?}", robot.links, robot.joints.len(), robot.links.len());

    for jt in robot.joints.iter() {
        println!("jt locked axes {:?}", jt.joint.locked_axes);
    }

    for lk in robot.links.iter() {
        println!("link fixed {:?}", lk.body.is_fixed());
    }

    let handles = robot.insert_using_multibody_joints(&mut bodies, &mut colliders, &mut multibody_joints, UrdfMultibodyOptions::DISABLE_SELF_CONTACTS);

    let multibody_handle = handles.joints.iter().last().unwrap().joint.unwrap();
    
    println!("multibody_handle {:?}", multibody_handle);
    println!("multibody {:#?}", multibody_joints.get_mut(multibody_handle));

    let jt_angles = inv_kin(&mut bodies,
                          &mut multibody_joints,
                          multibody_handle,
                          Isometry::identity()
    );
    println!("jt angles: {:?}", jt_angles);

    let ee_pose = fwd_kin(&mut bodies,
                          &mut multibody_joints,
                          multibody_handle,
                          handles.links.iter().last().unwrap().body,
                          jt_angles.as_slice());
    println!("ee_pose:   {:?}", ee_pose);

}
