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

    let result = UrdfRobot::from_file(urdf_path, urdf_options, None);

    let (mut robot, xrobot) = match result {
        Ok(val) => val,
        Err(err) => {
            panic!("Error in URDF loading {}", err);
        }
    };
   

    // for lk in xrobot.links.iter() { println!("xrobot link {:?}", lk.name); }   
    // for jt in robot.joints.iter() { println!("jt locked axes {:?}", jt.joint.locked_axes); }
    // for lk in robot.links.iter() { println!("link fixed {:?}", lk.body.is_fixed()); }
    
    let handles = robot.insert_using_multibody_joints(&mut bodies, &mut colliders, &mut multibody_joints, UrdfMultibodyOptions::DISABLE_SELF_CONTACTS);
    let ee_link_handle = handles.links[16].body;
    let (_, _, ee_joint_handle) = multibody_joints.attached_joints(ee_link_handle).last().unwrap();   
    
    //println!("ee_joint_handle {:?}", ee_joint_handle);
    let (mut multibody, link_id) = multibody_joints.get_mut(ee_joint_handle).unwrap();
    for lk in multibody.links() {
        println!("first link mb : {:?}", lk.is_root());
    }
    multibody.forward_kinematics(&mut bodies, false); // Needed so that the extra DOF in root are removed
    
    let ee_pose_zero = fwd_kin(&mut bodies,
                               &mut multibody,
                               ee_link_handle,
                               &[0.0; 7]);
    println!("ee_pose_zero: {:?}", ee_pose_zero);

    let ee_pose_ones = fwd_kin(&mut bodies,
                               &mut multibody,
                               ee_link_handle,
                               &[1.0; 7]);
    println!("ee_pose_ones: {:?}", ee_pose_ones);
    
    let jt_angles = inv_kin(&mut bodies,
                            &mut multibody,
                            link_id,
                            Isometry::translation(0.5,0.0,0.5)
    );
    println!("jt angles:   {:?}", jt_angles);

    let ee_pose = fwd_kin(&mut bodies,
                          &mut multibody,
                          ee_link_handle,
                          jt_angles.as_slice());
    println!("ee_pose:     {:?}", ee_pose);


}
