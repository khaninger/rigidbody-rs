use std::path::Path;

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};

mod kinematics;
use kinematics::kinematics::EEKinematicModel;

fn main() {    
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut colliders = ColliderSet::new();

    let mut rob = EEKinematicModel::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
    
    let ee_pose_zero = rob.fwd_kin(&[0.0; 7]);
    println!("ee_pose_zero: {:?}", ee_pose_zero);

    let ee_pose_ones = rob.fwd_kin(&[1.0; 7]);
    println!("ee_pose_ones: {:?}", ee_pose_ones);
    
    let jt_angles = rob.inv_kin(Isometry::translation(0.5,0.0,0.5));
    println!("jt angles:   {:?}", jt_angles);

    let ee_pose = rob.fwd_kin(jt_angles.as_slice());
    println!("ee_pose:     {:?}", ee_pose);
}
