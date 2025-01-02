use std::path::Path;

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};
use rapier3d::pipeline::PhysicsPipeline;

mod kinematics;
mod dynamics;
use kinematics::kinematics::EEKinematicModel;
use dynamics::dynamics::MultibodyLinkVec;

fn main() {    
    let mut rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
    
    println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
}
