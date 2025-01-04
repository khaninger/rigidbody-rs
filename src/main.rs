use std::path::Path;

//use rapier3d::prelude::*;
//use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};
//use rapier3d::pipeline::PhysicsPipeline;

use nalgebra::{Vector3, Point3, Isometry3};

mod kinematics;
mod dynamics;
mod spatial;
use kinematics::kinematics::EEKinematicModel;
use dynamics::dynamics::MultibodyLinkVec;
use spatial::*;


fn main() {
    let v = Vector3::new(0., 1., 2.);

    let p = Point3::new(2, 3, 4);
    let v2 = Vector3::new(1., 2., 3.);
    let r = v.cross(&v2);
    
    let T: Isometry3<f32> = Isometry3::identity();

    let sv = SpatialVelocity { lin: v, rot: v2 };
    println!("{:?}", sv);
    println!("{:?}", T*sv);

    
    //println!("{:?}", T.transform_vector(&v));
    //println!("{:?}", T.rotation*v);
    //println!("{:?}", v.shape());


    //let mut rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
    //println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
}
