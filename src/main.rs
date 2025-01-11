#![allow(warnings)]
use std::path::Path;

//use rapier3d::prelude::*;
//use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};
//use rapier3d::pipeline::PhysicsPipeline;

use nalgebra::{Isometry3, IsometryMatrix3, Point3, Translation3, Unit, UnitQuaternion, Vector3};

mod kinematics;
mod dynamics;
mod spatial;
mod joint;

use kinematics::kinematics::EEKinematicModel;
use dynamics::dynamics::MultibodyLinkVec;
use spatial::*;
use joint::*;


fn cross() {
    let v = Vector3::new(0., 1., 2.);
    let p = Point3::new(2, 3, 4);
    let v2 = Vector3::new(1., 2., 3.);
    let r = v.cross(&v2);
    
    let T: Isometry3<f32> = Isometry3::identity();

    
    
    //let sv = SpatialVelocity { lin: v, rot: v2 };

    //println!("{:?}", T.translation.vector.cross(&v));
    //println!("{:?}", sv);
    //println!("{:?}", T*sv);

    
    //println!("{:?}", T.transform_vector(&v));
    //println!("{:?}", T.rotation*v);
    //println!("{:?}", v.shape());

}

fn rnea() {
    let mut rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
    //println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
    let res = rob.rnea(&[0.1; 9], &[0.2; 9], &[0.3; 9]);
    println!("rnea: {:?}", res);
}


fn main() {
    let world_frame = Transform::identity();
    let local = Transform {
        rotation: UnitQuaternion::identity(),
        //rotation: UnitQuaternion::from_scaled_axis(Vector3::new(std::f32::consts::PI, 0., 0.)),
        translation: Translation3::new(0.,0.,1.0)
    };

    let pt = Pose { coord_frame: &world_frame, pose: local };
    
    println!("pt {:?}", pt);
    println!("pt in world {:?}", pt.to_world());
    
    let jt = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent_anchor: pt,
        child_frame: Isometry3::identity()            
    };

    let child_pt = Pose {
        coord_frame: &jt.child_frame,
        pose: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(0.,1.,0.)
        }
    };

    println!("jt trans {:?}", jt.child_to_parent(std::f32::consts::PI, child_pt));
}
