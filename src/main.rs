#![allow(warnings)]
use std::path::Path;
use std::iter::zip;

//use rapier3d::prelude::*;
//use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};
//use rapier3d::pipeline::PhysicsPipeline;

use nalgebra::{Isometry3, IsometryMatrix3, Point3, Translation3, Unit, UnitQuaternion, Vector3};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};

mod spatial;
mod joint;
mod kinematics;

use spatial::*;
use joint::*;
/*

mod dynamics;
use kinematics::kinematics::EEKinematicModel;
use dynamics::dynamics::MultibodyLinkVec;

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
*/
fn relative_trans() {
    let local = Transform {
        rotation: UnitQuaternion::identity(),
        //rotation: UnitQuaternion::from_scaled_axis(Vector3::new(std::f32::consts::PI, 0., 0.)),
        translation: Translation3::new(0.,0.,1.0)
    };

    let pt = RelativeTransform { coord_frame: &Coord::WORLD, pose: local };
    
    println!("pt {:?}", pt);
//    println!("pt in world {:?}", pt.to_world());
}

fn joint() {
    let jt1 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: RelativeTransform{
            coord_frame: &Coord::WORLD,
            pose: Transform{
                rotation: UnitQuaternion::identity(),
                translation: Translation3::new(1.,0.,0.)
            }
        },
        child: Isometry3::identity()            
    };
    let jt2 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: RelativeTransform{
            coord_frame: &Coord::FIXED(jt1.child),
            pose: Transform{
                rotation: UnitQuaternion::identity(),
                translation: Translation3::new(1.,0.,0.)
            }
        },
        child: Isometry3::identity()
    };
    
    let child_pt = RelativeTransform {
        coord_frame: &Coord::FIXED(jt2.child),
        pose: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        }
    };

    let q1 = std::f32::consts::PI;
    let q2 = std::f32::consts::FRAC_PI_2;
    let child_pt_in_jt1 = jt2.child_to_parent(q2, &child_pt);
    let child_pt_in_world = jt1.child_to_parent(q1, &child_pt_in_jt1);

    println!("pt in jt2 {:?}", child_pt.pose);
    println!("pt in jt1 {:?}", child_pt_in_jt1.pose);
    println!("pt in world {:?}", child_pt_in_world.pose);
}

fn main() {
    let robot = parse_urdf_from_file(Path::new("assets/fr3.urdf")).unwrap();

    let mut jts = Vec::<RevoluteJoint>::new();
    let mut prev_frame:Coord = Coord::WORLD;
    
    for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
        if !link.name.contains("sc") && !link.name.contains("0") {
            let rev_joint = RevoluteJoint::from_xurdf_joint(joint, &prev_frame);
            //prev_frame = Coord::FIXED(rev_joint.child);
 
            jts.push(rev_joint);
        }
    }
    println!("Xurdf Joints: {:?}, Xurdf Links: {:?}, Proc Joints: {:?}", robot.joints.len(), robot.links.len(), jts.len());

    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(1., &ee);
    }

    println!("EE Pose in world, ones {:?}", ee.pose);
        
    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(0., &ee);
    }

    println!("EE Pose in world, zeros {:?}", ee.pose);
    

    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(-1., &ee);
    }

    println!("EE Pose in world, -ones {:?}", ee.pose);

}
