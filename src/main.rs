#![allow(warnings)] // Suppress warning message on compile
use std::path::Path;
use std::iter::zip;

use nalgebra::{Isometry3, IsometryMatrix3, Matrix3, Point3, OPoint, Translation3, Unit, UnitQuaternion, Vector3, U3};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;

mod spatial;
mod joint;
mod dynamics;

use spatial::*;
use joint::*;
use dynamics::*;

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
        child: Isometry3::identity(),
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        ),            
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
        child: Isometry3::identity(),
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        )
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

    println!("pt in jt2   {:?}", child_pt.pose);
    println!("pt in jt1   {:?}", child_pt_in_jt1.pose);
    println!("pt in world {:?}", child_pt_in_world.pose);

}

fn urdf() {
    let robot = parse_urdf_from_file(Path::new("assets/fr3.urdf")).unwrap();

    let mut jts = Vec::<RevoluteJoint>::new();
    let mut prev_frame:Coord = Coord::WORLD;
    
    for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
        if !link.name.contains("sc") && !link.name.contains("0") {
            let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link, &prev_frame);
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

pub fn rnea() {

    let robot = parse_urdf_from_file(Path::new("assets/fr3.urdf")).unwrap();

    let mut jts = Vec::<RevoluteJoint>::new();
    let mut prev_frame:Coord = Coord::WORLD;
    
    for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
        if !link.name.contains("sc") && !link.name.contains("0") {
            let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link, &prev_frame);
            jts.push(rev_joint);
        }
    }

    let q = &[0.1; 9];
    let dq = &[0.2; 9];
    let ddq = &[0.0; 9];

    let mut tau = Vec::<Real>::new();
    
    let mut v = SpatialVelocity::new();
    let mut a = SpatialVelocity {
        lin: Vector3::<Real>::new(0., 0., -9.81),
        rot: Vector3::<Real>::zeros()
    };
    let mut f = Vec::<SpatialForce>::new();

    let mut tr_world_to_child = Isometry3::identity();

    for (i, jt) in jts.iter().enumerate() {
        // Local joint transformation and velocity
        let tr_joint = jt.joint_transform(q[i]);   // X_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4
        let body_jac = BodyJacobian::revolute_z(); // S, assume all jts are revolute about z
        let vel_joint = body_jac*dq[i];  // vJ, (3.33)
        let cJ = SpatialVelocity::new(); // (3.42 & 3.43, no rate of change 

        tr_world_to_child = tr_joint*tr_world_to_child;

        v = tr_joint*v + vel_joint;
        let body_jac = BodyJacobian::revolute_z(); //TODO: make a borrow multiply so don't double init?
        a = tr_joint*a + body_jac*ddq[i] + cJ; // + v.cross(vel_joint);
        let I = jt.child_mass.reconstruct_inertia_matrix();
        let fi = SpatialForce {
            lin: a.lin*jt.child_mass.mass(),
            rot: I*a.rot
        };// + v.gcross_matrix()*I*v-Xj*f;

        println!("jt mass: {:?}", jt.child_mass.mass());
        f.push(fi); 
    }
    for (i, link) in jts.iter().rev().enumerate() {
        tau.push(BodyJacobian::revolute_z()*f[i].clone());
        //f = f[i] + Xj.transpose()*f
    }
    
    println!("vel: {:?} \nacc: {:?}", v, a);
    println!("tau: {:?}", tau);
}

fn main() {
    //joint();
    //urdf();
    rnea();
}
