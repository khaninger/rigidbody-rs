#![allow(warnings)] // Suppress warning message on compile
use std::path::Path;
use std::iter::zip;
use std::time::{Duration, Instant};

use nalgebra::{Isometry3, IsometryMatrix3, Matrix3, Point3, OPoint, Translation3, Unit, UnitQuaternion, Vector3, U3};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;

mod spatial;
mod joint;


use spatial::*;
use joint::*;

fn parse_urdf(path: &Path) -> [RevoluteJoint; 7] {
    let robot = parse_urdf_from_file(path).unwrap();

    let mut jts = Vec::<RevoluteJoint>::new();
    let mut prev_frame:Coord = Coord::WORLD;
    
    for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
        if !joint.joint_type.contains("fixed") {
            let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link);
            jts.push(rev_joint);
        }
    }
    jts.try_into().unwrap()
}


const body_jac: BodyJacobian = BodyJacobian::revolute_z();

pub fn rnea() {
    let jts = parse_urdf(&Path::new("assets/fr3.urdf"));

    let q = &[0.; 7];
    let dq = &[0.; 7];
    let ddq = &[0.; 7];

    let mut tau = Vec::<Real>::new();
    
    let mut v = SpatialVelocity::new();
    let mut a = SpatialVelocity {
        lin: Vector3::<Real>::new(0., 0., -9.81),
        rot: Vector3::<Real>::zeros()
    };
    let mut f = Vec::<SpatialForce>::new();

    // Applying this should transform vels/forces
    // in current body coordinates into world coordinates
    let mut body_to_world = Isometry3::identity();

    for (i, jt) in jts.iter().enumerate() {
        // Local joint transformation and velocity
        let parent_to_body = jt.joint_transform(q[i]).inverse();   // X_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4
        let vel_joint = body_jac*dq[i];  // vJ, (3.33)
        let cJ = SpatialVelocity::new(); // (3.42 & 3.43, no rate of change 

        body_to_world = parent_to_body*body_to_world;

        v = parent_to_body*&v + vel_joint;
        a = parent_to_body*&a + body_jac*ddq[i] + cJ; // + v.cross(vel_joint);
        let I = jt.child_mass.reconstruct_inertia_matrix();
        let fi = SpatialForce {
            lin: a.lin*jt.child_mass.mass(),
            rot: I*a.rot
        };// + v.gcross_matrix()*I*v-Xj*f;

        //println!("jt {:?} mass: {:?}", i, jt.child_mass.mass());
        println!("jt {:?} vel: {:?}", i, v);
        f.push(fi); 
    }
    println!("world to ee: {:?}", body_to_world);

    for (i, link) in jts.iter().rev().enumerate() {
        tau.push(BodyJacobian::revolute_z()*&f[i]);
        //f = f[i] + Xj.transpose()*f
    }
    
    println!("vel: {:?}", v);
    println!("acc: {:?}", a);
    println!("tau: {:?}", tau);
}

fn fwd_kin (q: &[Real; 7]) {
    let jts = parse_urdf(&Path::new("assets/fr3.urdf"));
    
    let start = Instant::now();
    let mut tr = Transform::identity();
    for (i, jt) in jts.iter().rev().enumerate() {
        tr = jt.joint_transform(q[i])*tr;
    }
    let elapsed = start.elapsed();

    println!("EE Pose in world, in ms {:?}\n q: {:?}, tr: {:?}", elapsed.as_nanos() as f64 / 1_000_000., q[0], tr);
}

fn main() {
    fwd_kin(&[1.; 7]);
    fwd_kin(&[0.; 7]);
    fwd_kin(&[-1.; 7]);
    rnea();
}

