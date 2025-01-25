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

    // Joint torques
    let mut tau = Vec::<Real>::new();

    // Vel, acc, force of current link, in local link coordinates
    let mut v = SpatialVelocity::new();
    let mut a = SpatialVelocity {
        lin: Vector3::<Real>::new(0., 0., -9.81),
        rot: Vector3::<Real>::zeros()
    };
    let mut f = Vec::<SpatialForce>::new();

    // Transform spatial vectors in link coordinates to world
    let mut link_to_world = Isometry3::identity();

    for (i, jt) in jts.iter().enumerate() {
        // Transform spatial vectors from parent to child link
        let parent_to_link = jt.joint_transform(q[i]).inverse();   // X_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4

        // Velocity of the joint, expressed in child link coordinates
        let vel_joint = body_jac*dq[i];  // vJ, (3.33)

        
        link_to_world = parent_to_link*link_to_world;

        v = parent_to_link*&v + vel_joint;
        a = parent_to_link*&a + body_jac*ddq[i]; // + cJ // + v.cross(vel_joint);
        let I = jt.child_mass.reconstruct_inertia_matrix();
        let c = jt.child_mass.local_com.coords;
        let fi = SpatialForce {
            lin: a.lin*jt.child_mass.mass() - c.cross(&a.rot),
            rot: I*a.rot + c.cross(&a.lin)*jt.child_mass.mass()
        };// + v.gcross_matrix()*I*v-Xj*f;

        println!("jt {:?}\n   vel: {:?}\n   acc: {:?}\n force: {:?}", i, v, a, fi);
        println!("mass: {:?} com: {:?}", jt.child_mass.mass(), jt.child_mass.local_com);
        f.push(fi); 
    }
    println!("world to ee \n q: {:?}, tr: {:?}", q[0], link_to_world);

    for (i, jt) in jts.iter().enumerate().rev() {
        tau.push(BodyJacobian::revolute_z()*&f[i]);
        if i > 0 {
            let f_tr = f[i].inv_transform(&jt.joint_transform(q[i]));
            // are we transforming the forces right?
            println!("jt {:?}\n   orig f: {:?}\n   tran f: {:?}", i, f[i], f_tr); 

            //println!("jt {:?} transformed f: {:?}, original f: {:?}", i, f_tr, f[i-1]);
            f[i-1] += f_tr;
        }
    };
    
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

