#![allow(warnings)] // Suppress warning message on compile
use std::path::Path;
use std::iter::zip;

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

pub fn rnea() {
    let jts = parse_urdf(&Path::new("assets/fr3.urdf"));
    let q = &[0.1; 7];
    let dq = &[0.2; 7];
    let ddq = &[0.1; 7];

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

        v = tr_joint*&v + vel_joint;
        let body_jac = BodyJacobian::revolute_z(); //TODO: make a borrow multiply so don't double init?
        a = tr_joint*&a + body_jac*ddq[i] + cJ; // + v.cross(vel_joint);
        let I = jt.child_mass.reconstruct_inertia_matrix();
        let fi = SpatialForce {
            lin: a.lin*jt.child_mass.mass(),
            rot: I*a.rot
        };// + v.gcross_matrix()*I*v-Xj*f;

        println!("jt {:?} mass: {:?}", i, jt.child_mass.mass());
        f.push(fi); 
    }
    for (i, link) in jts.iter().rev().enumerate() {
        tau.push(BodyJacobian::revolute_z()*&f[i]);
        //f = f[i] + Xj.transpose()*f
    }
    
    println!("vel: {:?}", v);
    println!("acc: {:?}", a);
    println!("tau: {:?}", tau);
}

/* Featherstone's transformation notation
^BX_A denotes a transofrmation from A to B for a motion vector.
^BX_A^* is the same transformation for forces, = ^BX_A^{-T}
*/

fn main() {
    //joint();
    //urdf();
    //rnea();

}

