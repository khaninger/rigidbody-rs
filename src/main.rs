#![allow(warnings)] // Suppress warning message on compile
use std::path::Path;
use std::iter::zip;
use std::time::{Duration, Instant};

use nalgebra::{Isometry3, IsometryMatrix3, Matrix3, Point3, OPoint, Translation3, Unit, UnitQuaternion, Vector3, U3};

mod spatial;
mod joint;

use spatial::*;
use joint::*;


fn main() {
    let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));
    let q = &[0.; 7];
    let dq = &[0.; 7];
    let ddq = &[0.; 7];
    //fwd_kin(&[1.; 7]);
    //fwd_kin(&[0.; 7]);
    //fwd_kin(&[-1.; 7]);

    let start = Instant::now();
    let tau = mb.rnea(q, dq, ddq);
    let elapsed = start.elapsed();
    println!("RNEA:   took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 1_000., tau);

    let start = Instant::now();
    let tr = mb.fwd_kin(q);
    let tr = mb.fwd_kin(q);
    let tr = mb.fwd_kin(q);
    let elapsed = start.elapsed();
    println!("fwd_kin: took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 3_000., tr);

}

