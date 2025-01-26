#![allow(warnings)] // Suppress warning message on compile
#![feature(test)]

use std::path::Path;
use std::iter::zip;
use std::time::{Duration, Instant};

mod spatial;
mod joint;
mod rigidbody;
mod multibody;


use spatial::*;
use joint::*;
use multibody::*;

use nalgebra::{Vector3, Isometry3};

pub type Real = f32;
pub type Transform = Isometry3<Real>;

fn main() {
    let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));
    let q = &[0.; 7];
    let dq = &[0.; 7];
    let ddq = &[0.; 7];

    let start = Instant::now();
    let tau = mb.rnea(q, dq, ddq);
    let elapsed = start.elapsed();
    println!("rnea:    took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 1_000., tau);

    let start = Instant::now();
    let tr = mb.fwd_kin(&[-1.;7]);
    let tr = mb.fwd_kin(&[ 0.;7]);
    let tr = mb.fwd_kin(&[ 1.;7]);
    let elapsed = start.elapsed();
    println!("fwd_kin: took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 3_000., tr);

}

