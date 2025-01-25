#![allow(warnings)] // Suppress warning message on compile
use std::path::Path;
use std::iter::zip;
use std::time::{Duration, Instant};

mod spatial;
mod joint;
mod utils;

use spatial::*;
use joint::*;

use nalgebra::Vector3;

fn main() {
    let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));
    let q = &[0.; 7];
    let dq = &[0.; 7];
    let ddq = &[0.; 7];

    let t = &[0., 1., 2., 3., 4., 5.];
    let mut cnt:f32 = 0.;
    let f: Vec<SpatialForce> = t.iter().map(
        |ti| { cnt += ti;
               SpatialForce {lin: Vector3::new(cnt, 0., 0.), ..Default::default() } }
    ).collect();
    println!("{:?}", f);
    let start = Instant::now();
    let tau = mb.rnea(q, dq, ddq);
    let elapsed = start.elapsed();
    println!("rnea:    took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 1_000., tau);
    let start = Instant::now();
    let tau = mb.rnea_zip(q, dq, ddq);
    let elapsed = start.elapsed();
    println!("rneazip: took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 1_000., tau);

    let start = Instant::now();
    let tr = mb.fwd_kin(&[-1.;7]);
    let tr = mb.fwd_kin(&[ 0.;7]);
    let tr = mb.fwd_kin(&[ 1.;7]);
    let elapsed = start.elapsed();
    println!("fwd_kin: took us {} to find tau = {:?}", elapsed.as_nanos() as f64 / 3_000., tr);

}

