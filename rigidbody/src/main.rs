#![feature(portable_simd)]
#![feature(test)]


use std::path::Path;
use std::time::{Duration, Instant};

use rigidbody::multibody::*;

use nalgebra::{Vector3, Isometry3, UnitQuaternion};

extern crate test;
use simba::simd::AutoF32x8;
use simba::simd::SimdComplexField;

fn vec_simd(q: AutoF32x8, T: &mut Isometry3<AutoF32x8>) {
    T.translation.z = q.simd_cos()
}

fn vec_iter(q: &[f32; 8], T:&mut [Isometry3<f32>; 8]) {
    for (qi, Ti) in q.iter().zip(T.iter_mut()) {
        Ti.translation.z = qi.cos();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;

    #[bench]
    fn bench_vec_simd(b: &mut Bencher) {
        let q = AutoF32x8::ONE;
        let mut T = Isometry3::<AutoF32x8>::identity();
        b.iter(|| {
            vec_simd(q, &mut T);
        });
    }

    #[bench]
    fn bench_vec_iter(b: &mut Bencher) {
        let q = [1.0; 8];
        let mut T = [Isometry3::<f32>::identity(); 8];
        b.iter(|| {
            vec_iter(&q, &mut T);
        });
    }
}

fn kin_test() {

    let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));

    let q = &[0.; 7];
    let dq = &[0.; 7];
    let ddq = &[0.; 7];

    let start = Instant::now();
    let tau = mb.rnea(q, dq, ddq);
    let elapsed = start.elapsed();
    println!("rnea:    took {:.2}us to find tau={:?}", elapsed.as_nanos() as f64 / 1_000., tau);

    let start = Instant::now();
    let _ = mb.fwd_kin(&[-1.;7]);
    let _ = mb.fwd_kin(&[ 0.;7]);
    let tr = mb.fwd_kin(&[ 1.;7]);
    let elapsed = start.elapsed();
    println!("fwd_kin: took {:.2}us to find EE={:?}", elapsed.as_nanos() as f64 / 3_000., tr.translation);
}

fn main() {
    let q = AutoF32x8::ZERO;
    
    // Run SIMD version
    //let simd_result = vec_simd(q);
    //println!("{:?}", simd_result);

    // Run iterative version
    //let iter_result = vec_iter(&[0.0; 8]);
    //println!("{:?}", iter_result);
    

}
