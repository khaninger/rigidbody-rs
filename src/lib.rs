#![allow(warnings)] // Suppress warning message on compile
#![feature(test)]

use std::path::Path;
use std::iter::zip;
use std::time::{Duration, Instant};

mod spatial;
mod joint;
mod rigidbody;
pub mod multibody;

pub use spatial::*;
pub use joint::*;
pub use multibody::*;

use nalgebra::{Vector3, Isometry3};

pub type Real = f32;
pub type Transform = Isometry3<Real>;

#[no_mangle]
pub extern "C" fn multibody_new() -> *mut Multibody {
    println!("We're doing it, man!");
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    Box::into_raw(Box::new(mb))
}

#[no_mangle]
pub extern "C" fn multibody_rnea<'a>(q: &'a [f32; 7],
                                     dq: &'a [f32; 7],
                                     ddq: &'a [f32; 7]) -> [f32; 7] {
    println!("REally doing it! {:?}, {:?}, {:?}", q, dq, ddq);
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    mb.rnea(q, dq, dq)
}

#[no_mangle]
pub extern "C" fn multibody_free(mb_ptr: *mut Multibody) {
    if !mb_ptr.is_null() {
        drop( unsafe { Box::from_raw(mb_ptr) } );
    }
}
