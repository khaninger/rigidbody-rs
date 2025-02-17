#![allow(warnings)] // Suppress warning message on compile
//#![feature(test)]
//#![feature(slice_ptr_get)]

mod spatial;
mod joint;
mod inertia;
pub mod multibody;

pub use spatial::*;
pub use joint::*;
pub use multibody::*;

pub type Real = f64;
pub type Transform = nalgebra::Isometry3<Real>;
