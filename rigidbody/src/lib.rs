#![allow(warnings)] // Suppress warning message on compile
#![feature(test)]

mod spatial;
mod joint;
mod rigidbody;
pub mod multibody;

pub use spatial::*;
pub use joint::*;
pub use multibody::*;

pub type Real = f32;
pub type Transform = nalgebra::Isometry3<Real>;


