#![feature(autodiff)]
use std::autodiff::autodiff;

use rigidbody::Multibody;

//use std::autodiff::autodiff;
use std::path::Path;
use std::time::Instant;

#[autodiff]
fn square(x: f32) -> f32 {
    x*x
}



fn main() {
    let t = 1.;
    println!("fn: {:?}, diff: {:?}", square(t), square(t));

}
