#![feature(autodiff)]

use std::autodiff::autodiff;


#[autodiff]
fn square(x: f32) -> f32 {
    x*x
}

fn main() {    
    let t = 1.;
    println!("fn: {:?}, diff: {:?}", square(t), square(t));
}
