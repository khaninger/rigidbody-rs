#![feature(autodiff)]

#[autodiff]
fn sqr(x: f64) -> f64 {
    x*x
}

fn main() {
    println!("{:?}", sqr(3.));
}
