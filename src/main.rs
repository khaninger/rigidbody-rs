#![feature(autodiff)]
use std::autodiff::autodiff;

#[autodiff(df, Reverse, Duplicated, Active)]
fn f(x: &[f32; 2]) -> f32 {
    x[0] * x[0] + x[1] * x[0]
}

fn main() {
    let x = [2.0, 3.0];
    let mut bx = [0.0, 0.0];
    let by = 1.0; // seed
    let y = df(&x, &mut bx, by);
    assert_eq!([7.0, 2.0], bx);
    assert_eq!(10.0, y);
}
