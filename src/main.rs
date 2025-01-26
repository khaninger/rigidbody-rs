use rigidbody::Multibody;

use std::path::Path;
use std::time::Instant;

fn main() {
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
