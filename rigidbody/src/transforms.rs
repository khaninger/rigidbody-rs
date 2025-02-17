// Minimal test with transformations
use std::iter::zip;
use nalgebra::{
    UnitVector3,
    Isometry3,
    Vector3,
    Translation3,
    Rotation3,
    Matrix3,
    UnitQuaternion,
    Quaternion,
    Unit,
    OPoint,
    U3,
    convert
};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;
use itertools::izip;
use crate::spatial::{SpatialForce, SpatialVelocity, BodyJacobian};
use crate::{Real, Transform};
use crate::inertia::Inertia;
use crate::Multibody;

#[derive(Debug)]
pub struct Transforms([Transform; 7]);

impl Transforms {
    pub fn from_joints(mb: &Multibody, q: &[Real]) -> Self {
        let arr: [Transform; 7] = zip(mb.iter(), q.iter())
            .map(|(jt, &qi)| { jt.parent_to_child(qi)})
            .collect::<Vec<_>>()
            .try_into()
            .expect("Build transforms");
        Transforms(arr)
    }
}


#[cfg(test)]
mod test{
    use super::*;
    use std::path::Path;

    #[test]
    fn transforms () {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let ts = Transforms::from_joints(&mb, &[0.; 7]);
        println!("{:?}", ts);
    }
}
