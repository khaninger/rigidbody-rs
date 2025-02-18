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
use crate::RevoluteJoint;
use crate::inertia::Inertia;
use crate::Multibody;

#[derive(Debug)]
pub struct Transforms<'a>(&'a [Transform; 7]);

pub fn make_transform<'a> (q: &'a Real, jt: &RevoluteJoint, T: &mut Transform) {
    *T = jt.parent_to_child(*q);
}

impl <'a> Transforms<'a> {
    pub fn from_joints(mb: &Multibody, q: &[Real], arr: &'a mut [Transform; 7]) -> Self {
        izip!(mb.iter(), q.iter(), arr.iter_mut())
            .map(|(jt, qi, tr)| { make_transform(qi, jt, tr) } );
        Transforms(arr)
    }
}


#[cfg(test)]
mod test{
    use super::*;
    use std::path::Path;
    
    #[test]
    fn transforms () {
        let mut arr: [Transform; 7] = [Default::default(); 7];
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let mut ts;
        {
            let q = [0.1; 7];
            ts = Transforms::from_joints(&mb, &q, &mut arr);
            println!("{:?}", ts);
        }
        println!("{:?}", ts);
        
    }
}
