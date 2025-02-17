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
pub struct Transfo<'a> {
    mydata: &'a Real,
}

impl <'a> Transfo<'a> {
    pub fn from_angle(q: &'a Real) -> Transfo<'a> {
        Transfo { mydata: q }
    }

    pub fn get_data(&self) -> &'a Real {
        self.mydata
    }
}

#[derive(Debug)]
pub struct Transforms<'a>([Transfo<'a>; 7]);

impl <'a> Transforms<'a> {
    pub fn from_joints(mb: &Multibody, q: &'a [Real]) -> Self {
        let arr: [Transfo; 7] = zip(mb.iter(), q.iter())
            .map(|(jt, qi)| { Transfo::from_angle(qi) })//jt.parent_to_child(qi)})
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
    fn transfo () {
        let mut t;
        
        {
            let r = 0.1;
            t = Transfo::from_angle(&r);
            println!("{:?}", t.get_data());
        }
//        println!("{:?}", t.get_data());  // would cause error
    }
    
    #[test]
    fn transforms () {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let mut ts;
        {
            let q = [0.1; 7];
            ts = Transforms::from_joints(&mb, &q);
            println!("{:?}", ts);
        }
  //      println!("{:?}", ts); // would cause error
        
    }
}
