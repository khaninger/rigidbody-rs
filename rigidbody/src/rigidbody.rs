use std::ops::Mul;
use nalgebra::{OPoint, U3, Matrix3};
use crate::{Real};
use crate::spatial::{SpatialVelocity, SpatialForce};

#[derive(Debug)]
pub struct Rigidbody {
    pub mass: Real,
    pub com: OPoint<Real, U3>,
    pub inertia: Matrix3<Real>
}
    
    
impl Mul<&SpatialVelocity> for &Rigidbody {
    type Output = SpatialForce;

    fn mul(self, v: &SpatialVelocity) -> SpatialForce {
        SpatialForce { //(2.63)
            lin: v.lin*self.mass - self.mass*self.com.coords.cross(&v.rot),
            rot: self.inertia*v.rot + self.com.coords.cross(&v.lin)*self.mass
        }
    }
}


