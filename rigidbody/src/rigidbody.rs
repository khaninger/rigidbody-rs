use std::ops::Mul;
use nalgebra::{OPoint, U3, Matrix3};
use crate::{Real, Transform};
use crate::spatial::{SpatialVelocity, SpatialForce};

#[derive(Debug, Default)]
pub struct RigidbodyInertia {
    pub mass: Real,               // Upper block diagonal
    pub com: OPoint<Real, U3>, 
    pub inertia: Matrix3<Real>,   // Lower block diagonal
    pub cross_term: Matrix3<Real> // Upper right block
        
}
    
impl RigidbodyInertia {
    /// (2.66): ^BI = ^BX_A^* ^AI ^AX_B
    ///         if tr is ^BX_A;
    ///         ^BX_A^* = [tr.rot, -tr.rot*tr.lin.cross_matrix();
    ///                         0, tr.rot];
    ///         ^AX_B   = [tr.rot.T, 0;
    ///                    tr.lin.cross_matrix()*tr.rot.T, tr.rot.T];
    ///         (^AI*^AX_B) = [I.inertia*tr.rot.T, 0;
    ///                        I.m*tr.lin.cm()*tr.rot.T, I.m*tr.rot.T]
    fn transform(self, tr: Transform) -> Self {
        let translate_cr = tr.translation.vector.cross_matrix();
        let rot = tr.rotation.to_rotation_matrix();
        let inertia = rot*(self.inertia+self.mass*translate_cr*translate_cr.transpose())*(rot.transpose());
        let cross_term = rot*translate_cr*(-self.mass)*(rot.transpose());

        RigidbodyInertia { cross_term, inertia, ..self }
    }
}

impl Mul<&SpatialVelocity> for &RigidbodyInertia {
    type Output = SpatialForce;

    fn mul(self, v: &SpatialVelocity) -> SpatialForce {
        SpatialForce { //(2.63)
            lin: v.lin*self.mass - self.mass*self.com.coords.cross(&v.rot),
            rot: self.inertia*v.rot + self.com.coords.cross(&v.lin)*self.mass
        }
    }
}


