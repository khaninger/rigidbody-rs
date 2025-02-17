use std::ops::{Mul, Add, AddAssign};
use nalgebra::{OPoint, U3, Matrix3, Matrix6, Vector3, Translation3, UnitQuaternion, convert};
use crate::{Real, Transform};
use crate::spatial::{
    SpatialVelocity,
    SpatialForce,
    transform_to_A_X_B,
    transform_to_B_X_A_star
};


#[derive(Debug, Default, Clone)]
pub struct Inertia {
    pub mass: Real,                // Mass
    pub com: OPoint<Real, U3>,     // COM in origin coordinates
    pub inertia_com: Matrix3<Real>,// Inertia about COM
    inertia: Matrix3<Real>         // Inertia about origin 
}

impl Inertia {
    pub fn from_com<M, C, I>(mass: M, com: C, inertia_com: I) -> Self
    where
        M: Into<Real>,
        C: Into<OPoint<Real, U3>>,
        I: Into<Matrix3<Real>>,
    {
        let mass:Real = mass.into();
        let com:OPoint<Real, U3> = com.into();
        let inertia_com:Matrix3<Real> = inertia_com.into();

        let translate_cr = com.coords.cross_matrix();
        let inertia = inertia_com + mass*translate_cr*(translate_cr.transpose());

        Inertia { mass, com, inertia_com, inertia}
    }

    pub fn from_origin<M, C, I>(mass: M, com: C, inertia: I) -> Self
    where
        M: Into<Real>,
        C: Into<OPoint<Real, U3>>,
        I: Into<Matrix3<Real>>,
    {
        let mass:Real = mass.into();
        let com:OPoint<Real, U3> = com.into();
        let inertia:Matrix3<Real> = inertia.into();

        let translate_cr = com.coords.cross_matrix();
        let inertia_com = inertia - mass*translate_cr*(translate_cr.transpose());

        Inertia { mass, com, inertia_com, inertia}
    }
    
    pub fn to_matrix6(self) -> Matrix6<Real> {
        let m = self.mass;
        let c = self.com.coords;
        let c_cross = c.cross_matrix();
        let ul = self.inertia;
        let ur = m * c_cross;
        let ll = m * c_cross.transpose();

        Matrix6::<Real>::new(
            ul[(0,0)], ul[(0,1)], ul[(0,2)],   ur[(0,0)], ur[(0,1)], ur[(0,2)],
            ul[(1,0)], ul[(1,1)], ul[(1,2)],   ur[(1,0)], ur[(1,1)], ur[(1,2)],
            ul[(2,0)], ul[(2,1)], ul[(2,2)],   ur[(2,0)], ur[(2,1)], ur[(2,2)],

            ll[(0,0)], ll[(0,1)], ll[(0,2)],   m, 0., 0.,
            ll[(1,0)], ll[(1,1)], ll[(1,2)],   0., m, 0.,
            ll[(2,0)], ll[(2,1)], ll[(2,2)],   0., 0., m,
        )
    }        

    /// Transform inertia to a different coordinate system.
    /// (2.66): ^BI = ^BX_A^* ^AI ^AX_B
    ///         if tr is ^BX_A;
    ///         ^BX_A^* = [tr.rot, -tr.rot*tr.lin.cross_matrix();
    ///                         0, tr.rot];
    ///         ^AX_B   = [tr.rot.T, 0;
    ///                    tr.lin.cross_matrix()*tr.rot.T, tr.rot.T];
    ///         (^AI*^AX_B) = [I.inertia*tr.rot.T, 0;
    ///                        I.m*tr.lin.cm()*tr.rot.T, I.m*tr.rot.T]    
    pub fn transform(&self, tr: Transform) -> Self {
        let translate_cr = tr.translation.vector.cross_matrix();
        let rot = tr.rotation.to_rotation_matrix();

        //let inertia = rot*(self.inertia+self.mass*translate_cr*translate_cr.transpose())*(rot.transpose());
        //let cross_term = rot*translate_cr*(-self.mass)*(rot.transpose());
        let com = tr*self.com;
        Inertia::from_com(self.mass.clone(), com, rot*self.inertia_com.clone()*(rot.transpose()))
    }

    pub fn get_rotz(&self) -> Real {
        self.inertia[(2,2)]
    }
}

impl Add<Inertia> for Inertia {    
    type Output = Inertia;
    fn add(self, other: Inertia) -> Inertia {
        let com: OPoint<Real, U3> = ((self.mass*self.com.coords+other.mass*other.com.coords)/(self.mass+other.mass)).into();
        let mass = self.mass + other.mass;
        let inertia = self.inertia + other.inertia;
        
        Inertia::from_origin(mass, com, inertia)
    }
}

impl Mul<&SpatialVelocity> for &Inertia {
    type Output = SpatialForce;

    /// Return the forces from an acceleration expressed in origin of Inertia 
    fn mul(self, a: &SpatialVelocity) -> SpatialForce {
        SpatialForce { //(2.63)
            lin: self.mass*a.lin - self.mass*self.com.coords.cross(&a.rot),
            rot: self.inertia*a.rot + self.mass*self.com.coords.cross(&a.lin)
        }
    }
}

#[test]
fn test_inertia_transform() {
    let rb = Inertia::from_com(0.5, OPoint::<Real, U3>::new(0.,0.,1.), Matrix3::<Real>::new(0.1, 0.,0.,0.,0.2,0.,0.,0.,0.3));

    let rb6 = rb.clone().to_matrix6();
    let tr = Transform { translation:Translation3::new(0., 0., 1.),
                         rotation:UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0., 0.)};
    let rb2 = rb.clone().transform(tr.clone());

    let tr_inv = tr.inverse();
    let bxastar = transform_to_B_X_A_star(tr_inv.clone());
    let axb = transform_to_A_X_B(tr_inv);

    //println!("rb6: {}", rb6);
    //println!("axb: {}", axb);
    //println!("rb6*axb: {}", rb6*axb);    
    //println!("bxastar: {}", bxastar);
    let rb26_ = bxastar*rb6*axb;
    let rb26 = rb2.clone().to_matrix6();
    
    //println!("feather: {}", rb26_);
    //println!("hand:    {}", rb26);
}

#[test]
fn test_inertia_add() {
    let rb = Inertia::from_com(0.5,
                               OPoint::<Real, U3>::new(0.,0.,1.),
                               Matrix3::<Real>::new(0.1,0.,0.,0.,0.2,0.,0.,0.,0.3));
    let rb2 = Inertia::from_com(1.5,
                               OPoint::<Real, U3>::new(1.,0.,0.),
                               Matrix3::<Real>::new(1.1,0.,0.,0.,1.2,0.,0.,0.,1.3));

    let rb6 = rb.clone().to_matrix6();
    let rb26 = rb2.clone().to_matrix6();

    //println!("feather add: {}", rb6+rb26);
//    println!("hand add:    {}", (rb+rb2).to_matrix6());    
}

