use std::ops::{Mul, Add, AddAssign};
use nalgebra::{convert, OPoint, UnitVector3, Vector3, U3, Isometry3, Translation3, Point3};

type Real = f32;
type Transform = Isometry3<Real>;

#[derive(Debug, Default, Clone)]
pub struct SpatialVelocity {
    pub lin: Vector3<Real>,
    pub rot: Vector3<Real>,
}

impl Mul<&SpatialVelocity> for Transform {
    type Output = SpatialVelocity;

    fn mul(self, v: &SpatialVelocity) -> SpatialVelocity {
        v.transform(&self)
    }
}

/// A spatial velocity denotes the speed of a rigid body in a fixed coordinate system.
/// Linear translation expressed in self.coord, and rotation is about the axes of self.coord 
impl SpatialVelocity {
    pub fn new() -> Self {
        SpatialVelocity{lin:Vector3::new(0.,0.,0.), rot:Vector3::new(0.,0.,0.)}
    }

    /// Transform the spatial velocity from the coordinate system in self.coord to world
    pub fn transform(&self, tr: &Transform) -> Self {
        SpatialVelocity {
            lin: tr.rotation*(self.lin-tr.translation.vector.cross(&self.rot)),
            rot: tr.rotation*self.rot,
        }
    }

    ///Cross product with another spatial velocity
    pub fn cross(&self, other: &SpatialVelocity) -> SpatialVelocity {
        SpatialVelocity {
            lin: self.lin.cross(&other.rot) + other.lin.cross(&self.rot),
            rot: self.rot.cross(&other.rot)
        }
    }
}

impl Mul<Real> for SpatialVelocity {
    type Output = SpatialVelocity;

    fn mul(self, a: Real) -> SpatialVelocity {
        SpatialVelocity{lin: &self.lin*a, rot: &self.rot*a}
    }
}

impl Add<SpatialVelocity> for SpatialVelocity {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {lin: self.lin+other.lin, rot: self.rot+other.rot}
    }
}

impl Mul<&SpatialForce> for SpatialVelocity {
    type Output = Real;

    fn mul(self, f: &SpatialForce) -> Real {
        *(self.lin.transpose()*&f.lin + self.rot.transpose()*&f.rot).as_scalar()
    }
}

/// A body jacobian multiplied by a joint vel or acc yields a SpatialVelocity, hacky shortcut for now
pub type BodyJacobian = SpatialVelocity;

impl BodyJacobian {
    pub const fn revolute_z() -> Self {
        return BodyJacobian{lin: Vector3::new(0., 0., 0.), rot: Vector3::new(0.,0.,1.)}
    }

}

#[derive(Debug, Default, Clone)]
pub struct SpatialForce {
    pub lin: Vector3<Real>,
    pub rot: Vector3<Real>,
}


impl Add<&SpatialForce> for SpatialForce {
    type Output = Self;

    fn add(self, other: &Self) -> Self {
        Self {lin: self.lin+other.lin, rot: self.rot+other.rot}
    }
}

impl AddAssign for SpatialForce {
    fn add_assign(&mut self, other: Self) {
        self.lin += other.lin;
        self.rot += other.rot
    }
}

impl Mul<&SpatialForce> for Transform {
    type Output = SpatialForce;

    fn mul(self, f: &SpatialForce) -> SpatialForce {
        f.transform(&self)
    }
}

impl SpatialForce {
    /// (2.25) Transform the spatial force from the coordinate system in self.coord to world
    pub fn transform(&self, tr: &Transform) -> Self {
        SpatialForce {
            lin: tr.rotation*self.lin,
            rot: tr.rotation*(self.rot-tr.translation.vector.cross(&self.lin)),
        }
    }

    /// (2.27)
    pub fn inv_transform(&self, tr: &Transform) -> Self {
        let rot_inv = tr.rotation.inverse();
        let new_lin = rot_inv*self.lin;
        SpatialForce {
            lin: new_lin,
            rot: rot_inv*self.rot + tr.translation.vector.cross(&new_lin)
        }
    }
}
//TODO Not fully implemented/tested
fn vel_cross() {
    let v1 = Vector3::new(0., 0., 1.);
    let v2 = Vector3::new(1., 2., 3.);
    let T: Isometry3<f32> = Isometry3::identity();
    
    let sv1 = SpatialVelocity {
        lin: Vector3::new(0., 0., 1.),
        rot: Vector3::new(0., 0., 0.)
    };
    let sv2 = SpatialVelocity {
        lin: Vector3::new(0., 1., 0.),
        rot: Vector3::new(0., 0., 0.)
    };

    //println!("{:?}", sv1.cross(&sv2));
}

#[test]
fn vel_transform() {
    use nalgebra::UnitQuaternion;
    let v = SpatialVelocity{
        lin: Vector3::new(0., 1., 0.),
        rot: Vector3::new(1., 0., 0.)
    };

    // Translation
    let X1 = Transform{
        rotation: UnitQuaternion::identity(),
        translation: Translation3::new(0., 1., 0.)
    };
    
    let eps = 0.0001;
    let v1 = X1*&v;
    assert!(v1.lin.relative_eq(&Vector3::new(0., 1., 1.), eps, eps));
    assert!(v1.rot.relative_eq(&v.rot, eps, eps));
        
    let X2 = Transform {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(),
                                                  -std::f32::consts::FRAC_PI_2),
        translation: Translation3::new(0.,0.,0.)
    };
    let v2 = X2*&v;
    assert!(v2.lin.relative_eq(&Vector3::new(1., 0., 0.), eps, eps));
    assert!(v2.rot.relative_eq(&Vector3::new(0., -1., 0.), eps, eps));
}

#[test]
fn force_transform() {
    use nalgebra::UnitQuaternion;
    let f = SpatialForce{
        lin: Vector3::new(0., 1., 0.),
        rot: Vector3::new(1., 0., 0.)
    };

    // Translation
    let X1 = Transform{
        rotation: UnitQuaternion::identity(),
        translation: Translation3::new(1., 0., 0.)
    };
    
    let eps = 0.0001;
    let f1 = X1*&f;
    assert!(f1.lin.relative_eq(&f.lin, eps, eps));
    assert!(f1.rot.relative_eq(&Vector3::new(1., 0., -1.), eps, eps));
        
    let X2 = Transform {
        //WARNING! This friendly constructor constructs a Quat which rotates vectors abt z by -pi/2
        //         It is the inverse of constructing a rotation matrix about z for the coord transform
        rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(),
                                                  -std::f32::consts::FRAC_PI_2),
        translation: Translation3::new(0.,0.,0.)
    };
    let f2 = X2*&f;
    assert!(f2.lin.relative_eq(&Vector3::new(1., 0., 0.), eps, eps));
    assert!(f2.rot.relative_eq(&Vector3::new(0., -1., 0.), eps, eps));
}
