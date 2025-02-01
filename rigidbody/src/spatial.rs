use std::convert::From;
use std::ops::{Mul, Add, AddAssign};
use nalgebra::{
    convert,
    OPoint,
    UnitVector3,
    Vector3,
    Vector6,
    Matrix3,
    Matrix6,
    U3,
    Isometry3,
    Translation3,
    Rotation3,
    Point3,
    UnitQuaternion
};

type Real = f32;
type Transform = Isometry3<Real>;

impl Mul<&SpatialVelocity> for Transform {
    type Output = SpatialVelocity;

    fn mul(self, v: &SpatialVelocity) -> SpatialVelocity {
        v.transform(&self)
    }
}

/// Create Featherstone 6x6 matrix (2.24) from Isometry3.
///   where T is the transformation of frame A to B
/// Can't use a standard Into because Matrix6 and Isometry3
///   are not in our crate :(
fn transform_to_B_X_A(T: Transform) -> Matrix6<Real> {
    let E = T.rotation.inverse().to_rotation_matrix();
    let r = T.translation.vector;
    let r_cross = Matrix3::<Real>::new(   0., -r[2],  r[1],
                                        r[2],    0., -r[0],
                                       -r[1],  r[0],    0.);
    let Er_cross = E*r_cross;
    Matrix6::<Real>::new(
        E[(0, 0)], E[(0, 1)], E[(0, 2)], 0.0, 0.0, 0.0,
        E[(1, 0)], E[(1, 1)], E[(1, 2)], 0.0, 0.0, 0.0,
        E[(2, 0)], E[(2, 1)], E[(2, 2)], 0.0, 0.0, 0.0,
        -Er_cross[(0, 0)], -Er_cross[(0, 1)], -Er_cross[(0, 2)], E[(0, 0)], E[(0, 1)], E[(0, 2)],
        -Er_cross[(1, 0)], -Er_cross[(1, 1)], -Er_cross[(1, 2)], E[(1, 0)], E[(1, 1)], E[(1, 2)],
        -Er_cross[(2, 0)], -Er_cross[(2, 1)], -Er_cross[(2, 2)], E[(2, 0)], E[(2, 1)], E[(2, 2)],
    )
}

/// Create Featherstone 6x6 matrix (2.25) from Isometry3.
///   where T is the transformation of frame A to B
/// Can't use a standard Into because Matrix6 and Isometry3
///   are not in our crate :(
fn transform_to_B_X_A_star(T: Transform) -> Matrix6<Real>{
    let E = T.rotation.inverse().to_rotation_matrix();
    let r = T.translation.vector;
    let r_cross = Matrix3::<Real>::new(   0., -r[2],  r[1],
                                        r[2],    0., -r[0],
                                       -r[1],  r[0],    0.);
    let Er_cross = E*r_cross;
    Matrix6::<Real>::new(
        E[(0, 0)], E[(0, 1)], E[(0, 2)], -Er_cross[(0, 0)], -Er_cross[(0, 1)], -Er_cross[(0, 2)],
        E[(1, 0)], E[(1, 1)], E[(1, 2)], -Er_cross[(1, 0)], -Er_cross[(1, 1)], -Er_cross[(1, 2)], 
        E[(2, 0)], E[(2, 1)], E[(2, 2)], -Er_cross[(2, 0)], -Er_cross[(2, 1)], -Er_cross[(2, 2)], 
        0.0, 0.0, 0.0, E[(0, 0)], E[(0, 1)], E[(0, 2)], 
        0.0, 0.0, 0.0, E[(1, 0)], E[(1, 1)], E[(1, 2)],
        0.0, 0.0, 0.0, E[(2, 0)], E[(2, 1)], E[(2, 2)],
    )
}

#[derive(Debug, Default, Clone)]
pub struct SpatialVelocity {
    pub lin: Vector3<Real>,
    pub rot: Vector3<Real>,
}

/// A spatial velocity denotes the speed of a rigid body in a fixed coordinate system.
/// Linear translation expressed in self.coord, and rotation is about the axes of self.coord 
impl SpatialVelocity {
    pub fn new() -> Self {
        SpatialVelocity{lin:Vector3::new(0.,0.,0.), rot:Vector3::new(0.,0.,0.)}
    }

    /// Transform the spatial velocity from the coordinate system in self.coord to world
    #[inline(always)]
    pub fn transform(&self, tr: &Transform) -> Self {
        let rot = tr.rotation.inverse();
        SpatialVelocity {
            lin: rot*(self.lin-tr.translation.vector.cross(&self.rot)),
            rot: rot*self.rot,
        }
    }

    ///(2.33) Cross product with another spatial velocity
    #[inline(always)]
    pub fn cross(&self, other: &SpatialVelocity) -> SpatialVelocity {
        SpatialVelocity {
            lin: self.lin.cross(&other.rot) + other.lin.cross(&self.rot),
            rot: self.rot.cross(&other.rot)
        }
    }

    /// (2.34) Cross product with a spatial force
    #[inline(always)]
    pub fn cross_star(&self, f: &SpatialForce) -> SpatialForce {
        SpatialForce {
            lin: self.rot.cross(&f.lin),
            rot: self.rot.cross(&f.rot) + self.lin.cross(&f.lin)
        }       
    }
}

impl From<Vector6<Real>> for SpatialVelocity {
    fn from(v: Vector6<Real>) -> SpatialVelocity {
        SpatialVelocity { lin: Vector3::new(v[3], v[4], v[5]),
                          rot: Vector3::new(v[0], v[1], v[2]) }
    }
}

impl From<SpatialVelocity> for Vector6<Real> {
    fn from(v: SpatialVelocity) -> Vector6<Real> {
        Vector6::<Real>::new(v.rot[0], v.rot[1], v.rot[2],
                             v.lin[0], v.lin[1], v.lin[2])
    }
}


impl Mul<Real> for SpatialVelocity {
    type Output = SpatialVelocity;

    #[inline(always)]
    fn mul(self, a: Real) -> SpatialVelocity {
        SpatialVelocity{lin: &self.lin*a, rot: &self.rot*a}
    }
}

impl Add<&SpatialVelocity> for SpatialVelocity {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: &Self) -> Self {
        Self {lin: self.lin+other.lin, rot: self.rot+other.rot}
    }
}

impl Mul<&SpatialForce> for SpatialVelocity {
    type Output = Real;

    #[inline(always)]
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

#[derive(Debug, Default, Clone, Copy)]
pub struct SpatialForce {
    pub lin: Vector3<Real>,
    pub rot: Vector3<Real>,
}


impl Add<&SpatialForce> for SpatialForce {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: &Self) -> Self {
        Self {lin: self.lin+other.lin, rot: self.rot+other.rot}
    }
}

impl AddAssign for SpatialForce {
    #[inline(always)]
    fn add_assign(&mut self, other: Self) {
        self.lin += other.lin;
        self.rot += other.rot
    }
}

impl Mul<&SpatialForce> for Transform {
    type Output = SpatialForce;

    #[inline(always)]
    fn mul(self, f: &SpatialForce) -> SpatialForce {
        f.transform(&self)
    }
}


impl From<Vector6<Real>> for SpatialForce {
    fn from(f: Vector6<Real>) -> SpatialForce {
        SpatialForce { lin: Vector3::new(f[3], f[4], f[5]),
                       rot: Vector3::new(f[0], f[1], f[2]) }
    }
}

impl From<SpatialForce> for Vector6<Real> {
    fn from(f: SpatialForce) -> Vector6<Real> {
        Vector6::<Real>::new(f.rot[0], f.rot[1], f.rot[2],
                             f.lin[0], f.lin[1], f.lin[2])
    }
}

impl SpatialForce {
    pub fn new() -> Self {
        SpatialForce { lin: Vector3::new(0.,0.,0.), rot: Vector3::new(0.,0.,0.) }
    }
    
    /// (2.25) Transform the spatial force from the coordinate system in self.coord to world
    #[inline(always)]
    pub fn transform(&self, tr: &Transform) -> Self {
        let rot = tr.rotation.inverse();
        SpatialForce {
            lin: rot*self.lin,
            rot: rot*(self.rot-tr.translation.vector.cross(&self.lin)),
        }
    }

    /// (2.27)
    #[inline(always)]
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

/// Check if the conventions used match that of Featherstone
/// -> Featherstone's convention is for _elements_, not a transformation
///    of the coordinate frame. This is the inverse of typical
///    convention, incl. in nalgebra.
#[test]
fn coord_transforms() {
    use nalgebra::{UnitQuaternion, Matrix3};
    use std::f32::*;

    // Pure rotation
    let theta = std::f32::consts::FRAC_PI_2;
    let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(),
                                              -theta);
    let rz = |t: f32| -> Matrix3<f32> { Matrix3::new( t.cos(), t.sin(), 0.,
                                                     -t.sin(), t.cos(), 0.,
                                                      0.,      0.,      1.) };
    assert!(rot.to_rotation_matrix().matrix().relative_eq(&rz(theta), 1e-5, 1e-5));

    // Just make sure nothing funky is happening in multiplying w/ nalgebra
    let x = Vector3::new(1.,0.,0.);
    assert!((rot*x).relative_eq(&Vector3::new(0., -1., 0.), 1e-5, 1e-5));
}

#[test]
fn vel_transform() {
    use nalgebra::{Matrix3, Matrix6, Vector6};
    let eps = 0.0001;
    
    let v = SpatialVelocity{
        lin: *Vector3::y_axis(),
        rot: *Vector3::x_axis()
    };
    let v_v6: Vector6<Real> = v.clone().into();
    
    // Translation
    let X1 = Transform{
        translation: Translation3::new(0., 1., 0.),
        ..Default::default()
    };
    let X1_feath = transform_to_B_X_A(X1);
   
    let v1 = X1*&v;
    let v1_feath_v6: Vector6<Real> = X1_feath*v_v6;
    let v1_feath: SpatialVelocity = v1_feath_v6.into();

    assert!(v1.rot.relative_eq(&v1_feath.rot, eps, eps));
    assert!(v1.lin.relative_eq(&v1_feath.lin, eps, eps));

    let theta = std::f32::consts::FRAC_PI_2;
    let X2 = Transform {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta),
        translation: Translation3::new(0.,0.,0.)
    };
    let X2_feath = transform_to_B_X_A(X2);
    
    let v2 = X2*&v;    
    let v2_feath_v6 = X2_feath*v_v6;
    let v2_feath: SpatialVelocity = v2_feath_v6.into();
    
    assert!(v2.rot.relative_eq(&v2_feath.rot, eps, eps));
    assert!(v2.lin.relative_eq(&v2_feath.lin, eps, eps));
}

#[test]
fn force_transform() {
    use nalgebra::UnitQuaternion;

    let eps = 0.0001;
    let f = SpatialForce{
        lin: Vector3::new(0., 1., 0.),
        rot: Vector3::new(1., 0., 0.)
    };
    let f_v6: Vector6<Real> = f.clone().into();
    
    // Translation
    let X1 = Transform{
        rotation: UnitQuaternion::identity(),
        translation: Translation3::new(1., 0., 0.)
    };
    let X1_feath = transform_to_B_X_A_star(X1.clone());

    let f1 = X1*&f;
    let f1_feath_v6 = X1_feath*f_v6;
    let f1_feath:SpatialForce = f1_feath_v6.into();

    assert!(f1.rot.relative_eq(&f1_feath.rot, eps, eps));
    assert!(f1.lin.relative_eq(&f1_feath.lin, eps, eps));

    
    let theta = std::f32::consts::FRAC_PI_2;
    let X2 = Transform {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::z_axis(),
                                                  theta),
        translation: Translation3::new(0.3,0.5,0.)
    };
    let X2_feath = transform_to_B_X_A_star(X2.clone());
    let f2 = X2*&f;
    let f2_feath_v6 = X2_feath*f_v6;
    let f2_feath: SpatialForce = f2_feath_v6.into();
    
    println!("{:?}\n{:?}", f2, f2_feath);
    assert!(f2.rot.relative_eq(&f2_feath.rot, eps, eps));
    assert!(f2.lin.relative_eq(&f2_feath.lin, eps, eps));
}
