use std::ops::Mul;
use nalgebra::{convert, OPoint, UnitVector3, Vector3, U3, Isometry3};

type Real = f32;
type LinVel = Vector3<Real>;
type Transform = Isometry3<Real>;

/*pub struct IsometryGraph {
    transforms: Vec<Isometry3>
}*/

#[derive(Debug)]
pub struct SpatialVelocity {
    pub lin: LinVel,
    pub rot: Vector3<Real>,
}


impl Mul<SpatialVelocity> for Transform {
    type Output = SpatialVelocity;

    fn mul(self, v: SpatialVelocity) -> SpatialVelocity {
        v.transform(&self)
    }
}

/// A spatial velocity denotes the speed of a rigid body in a fixed coordinate system.
/// Linear translation expressed in self.coord, and rotation is about the axes of self.coord 
impl SpatialVelocity {
    /// Transform the spatial velocity from the coordinate system in self.coord to world
    pub fn transform(&self, tr: &Transform) -> Self {
        SpatialVelocity {
            lin: tr.rotation*(self.lin), //-tr.translation.cross(self.rot)),
            rot: tr.rotation*self.rot,
        }
    }
}

pub struct SpatialForce {
    lin: Vector3<Real>,
    rot: Vector3<Real>,
    //coord: &Transform
}
