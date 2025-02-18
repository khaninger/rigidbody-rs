//joint.rs
use std::path::Path;
use std::iter::{zip, Iterator};
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

/// An explicit revolute joint with a single degree of freedom
#[derive(Debug)]
pub struct RevoluteJoint  {
    pub axis: Unit<Vector3<Real>>, // Normed axis for the revolute joint
    pub parent: Transform, // Pose of joint coord system relative to parent
    pub body: Inertia,
}

impl RevoluteJoint {
    /// Transformation from the child coordinate system to the parent, used for reverse iteration
    #[inline(always)]
    pub fn parent_to_child(&self, q: Real) -> Transform {
        self.parent*&self.joint_transform(q)
    }
    
    /// Transformation from parent coordinate system to the child
    #[inline(always)]
    pub fn child_to_parent(&self, q: Real) -> Transform {
        self.joint_transform(-q)*(self.parent.inverse())
    }

    /// Transformation of the joint itself
    #[inline(always)]
    pub fn joint_transform(&self, q: Real) -> UnitQuaternion<Real> {
        UnitQuaternion::from_scaled_axis(self.axis.scale(q))
    }

    
    pub fn from_xurdf_joint(
        joint: &Joint, link: &Link,
    ) -> RevoluteJoint {
        let axis = UnitVector3::<Real>::new_normalize(convert(joint.axis));
        let parent = Transform::new(
            convert(joint.origin.xyz),
            Rotation3::<Real>::from_euler_angles(
                joint.origin.rpy[0] as Real,
                joint.origin.rpy[1] as Real,
                joint.origin.rpy[2] as Real
            ).scaled_axis()
        );
        
        let body  = Inertia::from_com(link.inertial.mass, link.inertial.origin.xyz, link.inertial.inertia);        
        RevoluteJoint{axis, parent, body}
    }
}
