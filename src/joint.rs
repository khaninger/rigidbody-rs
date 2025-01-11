use nalgebra::{UnitVector3, Isometry3, Vector3, Translation3, UnitQuaternion, Unit, convert};
use xurdf::{Link, Joint};

pub type Real = f32;
pub type Transform = Isometry3<Real>;


/// A pose expressed in a coordinate system. 
#[derive(Debug)]
pub struct Pose<'a> {
    pub coord_frame: &'a Transform, // Use reference so the validity of the transform is checked by borrow checker
    pub pose: Transform
}

impl <'a> Pose<'a>{
    pub fn to_world(&self) -> Transform {
        self.coord_frame.inv_mul(&self.pose)
    }
}

/// An explicit revolute joint with a single degree of freedom
pub struct RevoluteJoint<'a>  {
    pub axis: Unit<Vector3<Real>>,
    pub parent_anchor: Pose<'a>, // Pose of joint relative to parent
    pub child_frame: Transform
}


impl <'b> RevoluteJoint<'b> {
    /// A pose expressed in the child coord system to a pose in the parent
    pub fn child_to_parent(&self, q: Real, child: Pose) -> Pose {
        let jt_transform = Transform { rotation: UnitQuaternion::from_scaled_axis(self.axis.scale(q)),
                                       translation: Translation3::identity() };
        Pose {
            coord_frame: &self.parent_anchor.coord_frame,
            pose: jt_transform*child.pose
        }
    }
    
    pub fn from_urdf(joint: &Joint, parent: &'b Transform) -> RevoluteJoint<'b> {
        let axis = UnitVector3::<f32>::new_normalize(convert(joint.axis));
        let parent_anchor = Pose{
            coord_frame: parent,
            pose: Transform::new(Vector3::zeros(), Vector3::z())
        };
        RevoluteJoint { axis: axis,
                        parent_anchor:parent_anchor,
                        child_frame:Isometry3::identity() }
    }
}
