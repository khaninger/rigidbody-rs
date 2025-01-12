use nalgebra::{UnitVector3, Isometry3, Vector3, Translation3, Rotation3, UnitQuaternion, Unit, convert};
use xurdf::{Link, Joint};

pub type Real = f32;
pub type Transform = Isometry3<Real>;
//pub type Coord = Option<&Transform>; // None is in worldd, 


/// Types of coordinate systems which can be expressed
#[derive(Debug)]
pub enum Coord<'a> {
    WORLD, // Reference coordinate system
    FIXED(Transform), // A fixed coordinate system relative to world
    REL(RelativeTransform<'a>) // A coordinate system relative to another coordinate system
}

/// A transform expressed in a coordinate system. 
#[derive(Debug)]
pub struct RelativeTransform<'a> {
    pub coord_frame: &'a Coord<'a>, // Use reference so the validity of the transform is checked by borrow checker
    pub pose: Transform
}

//impl <'a> RelativeTransform<'a>{
//    pub fn to_world(&self) -> Transform {
//        self.coord_frame.inv_mul(&self.pose)
//    }
//}

/// An explicit revolute joint with a single degree of freedom
pub struct RevoluteJoint<'a>  {
    pub axis: Unit<Vector3<Real>>, // Normed axis for the revolute joint
    pub parent: RelativeTransform<'a>, // Pose of joint coord system relative to parent
    pub child: Transform // TODO: get rid of? Itś not 
}

impl <'b> RevoluteJoint<'b> {
    /// A pose expressed in the child coord system to a pose in the parent.
    pub fn child_to_parent(&self,
                           q: Real,
                           child: &RelativeTransform
    ) -> RelativeTransform {
        //assert!(*child.coord_frame == self.child, "Child argument is not expressed in correct coordinate system");
        let jt_transform = Transform {
            rotation: UnitQuaternion::from_scaled_axis(self.axis.scale(q)),
            translation: Translation3::identity()
        };
        RelativeTransform {
            coord_frame: &self.parent.coord_frame,
            pose: self.parent.pose*jt_transform*child.pose
        }
    }
    
    pub fn from_xurdf_joint(joint: &Joint, parent_frame: &'b Coord) -> RevoluteJoint<'b> {
        let axis = UnitVector3::<Real>::new_normalize(convert(joint.axis));
        let parent = RelativeTransform{
            coord_frame: parent_frame,
            pose: Transform::new(
                convert(joint.origin.xyz),
                Rotation3::<Real>::from_euler_angles(
                    joint.origin.rpy[0] as Real,
                    joint.origin.rpy[1] as Real,
                    joint.origin.rpy[2] as Real
                ).scaled_axis()
            )
        };
        RevoluteJoint{axis, parent, child:Isometry3::identity()}
    }
}


