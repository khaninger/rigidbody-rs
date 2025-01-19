use nalgebra::{
    UnitVector3,
    Isometry3,
    Vector3,
    Translation3,
    Rotation3,
    UnitQuaternion,
    Unit,
    OPoint,
    U3,
    convert
};
use xurdf::{Link, Joint};
use parry3d::mass_properties::MassProperties;

pub type Real = f32;
pub type Transform = Isometry3<Real>;

/// Types of coordinate systems which can be expressed
#[derive(Debug)]
pub enum Coord<'a> {
    WORLD, // Reference coordinate system
    FIXED(Transform), // A fixed coordinate system relative to world
    REL(RelativeTransform<'a>), // A coordinate system relative to another coordinate system
    //TODO Reasonable? Not sure exactly. Would allow to ref to RigidBody, could avoid need for a fake Child coord frame
    BODY(RigidBody) // A coordinate system attached to a body
}

/// A transform expressed in a coordinate system. 
#[derive(Debug)]
pub struct RelativeTransform<'a> {
    pub coord_frame: &'a Coord<'a>, // Use reference so the validity of the transform is checked by borrow checker
    pub pose: Transform
}

#[derive(Debug)]
pub struct RigidBody {
    pub mass_props: MassProperties, // expressed in local coordinates
    pub local_coord_frame: Transform, // are essentially arbitrary? But might need as reference       
}

/// An explicit revolute joint with a single degree of freedom
#[derive(Debug)]
pub struct RevoluteJoint<'a>  {
    pub axis: Unit<Vector3<Real>>, // Normed axis for the revolute joint
    pub parent: RelativeTransform<'a>, // Pose of joint coord system relative to parent
    // TODO: These two can be replaced with RigidBody?
    pub child: Transform, // Currently always identity, only here to get refs for RelativeTransform
    pub child_mass: MassProperties
}

impl <'b> RevoluteJoint<'b> {
    /// A pose expressed in the child coord system to a pose in the parent.
    pub fn child_to_parent(&self,
                           q: Real,
                           child: &RelativeTransform
    ) -> RelativeTransform {
        //assert!(*child.coord_frame == self.child, "Child argument is not expressed in correct coordinate system");
        RelativeTransform {
            coord_frame: &self.parent.coord_frame,
            pose: self.joint_transform(q)*child.pose
        }
    }

    /// Transform from child to parent
    pub fn joint_transform(&self, q: Real) -> Transform {
        let jt = Transform { rotation: UnitQuaternion::from_scaled_axis(self.axis.scale(q)),
                             translation: Translation3::identity() };
        self.parent.pose*jt
    }
    
    pub fn from_xurdf_joint(
        joint: &Joint, link: &Link, //  parent_frame: &'b Coord
    ) -> RevoluteJoint<'b> {
        let axis = UnitVector3::<Real>::new_normalize(convert(joint.axis));
        let parent = RelativeTransform{
            coord_frame: &Coord::WORLD, // parent_frame,
            pose: Transform::new(
                convert(joint.origin.xyz),
                Rotation3::<Real>::from_euler_angles(
                    joint.origin.rpy[0] as Real,
                    joint.origin.rpy[1] as Real,
                    joint.origin.rpy[2] as Real
                ).scaled_axis()
            )
        };
        let local_com = OPoint::<Real, U3>::new(
            link.inertial.origin.xyz[0] as Real,
            link.inertial.origin.xyz[1] as Real,
            link.inertial.origin.xyz[2] as Real,
        );
        let mass:Real = convert(link.inertial.mass);
        let inertia = convert(link.inertial.inertia); 
        let child_mass = MassProperties::with_inertia_matrix(local_com, mass, inertia);
        RevoluteJoint{axis, parent, child:Isometry3::identity(), child_mass}
    }
}

#[test]
fn joint() {
    use crate::spatial::*;
    let jt1 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: RelativeTransform{
            coord_frame: &Coord::WORLD,
            pose: Transform{
                rotation: UnitQuaternion::identity(),
                translation: Translation3::new(1.,0.,0.)
            }
        },
        child: Isometry3::identity(),
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        ),            
    };
    let jt2 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: RelativeTransform{
            coord_frame: &Coord::FIXED(jt1.child),
            pose: Transform{
                rotation: UnitQuaternion::identity(),
                translation: Translation3::new(1.,0.,0.)
            }
        },
        child: Isometry3::identity(),
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        )
    };
    
    let child_pt = RelativeTransform {
        coord_frame: &Coord::FIXED(jt2.child),
        pose: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        }
    };

    let q1 = std::f32::consts::PI;
    let q2 = std::f32::consts::FRAC_PI_2;
    let child_pt_in_jt1 = jt2.child_to_parent(q2, &child_pt);
    let child_pt_in_world = jt1.child_to_parent(q1, &child_pt_in_jt1);

    let eps = 0.00001;
    
    assert!(child_pt_in_jt1.pose.translation.vector.relative_eq(
        &Vector3::new(1.,1.,0.), eps, eps
    ));
    assert!(child_pt_in_world.pose.translation.vector.relative_eq(
        &Vector3::new(0., -1., 0.), eps, eps
    ));

    let v = SpatialVelocity{ lin:Vector3::new(1.,0.,0.), rot:Vector3::new(1.,0.,0.)};
    let child_to_parent = jt2.joint_transform(std::f32::consts::FRAC_PI_2);
    let parent_to_child = child_to_parent.inverse();
    println!("Vel in parent frame: {:?} \n        child frame: {:?}", v, parent_to_child*&v);
    
}
/*
#[test]
fn urdf() {   
    let robot = parse_urdf_from_file(Path::new("assets/fr3.urdf")).unwrap();

    let mut jts = Vec::<RevoluteJoint>::new();
    let mut prev_frame:Coord = Coord::WORLD;
    
    for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
        if !joint.joint_type.contains("fixed") {
            let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link, &prev_frame);
            //prev_frame = Coord::FIXED(rev_joint.child);
 
            jts.push(rev_joint);
        }
    }
    println!("Xurdf Joints: {:?}, Xurdf Links: {:?}, Proc Joints: {:?}", robot.joints.len(), robot.links.len(), jts.len());

    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(1., &ee);
    }

    println!("EE Pose in world, ones {:?}", ee.pose);
        
    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(0., &ee);
    }

    println!("EE Pose in world, zeros {:?}", ee.pose);
    

    let mut ee = RelativeTransform{coord_frame: &prev_frame, pose: Isometry3::<Real>::identity()};
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(-1., &ee);
    }

    println!("EE Pose in world, -ones {:?}", ee.pose);
}
*/
