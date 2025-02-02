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
use crate::rigidbody::Rigidbody;

/// An explicit revolute joint with a single degree of freedom
#[derive(Debug)]
pub struct RevoluteJoint  {
    pub axis: Unit<Vector3<Real>>, // Normed axis for the revolute joint
    pub parent: Transform, // Pose of joint coord system relative to parent
    pub body: Rigidbody,
}

impl RevoluteJoint {
    pub fn parent_to_child_mut(&self, q: Real, tr: &mut Transform) {
        //TODO These are just roughed in to get an idea of the time savings it might have
        //     But pretty sure these are not matching the operations yet
        tr.rotation = &self.joint_transform(q) * tr.rotation;
        //tr.append_rotation_mut(&self.joint_transform(q));
        tr.translation.vector =  self.parent.translation.vector;
        //*tr = self.parent*self.joint_transform(q)*(*tr);
    }

    /// Transformation from the child coordinate system to the parent, used for reverse iteration
    #[inline(always)]
    pub fn parent_to_child(&self, q: Real) -> Transform {
        self.parent*self.joint_transform(q)
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
        joint: &Joint, link: &Link, //  parent_frame: &'b Coord
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
        let com = OPoint::<Real, U3>::new(
            link.inertial.origin.xyz[0] as Real,
            link.inertial.origin.xyz[1] as Real,
            link.inertial.origin.xyz[2] as Real,
        );
        let mass:Real = convert(link.inertial.mass);
        let inertia_com:Matrix3<Real> = convert(link.inertial.inertia);
        let translate_cr = com.coords.cross_matrix();
        let inertia = inertia_com + mass*translate_cr*(translate_cr.transpose());
        
        let body = Rigidbody{mass, com, inertia};
        RevoluteJoint{axis, parent, body}
    }
}


#[test]
fn joint() {
    use crate::spatial::*;
    let jt1 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        },
        body: Default::default()
    };
    let jt2 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        },
        body: Default::default()
    };
    let mut child_pt = Transform{
        rotation: UnitQuaternion::identity(),
        translation: Translation3::new(0.,0.,0.)
    };

    let q1 = std::f32::consts::FRAC_PI_2;
    let q2 = 0.;//std::f32::consts::FRAC_PI_2;

    /*
    let world_to_ee = jt2.joint_transform(q2)*jt1.joint_transform(q1);
   
    let child_pt_in_jt1 = jt2.child_to_parent(q2)*child_pt;
    let child_pt_in_world = jt1.child_to_parent(q1)*child_pt_in_jt1;
    
    let eps = 0.00001;
    
    assert!(child_pt_in_jt1.translation.vector.relative_eq(
        &Vector3::new(-1.,0.,0.), eps, eps
    ));
    assert!(child_pt_in_world.translation.vector.relative_eq(
        &Vector3::new(0., 2., 0.), eps, eps
    ));
     */
    jt2.parent_to_child_mut(q1, &mut child_pt);
    println!("{:?}", child_pt);
    assert!(false);
    let v = SpatialVelocity{ lin:Vector3::new(1.,0.,0.), rot:Vector3::new(1.,0.,0.)};
    let child_to_parent = jt2.joint_transform(std::f32::consts::FRAC_PI_2);
    let parent_to_child = child_to_parent.inverse();
    //println!("Vel in parent frame: {:?} \n        child frame: {:?}", v, parent_to_child*&v);    
}
/*
#[test]
fn fwd_kin_from_ee() {
    let jts = Multibody::from_urdf(Path::new("assets/fr3.urdf"));
    
    let mut ee = Isometry3::<Real>::identity();
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(1.)*ee;
    }

    println!("EE Pose in world, ones {:?}", ee);

    ee = Isometry3::<Real>::identity();
    for (i, jt) in jts.iter().enumerate().rev() {
        println!{"Joint: {}, {:?}", i, jt.child_to_parent(0.)};
        ee = jt.child_to_parent(0.)*ee;
    }

    println!("EE Pose in world, zeros {:?}", ee);
    
    ee = Isometry3::<Real>::identity();
    for jt in jts.iter().rev() {
        ee = jt.child_to_parent(-1.)*ee;
    }

    println!("EE Pose in world, -ones {:?}", ee);
}

#[test]
fn fwd_kin_from_world() {
    // ee is now world frame in link coordinates, and thus invert at end to get ee in world
    let jts = parse_urdf(Path::new("assets/fr3.urdf"));
    
    let mut world = Isometry3::<Real>::identity();
    for jt in jts.iter() {
        world = jt.parent_to_child(1.)*world;
    }

    println!("WORLD Pose in world, world ones {:?}", world.inverse());

    world = Isometry3::<Real>::identity();
    for jt in jts.iter() {
        world = jt.parent_to_child(0.)*world;
    }

    println!("WORLD Pose in world, world zeros {:?}", world.inverse());
    
    world = Isometry3::<Real>::identity();
    for jt in jts.iter() {
        world = jt.parent_to_child(-1.)*world;
    }

    println!("WORLD Pose in world, world -ones {:?}", world.inverse());
}
*/
