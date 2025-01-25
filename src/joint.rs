use std::path::Path;
use std::iter::{zip, Iterator};
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
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;
use itertools::izip;
use crate::spatial::{SpatialForce, SpatialVelocity, BodyJacobian};

pub type Real = f32;
pub type Transform = Isometry3<Real>;

const body_jac: BodyJacobian = BodyJacobian::revolute_z();

//pub struct Multibody(Vec<RevoluteJoint>);
pub struct Multibody([RevoluteJoint; 7]);


impl Multibody {
    pub fn from_urdf(path: &Path) -> Multibody {
        let robot = parse_urdf_from_file(path).unwrap();
        
        let mut jts = Vec::<RevoluteJoint>::new();
        
        for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
            if !joint.joint_type.contains("fixed") {
                let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link);
                jts.push(rev_joint);
            }
        }
        Multibody(jts.try_into().unwrap())
    }

    pub fn fwd_kin (&self, q: &[Real; 7]) -> Transform {        
        zip(self.0.iter(), q.iter()).rev().fold(
            Transform::identity(),
            | tr, (jt, qi)| { jt.child_to_parent(*qi)*tr }
        )
    }

    
    pub fn rnea(&self, q: &[Real], dq: &[Real], ddq: &[Real]) -> [Real; 7] {
        // Joint torques
        let mut tau = [0.; 7];

        // Vel, acc, force of current link, in local link coordinates
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity {
            lin: Vector3::<Real>::new(0., 0., -9.81),
            rot: Vector3::<Real>::zeros()
        };
        let mut f = [SpatialForce::new(); 7];


        for (i, jt) in self.0.iter().enumerate() {
            // Transform spatial vectors from parent to child link
            let parent_to_link = jt.child_to_parent(q[i]); //_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4

            // Velocity of the joint, expressed in child link coordinates
            let vel_joint = body_jac*dq[i];  // vJ, (3.33)

            v = parent_to_link*&v + vel_joint;
            a = parent_to_link*&a + body_jac*ddq[i]; // + cJ // + v.cross(vel_joint);
            let I = jt.child_mass.reconstruct_inertia_matrix();
            let c = jt.child_mass.local_com.coords;

            f[i].lin = a.lin*jt.child_mass.mass() - c.cross(&a.rot);
            f[i].rot = I*a.rot + c.cross(&a.lin)*jt.child_mass.mass();// + v.gcross_matrix()*I*v-Xj*f;

            //println!("jt {:?}\n   vel: {:?}\n   acc: {:?}\n force: {:?}", i+1, v, a, fi);
            //println!("   parent_to_link: {:?}", parent_to_link);
            //println!("mass: {:?} com: {:?}", jt.child_mass.mass(), jt.child_mass.local_com);
        }
        //TODO make a nice big map / collect to directly produce tau
        // tau = self.0.iter().enumerate().rev().map(
        for (i, jt) in self.0.iter().enumerate().rev() {
            tau[i] = body_jac*&f[i];
            if i > 0 {
                let f_tr = jt.parent_to_child(q[i])*&f[i];
                f[i-1] += f_tr;
            }
        };

        tau
        //println!("vel: {:?}", v);
        //println!("acc: {:?}", a);
        //println!("tau: {:?}", tau);
    }
    
    pub fn rnea_zip(&self, q: &[Real], dq: &[Real], ddq: &[Real]) -> [Real; 7] {
        // Vel, acc, force of current link, in local link coordinates
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity {
            lin: Vector3::<Real>::new(0., 0., -9.81),
            rot: Vector3::<Real>::zeros()
        };
        let mut f = [SpatialForce::new(); 7];

        let f_vec = Vec::<SpatialForce>::new();
        
        let mut f: [SpatialForce; 7] = izip!(self.0.iter(), q.iter(), dq.iter(), ddq.iter()).map(
            |(jt, &qi, &dqi, &ddqi)| {
                // Transform spatial vectors from parent to child link
                let parent_to_link = jt.child_to_parent(qi); //_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4

                // Velocity of the joint, expressed in child link coordinates
                let vel_joint = body_jac*dqi;  // vJ, (3.33)

                v = parent_to_link*&v + vel_joint;
                a = parent_to_link*&a + body_jac*ddqi; // + cJ // + v.cross(vel_joint);
                let I = jt.child_mass.reconstruct_inertia_matrix();
                let c = jt.child_mass.local_com.coords;

                SpatialForce {
                    lin: a.lin*jt.child_mass.mass() - c.cross(&a.rot),
                    rot: I*a.rot + c.cross(&a.lin)*jt.child_mass.mass()// + v.gcross_matrix()*I*v-Xj*f;
                }
            }
        ).collect::<Vec<_>>().try_into().expect("Direclty build forces");

        // tau = self.0.iter().enumerate().rev().map(
        let mut f_last = SpatialForce::new();

        izip!(self.0.iter(), q.iter(), f.iter()).rev().map(
            |(jt, &qi, &fi)| {
                let taui = body_jac*&(fi+&f_last);
                f_last = jt.parent_to_child(qi)*&fi;
                taui
            }
        ).collect::<Vec<_>>().try_into().expect("Directly build torques")
    }

}

/// An explicit revolute joint with a single degree of freedom
#[derive(Debug)]
pub struct RevoluteJoint  {
    pub axis: Unit<Vector3<Real>>, // Normed axis for the revolute joint
    pub parent: Transform, // Pose of joint coord system relative to parent
    pub child_mass: MassProperties
}

impl RevoluteJoint {
    /// Transformation from the child coordinate system to the parent, used for reverse iteration
    pub fn child_to_parent(&self, q: Real) -> Transform {
        //assert!(*child.coord_frame == self.child, "Child argument is not expressed in correct coordinate system")
        self.parent*self.joint_transform(q)
    }

    /// Transformation from parent coordinate system to the child
    pub fn parent_to_child(&self, q: Real) -> Transform {
        self.joint_transform(-q)*(self.parent.inverse())
    }

    /// Transformation of the joint itself
    pub fn joint_transform(&self, q: Real) -> Transform {
        let jt_rot = UnitQuaternion::from_scaled_axis(self.axis.scale(q));
        Transform{translation: Translation3::identity(), rotation: jt_rot}
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
        let local_com = OPoint::<Real, U3>::new(
            link.inertial.origin.xyz[0] as Real,
            link.inertial.origin.xyz[1] as Real,
            link.inertial.origin.xyz[2] as Real,
        );
        let mass:Real = convert(link.inertial.mass);
        let inertia = convert(link.inertial.inertia); 
        let child_mass = MassProperties::with_inertia_matrix(local_com, mass, inertia);
        RevoluteJoint{axis, parent, child_mass}
    }
}


//#[test]
fn joint() {
    use crate::spatial::*;
    let jt1 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        },
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        ),            
    };
    let jt2 = RevoluteJoint{
        axis: Unit::new_normalize(Vector3::z()),
        parent: Transform{
            rotation: UnitQuaternion::identity(),
            translation: Translation3::new(1.,0.,0.)
        },
        child_mass: MassProperties::new(
            OPoint::<f32, U3>::new(0., 0., 0.),
            0.5,
            Vector3::<f32>::new(1.,1.,1.)
        )
    };
    let child_pt = Transform{
        rotation: UnitQuaternion::identity(),
        translation: Translation3::new(0.,0.,0.)
    };

    let q1 = std::f32::consts::FRAC_PI_2;
    let q2 = 0.;//std::f32::consts::FRAC_PI_2;

    let world_to_ee = jt2.joint_transform(q2)*jt1.joint_transform(q1);
    println!("jt1 trans {:?} jt2 trans {:?}", jt1.joint_transform(q1), jt2.joint_transform(q2));
    println!("world_to_ee {:?}", world_to_ee);
    let child_pt_in_jt1 = jt2.child_to_parent(q2)*child_pt;
    let child_pt_in_world = jt1.child_to_parent(q1)*child_pt_in_jt1;
    println!("ee in jt1 {:?}, ee in world {:?}", child_pt_in_jt1, child_pt_in_world);
    let eps = 0.00001;
    
    assert!(child_pt_in_jt1.translation.vector.relative_eq(
        &Vector3::new(1.,1.,0.), eps, eps
    ));
    assert!(child_pt_in_world.translation.vector.relative_eq(
        &Vector3::new(0., -1., 0.), eps, eps
    ));

    let v = SpatialVelocity{ lin:Vector3::new(1.,0.,0.), rot:Vector3::new(1.,0.,0.)};
    let child_to_parent = jt2.joint_transform(std::f32::consts::FRAC_PI_2);
    let parent_to_child = child_to_parent.inverse();
    println!("Vel in parent frame: {:?} \n        child frame: {:?}", v, parent_to_child*&v);    
}

/*#[test]
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
