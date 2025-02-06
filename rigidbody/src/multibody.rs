//multibody.rs
use std::path::Path;
use std::iter::{zip, Iterator};
use nalgebra::{
    UnitVector3,
    Isometry3,
    Vector3,
    Translation3,
    Rotation3,
    Matrix3,
    Matrix,
    UnitQuaternion,
    Unit,
    OPoint,
    U3,
    convert,
    Const        
};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;
use itertools::izip;
use crate::spatial::{SpatialForce, SpatialVelocity, BodyJacobian};
use crate::joint::*;
use crate::rigidbody::RigidbodyInertia;
use crate::{Real, Transform};

const body_jac: BodyJacobian = BodyJacobian::revolute_z();

//pub struct Multibody(Vec<RevoluteJoint>);
#[derive(Debug)]
pub struct Multibody([RevoluteJoint; 7]);


impl Multibody {
    pub fn from_urdf(path: &Path) -> Multibody {
        let robot = parse_urdf_from_file(path).expect("Error loading URDF");
        
        let mut jts = Vec::<RevoluteJoint>::new();
        
        for (joint, link) in zip(robot.joints.iter(), robot.links.iter()) {
            if !joint.joint_type.contains("fixed") {
                let rev_joint = RevoluteJoint::from_xurdf_joint(joint, link);
                jts.push(rev_joint);
            }
        }
        Multibody(jts.try_into().unwrap())
    }

    pub fn fwd_kin (&self, q: &[Real]) -> Transform {        
        let mut tr = Transform::identity();
        for (jt, qi) in zip(self.0.iter(), q.iter()).rev() {
            tr = jt.parent_to_child(*qi)*tr;
        }
        tr            
    }
    
    pub fn rnea(&self, q: &[Real], dq: &[Real], ddq: &[Real]) -> [Real; 7] {
        let mut tau = [0.; 7]; // joint torques
        let mut f = [SpatialForce::new(); 7];  // spatial forces on each link 
        let mut jt_transforms = [Transform::identity(); 7]; // transform btwn links

        // Vel, acc, force of current link, in local link coordinates
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity {
            lin: Vector3::<Real>::new(0., 0., 9.81),
            rot: Vector3::<Real>::zeros()
        };

        for (i, jt) in self.0.iter().enumerate() {
            // Transform spatial vectors from parent to child link
            jt_transforms[i] = jt.parent_to_child(q[i]); //_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4
          
            // Velocity of the joint, expressed in child link coordinates
            //let vel_joint = body_jac*dq[i];  // vJ, (3.33)
            let vel_joint = SpatialVelocity::z_rot(dq[i]); // vJ, (3.33)

            // Update velocity/acc of current link            
            //v = jt_transforms[i]*&v + &vel_joint;
            //a = jt_transforms[i]*&a + &(body_jac*ddq[i]) + &(v.cross(&vel_joint));
            v = jt_transforms[i]*&v;
            v.rot[2] += dq[i];
               
            a = jt_transforms[i]*&a; //+ &(v.cross(&vel_joint));
            a.rot[2] += ddq[i]; // jt acc

            a.lin[0] += v.lin[1]*dq[i];  // cross product unrolled
            a.lin[1] += -v.lin[0]*dq[i];
            a.rot[0] += v.rot[1]*dq[i];
            a.rot[1] += -v.rot[0]*dq[i];
            
            f[i] = &jt.body*&a + &v.cross_star(&(&jt.body*&v));
        }
        
        for (i, jt) in self.0.iter().enumerate().rev() {
            //tau[i] = body_jac*&f[i];
            tau[i] = f[i].rot[2];
            if i > 0 {
                let f_tr = jt_transforms[i].inverse()*&f[i];
                f[i-1] += f_tr;
            }
        };

        tau
    }
    
    pub fn rnea_zip(&self, q: &[Real], dq: &[Real], ddq: &[Real]) -> [Real; 7] {
        // Vel, acc, force of current link, in local link coordinates
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity {
            lin: Vector3::<Real>::new(0., 0., -9.81),
            rot: Vector3::<Real>::zeros()
        };
        
        let mut f: [SpatialForce; 7] = izip!(self.0.iter(), q.iter(), dq.iter(), ddq.iter()).map(
            |(jt, &qi, &dqi, &ddqi)| {
                let jt_transform = jt.parent_to_child(qi); //_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4

                // Velocity of the joint, expressed in child link coordinates
                //let vel_joint = body_jac*dq[i];  // vJ, (3.33)
                let vel_joint = SpatialVelocity::z_rot(dqi); // vJ, (3.33)

                // Update velocity/acc of current link            
                //v = jt_transforms[i]*&v + &vel_joint;
                //a = jt_transforms[i]*&a + &(body_jac*ddq[i]) + &(v.cross(&vel_joint));
                v = jt_transform*&v;
                v.rot[2] += dqi;

                a = jt_transform*&a; //+ &(v.cross(&vel_joint));
                a.rot[2] += ddqi; // jt acc

                a.lin[0] += v.lin[1]*dqi;  // cross product unrolled
                a.lin[1] += -v.lin[0]*dqi;
                a.rot[0] += v.rot[1]*dqi;
                a.rot[1] += -v.rot[0]*dqi;

                &jt.body*&a + &v.cross_star(&(&jt.body*&v))
            }
        ).collect::<Vec<_>>().try_into().expect("Direclty build forces");

        let mut f_last = SpatialForce::new();
        izip!(self.0.iter(), q.iter(), f.iter()).rev().map(
            |(jt, &qi, &fi)| {
                let taui = body_jac*&(fi+&f_last);
                f_last = jt.child_to_parent(qi)*&fi;
                taui
            }
        ).collect::<Vec<_>>().try_into().expect("Directly build torques")
    }

    pub fn crba(&self, q: &[Real]) -> &[Real] {
        let H = Matrix::<Real, Const<7>, Const<7>, _>::identity();
        let I: [&RigidbodyInertia; 7] = self.0.iter()
            .map(|jt| {&jt.body})
            .collect::<Vec<_>>()
            .try_into()
            .expect("Directly build inertias");
        for i in 7..0 {
            let jt_transform = self.0[i].parent_to_child(q[i]);
            if i > 0 {
                //I[i-1] += 
            }
        }
        &[0.;7]
    }
}



#[cfg(test)]
mod test{
    use super::*;
    extern crate test;
    use test::Bencher;

    #[bench]
    fn bench_fwd_kin(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));    
        b.iter(|| { mb.fwd_kin(&[0.;7]); })        
    }

    #[bench]
    fn bench_transform_allocate(b: &mut Bencher) {
        b.iter(|| { Transform::identity() })        
    }
    
    #[bench]
    fn bench_rnea(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));    
        b.iter(|| { mb.rnea(&[0.;7], &[0.;7], &[0.;7]); })        
    }

    #[bench]
    fn bench_rneazip(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));    
        b.iter(|| { mb.rnea_zip(&[0.;7], &[0.;7], &[0.;7]); })        
    }
}
