//multibody.rs
use std::path::Path;
use std::ops::Index;
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
    Const,
    SMatrix
};
use xurdf::{Robot, Link, Joint, parse_urdf_from_file};
use parry3d::mass_properties::MassProperties;
use itertools::izip;
use crate::spatial::{SpatialForce, SpatialVelocity, BodyJacobian};
use crate::joint::*;
use crate::inertia::Inertia;
use crate::{Real, Transform};

const body_jac: BodyJacobian = BodyJacobian::revolute_z();

#[derive(Debug)]
pub struct Multibody([RevoluteJoint; 7]);

#[derive(Debug)]
pub struct MBTransforms<'a> {
    tr: [Transform; 7],
    _phantom: std::marker::PhantomData<&'a Real>
}

impl <'a> MBTransforms<'a> {
    pub fn from_joint_angles<'b>(mb: &Multibody, q: &'a [Real]) -> MBTransforms<'a> {
        let tr: [Transform;7]  = zip(mb.iter(), q.iter())
            .map(|(jt, &qi)| {jt.parent_to_child(qi)})
            .collect::<Vec<_>>()
            .try_into()
            .expect("Direclty build transforms");

        MBTransforms{ tr, _phantom: std::marker::PhantomData }        
    }

    pub fn iter(&self) -> impl Iterator<Item=&Transform> {
        self.tr.iter()
    }
}

impl Index<usize> for MBTransforms<'_> {
    type Output = Transform;

    fn index(&self, idx: usize) -> &Self::Output {
        &self.tr[idx]
    }
}

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
    
    pub fn iter(&self) -> impl Iterator<Item=&RevoluteJoint> {
        self.0.iter()
    }            

    pub fn get_transforms<'a>(&self, q: &'a [Real]) -> MBTransforms<'a> {
        MBTransforms::from_joint_angles(&self, q)
    }
    
    pub fn fwd_kin (&self, mb_tr: &MBTransforms) -> Transform {        
        let mut tr = Transform::identity();
        for t in mb_tr.iter() {
            tr = t*tr;
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
            jt_transforms[i] = jt.parent_to_child(q[i]); //_J*X_T(i), X_J: Table 4.1, X_T(i): Ch. 4
          
            // Velocity of the joint, expressed in child link coordinates
            let vel_joint = SpatialVelocity::z_rot(dq[i]); // vJ, (3.33)

            // Update velocity/acc of current link            
            v = jt_transforms[i]*&v;
            v.rot[2] += dq[i];
               
            a = jt_transforms[i]*&a;
            a.rot[2] += ddq[i];

            a.lin[0] += v.lin[1]*dq[i];  // cross product unrolled
            a.lin[1] += -v.lin[0]*dq[i];
            a.rot[0] += v.rot[1]*dq[i];
            a.rot[1] += -v.rot[0]*dq[i];
            
            f[i] = &jt.body*&a + &v.cross_star(&(&jt.body*&v));
        }
        
        for (i, jt) in self.0.iter().enumerate().rev() {
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
    
    pub fn crba(&self, q: &[Real]) -> SMatrix::<Real, 7, 7> {
        let mut H = Matrix::<Real, Const<7>, Const<7>, _>::identity();
        let mut I: [Inertia; 7] = self.0.iter()
            .map(|jt| {jt.body.clone()})
            .collect::<Vec<_>>()
            .try_into()
            .expect("Directly build inertias");
        for i in (0..7).rev() {
            let jt_transform = self.0[i].parent_to_child(q[i]);
            if i > 0 {
                I[i-1] = I[i-1].clone() + I[i].transform(jt_transform);
            }
            H[(i,i)] = I[i].get_rotz();
            let mut F = &I[i]*&body_jac;
            for j in (0..i).rev() {
                F = self.0[j+1].child_to_parent(q[j+1])*&F;                
                H[(j,i)] = F.rot[2].clone();
            }
        }
        H
    }
}


#[cfg(test)]
mod test{
    use super::*;
    extern crate test;
    use test::Bencher;

    #[test]
    fn test_transforms() {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let mut tr;
        {
            let q = [0.; 7];
            tr = MBTransforms::from_joint_angles(&mb, &q);
            println!("{:?}", tr);
        }
        //println!("{:?}", tr);
    }
    
    #[bench]
    fn bench_fwd_kin(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let mb_tr = mb.get_transforms(&[0.; 7]);
        b.iter(|| { mb.fwd_kin(&mb_tr); })        
    }

    #[bench]
    fn bench_transform_allocate(b: &mut Bencher) {
        b.iter(|| { Transform::identity() })        
    }
    
    #[bench]
    fn bench_rnea(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));    
        let mb_tr = mb.get_transforms(&[0.; 7]);
        b.iter(|| { mb.rnea(&[0.; 7], &[0.;7], &[0.;7]); })        
    }

    #[bench]
    fn bench_rneazip(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));    
        b.iter(|| { mb.rnea_zip(&[0.;7], &[0.;7], &[0.;7]); })        
    }

    #[test]
    fn test_crba() {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let H = mb.crba(&[1.; 7]);
        //println!("crba: {}", H);
    }
}
