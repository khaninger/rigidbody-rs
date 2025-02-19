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
    
    pub fn rnea(&self, mb_tr: &MBTransforms, dq: &[Real], ddq: &[Real]) -> [Real; 7] {
        let mut tau = [0.; 7]; // joint torques
        let mut f = [SpatialForce::new(); 7];  // spatial forces on each link 

        // Vel, acc, force of current link, in local link coordinates
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity {
            lin: Vector3::<Real>::new(0., 0., 9.81),
            rot: Vector3::<Real>::zeros()
        };

        for (i, jt) in self.iter().enumerate() {
            let jt_transform = mb_tr[i];
          
            // Velocity of the joint, expressed in child link coordinates
            let vel_joint = SpatialVelocity::z_rot(dq[i]); // vJ, (3.33)

            // Update velocity/acc of current link            
            v = jt_transform*&v;
            v.rot[2] += dq[i];
               
            a = jt_transform*&a;
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
                let f_tr = mb_tr[i].inverse()*&f[i];
                f[i-1] += f_tr;
            }
        };

        tau
    }
         
    pub fn crba(&self, mb_tr: &MBTransforms) -> SMatrix::<Real, 7, 7> {
        let mut H = Matrix::<Real, Const<7>, Const<7>, _>::identity();
        let mut I: [Inertia; 7];
        let mut I = self.0.last().unwrap().body.clone();
        for i in (0..7).rev() {
            let jt_transform = mb_tr[i];
            
            H[(i,i)] = I.get_rotz();
            let mut F = &I*&body_jac;
            for j in (0..i).rev() {
                F = mb_tr[j+1]*&F;                
                H[(j,i)] = F.rot[2].clone();
            }
            
            if i > 0 {
                I = self.0[i-1].body.clone() + I.transform(jt_transform);
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
        //println!("{:?}", tr); // should error
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
        b.iter(|| { mb.rnea(&mb_tr, &[0.;7], &[0.;7]); })        
    }

    #[bench]
    fn test_crba(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("../assets/fr3.urdf"));
        let mb_tr = mb.get_transforms(&[0.; 7]);
        b.iter(|| { mb.crba(&mb_tr); })
        //println!("crba: {}", H);
    }
}
