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
use crate::joint::*;

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
            let I = jt.child_inertia;
            let m = jt.child_mass_prop.mass();
            let c = jt.child_mass_prop.local_com.coords;

            f[i].lin = a.lin*jt.child_mass_prop.mass() - c.cross(&a.rot);
            f[i].rot = I*a.rot + c.cross(&a.lin)*m;// + v.gcross_matrix()*I*v-Xj*f;

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
                let I = jt.child_mass_prop.reconstruct_inertia_matrix();
                let c = jt.child_mass_prop.local_com.coords;

                SpatialForce {
                    lin: a.lin*jt.child_mass_prop.mass() - c.cross(&a.rot),
                    rot: I*a.rot + c.cross(&a.lin)*jt.child_mass_prop.mass()// + v.gcross_matrix()*I*v-Xj*f;
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



#[cfg(test)]
mod test{
    use super::*;
    extern crate test;
    use test::Bencher;

    #[bench]
    fn bench_fwd_kin(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));    
        b.iter(|| { mb.fwd_kin(&[0.;7]); })        
    }

    #[bench]
    fn bench_rnea(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));    
        b.iter(|| { mb.rnea(&[0.;7], &[0.;7], &[0.;7]); })        
    }

    #[bench]
    fn bench_rneazip(b: &mut Bencher) {
        let mb = Multibody::from_urdf(&Path::new("assets/fr3.urdf"));    
        b.iter(|| { mb.rnea_zip(&[0.;7], &[0.;7], &[0.;7]); })        
    }
}
