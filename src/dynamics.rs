use std::iter::zip;
use std::ops::{Deref, DerefMut};
use std::path::Path;
use nalgebra::{convert, OPoint, UnitVector3, Vector3, U3};

use xurdf::{Link, Joint};
use parry3d::mass_properties::MassProperties;

use crate::{SpatialVelocity, BodyJacobian, joint::RevoluteJoint};

fn xurdf_to_massproperties(link: &Link) -> MassProperties {
   let local_com:OPoint<f32, U3> = OPoint::<f32, U3>::new(
       link.inertial.origin.xyz[0] as f32,
       link.inertial.origin.xyz[1] as f32,
       link.inertial.origin.xyz[2] as f32
   );
   MassProperties::with_inertia_matrix(
       local_com,
       link.inertial.mass as f32,
       convert(link.inertial.inertia)
   )
}

/*
// Example of custom typedef
pub struct MultibodyLinkVec(pub Vec<Rigidbody>);
impl Deref for MultibodyLinkVec {
    type Target = Vec<Rigidbody>;

    #[inline]
    fn deref(&self) -> &Vec<Rigidbody> {
        let MultibodyLinkVec(ref me) = *self;
        me
    }
}
impl DerefMut for MultibodyLinkVec {
    #[inline]
    fn deref_mut(&mut self) -> &mut Vec<MultibodyLink> {
        let MultibodyLinkVec(ref mut me) = *self;
        me
    }
}
impl MultibodyLinkVec {
    pub fn new() -> Self {
        return MultibodyLinkVec(Vec::new());
    }
    
    pub fn rnea(&self, q: &[f32],dq: &[f32], ddq: &[f32]) -> SpatialVelocity {
        // Table 5.1
        let mut v = SpatialVelocity::new();
        let mut a = SpatialVelocity::new();
        let mut f = 0;

        let mut tr_link_to_world = Isometry::identity();

        // Relevant functions:
        // multibody_joint::jacobian(&self, transform: &Rotation<Real>, out: &mut JacobianViewMut<real>)
        // multibody_joint::jacobian_mul_coordinates(&self, acc: &[Real]) -> RigidBOdyVelocity
        // Vector3::cross, Vector3::gcross_matrix_tr
        // Matrix::gemm
        // Isometry::inv and ::inv_mul
        for (i, link) in self.iter().enumerate() {

            // Local joint transformation and velocity
            let tr_joint = Isometry::new(Vector3::default(),
                                         Vector3::z()*q[i]);      // XJ, Table 4.1
            let body_jac = BodyJacobian::revolute_z(); // S, assume all jts are revolute about z
            let vel_joint = body_jac*dq[i];  // vJ, (3.33)
            let cJ = SpatialVelocity::new(); // (3.42 & 3.43, no rate of change 

            let tr_child_to_parent = tr_joint; //TODO
            let tr_child_to_world = tr_child_to_parent*tr_link_to_world;

            v = tr_child_to_parent*v + vel_joint; 
            let body_jac = BodyJacobian::revolute_z(); // S, assume all jts are revolute about z
            a = tr_child_to_parent*a + body_jac*ddq[i] + cJ; // + v.cross(vel_joint);
            //let I = link.rigid_body.reconstruct_inertia_matrix();
            //f[i] = I*a + v.gcross_matrix()*I*v-Xj*f;
        }
        for (i, link) in self.iter().rev().enumerate() {
            //tau[i] = Si.transpose()*f;
            //f = f[i] + Xj.transpose()*f
        }

        v
    }

    pub fn inertia_matrix(&self, q: &[f32]) -> DMatrix<Real> {
        let mut augmented_mass = DMatrix::zeros(7, 7);
        let mut i = 0;
        for link in self.iter() {
            augmented_mass[i] = link.rigid_body.inv_mass;
            i=i+1;
        }
        augmented_mass
    }
}
*/
/*
pub fn inv_dyn(&mut self,
               q: &[f32],
               qd: &[f32],
               qdd: &[f32]
) -> &[f32] {
    let (mut multibody, link_id) = self.multibody_joints.get_mut(self.ee_joint_handle).unwrap();
    let inertia_matrix = multibody.inv_augmented_mass();
    let inertial_forces = inertia_matrix.multiply(qdd);

    let rb = self.bodies.index_mut_internal(self.ee_link_handle);
}*/

#[cfg(test)] //This flag only adds this code when compiling for tests
mod tests {
    use super::*;

    #[test]
    fn inertia_matrix(){
        let rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
        println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
    }

    #[test]
    fn rnea() {
        let mut rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
        //println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
        let res = rob.rnea(&[0.1; 9], &[0.2; 9], &[0.3; 9]);
        println!("rnea: {:?}", res);
    }
    
}
