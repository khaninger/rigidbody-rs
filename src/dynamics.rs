pub mod dynamics{
    use std::iter::zip;
    use std::ops::{Deref, DerefMut};
    use std::path::Path;
    use nalgebra::{convert, Const, OPoint, UnitVector3};
    use rapier3d::prelude::*;
    use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};

    use xurdf::{Link, Joint};
    use parry3d::mass_properties::MassProperties;
    
    use crate::kinematics::kinematics::EEKinematicModel;


    fn xurdf_to_massproperties(link: &Link) -> MassProperties {
        let local_com:OPoint<f32, Const<3>> = OPoint::<f32, Const<3>>::new(
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

    fn xurdf_joint_to_revolute(joint: &Joint) -> RevoluteJoint {
        let axis = UnitVector3::<f32>::new_normalize(convert(joint.axis));
        let jt = RevoluteJointBuilder::new(axis)
            .local_anchor1(point![
                joint.origin.xyz[0] as f32,
                joint.origin.xyz[1] as f32,
                joint.origin.xyz[2] as f32,
            ]).build();
        jt
    }
    
    pub struct MultibodyLink {
        rigid_body: MassProperties,
        joint: RevoluteJoint, //MultibodyJoint,
    }
    
       // We re-implement b/c too many attributes are private
    pub struct MultibodyLinkVec(pub Vec<MultibodyLink>);

    impl Deref for MultibodyLinkVec {
        type Target = Vec<MultibodyLink>;

        #[inline]
        fn deref(&self) -> &Vec<MultibodyLink> {
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
        
        pub fn from_urdf(urdf_path: &Path, ee_name: &str) -> MultibodyLinkVec {
            let urdf_options = UrdfLoaderOptions {
                create_colliders_from_collision_shapes: false,
                create_colliders_from_visual_shapes: false,
                apply_imported_mass_props: true,
                make_roots_fixed: true,
                ..Default::default()
            };
            let result = UrdfRobot::from_file(urdf_path, urdf_options, None);
            let (mut robot, xrobot) = match result {
                Ok(val) => val,
                Err(err) => panic!("Error in URDF loading {}", err),
            };

            
            let mut bodies = RigidBodySet::new();
            let mut multibody_joints = MultibodyJointSet::new();
            let mut colliders = ColliderSet::new();

            let handles = robot.insert_using_multibody_joints(&mut bodies, &mut colliders, &mut multibody_joints, UrdfMultibodyOptions::DISABLE_SELF_CONTACTS);            
            
            let ee_index_result = xrobot.links.iter().position(|el| el.name == ee_name);
            let ee_index = match ee_index_result {
                Some(val) => val,
                None => {
                    let avail_names:Vec<String> = xrobot.links.iter().map(|e| e.name.clone()).collect();
                    panic!("Error finding ee_link {}, available names are {}",
                           ee_name, avail_names.join(", "));
                }
            };
            
            let mut mb_link_vec = MultibodyLinkVec::new();
            for (joint, link) in zip(xrobot.joints.iter(), xrobot.links.iter()) {
                mb_link_vec.push( MultibodyLink {
                    rigid_body: xurdf_to_massproperties(link),
                    joint: xurdf_joint_to_revolute(joint)
                });
            }
            
            mb_link_vec          
            
        }

        pub fn inertia_matrix(&self, q: &[f32]) -> DMatrix<Real> {
            // Needs body_jacobians and rigid_bodies
            let mut augmented_mass = DMatrix::zeros(7, 7);
            let mut i = 0;
            for link in self.iter() {
                augmented_mass[i] = link.rigid_body.inv_mass;
                i=i+1;
            }
            augmented_mass
        }
    }
        
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

    #[cfg(test)]
    mod tests {
        use super::*;
        
        #[test]
        fn inertia_matrix(){
            let rob = MultibodyLinkVec::from_urdf(Path::new("assets/fr3.urdf"), "fr3_link8");
            println!("inertia mat: {:?}", rob.inertia_matrix(&[0.0; 7]));
        }
    }
}
