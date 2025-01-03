pub mod kinematics{
    use std::path::Path;
    use rapier3d::prelude::*;
    use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};
    
    pub struct EEKinematicModel {
        bodies: RigidBodySet,
        multibody_joints: MultibodyJointSet,
        ee_link_handle: RigidBodyHandle,
        ee_joint_handle: MultibodyJointHandle
    }
    
    impl EEKinematicModel {
        pub fn from_urdf(urdf_path: &Path, ee_name: &str) -> EEKinematicModel {
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
                
            let ee_link_handle = handles.links[ee_index].body;
            let (_, _, ee_joint_handle) = multibody_joints.attached_joints(ee_link_handle).last().unwrap();

            let (mut multibody, ee_link_id) = multibody_joints.get_mut(ee_joint_handle).unwrap();
            multibody.forward_kinematics(&mut bodies, false); // Needed so that the extra DOF in root are removed

            EEKinematicModel {
                bodies: bodies,
                multibody_joints: multibody_joints,
                ee_link_handle: ee_link_handle,
                ee_joint_handle: ee_joint_handle,
            }
        }
        
        pub fn fwd_kin(&mut self, joint_angles: &[f32]) -> Isometry<f32> {
            let (mut multibody, link_id) = self.multibody_joints.get_mut(self.ee_joint_handle).unwrap();
            
            multibody.apply_displacements(joint_angles);
            multibody.forward_kinematics(&self.bodies, false);
            multibody.update_rigid_bodies(&mut self.bodies, false);
            
            return *self.bodies.get(self.ee_link_handle)
                .expect("ee_link not found in bodies.")
                .position()
        }
        
        pub fn inv_kin(&mut self, ee_pose: Isometry<f32>) -> DVector<f32> {
            let (mut multibody, link_id) = self.multibody_joints.get_mut(self.ee_joint_handle).unwrap();

            let mut displacements = DVector::zeros(0);
            displacements = DVector::zeros(multibody.ndofs());
            
            let options = InverseKinematicsOption {
                ..Default::default()
            };
            
            multibody.inverse_kinematics(
                &self.bodies,
                link_id,
                &options,
                &ee_pose,
                |lk| !lk.is_root(),
                &mut displacements
            );
            
            return displacements
        }    
        
    }
    

    #[cfg(test)]
    mod tests {
        use super::*;
        
        #[test]
        fn fwd_kin_test(){
            let mut bodies = RigidBodySet::new();
            let mut multibody_joints = MultibodyJointSet::new();

            let world = bodies.insert(RigidBodyBuilder::fixed());
            let rigid_body_1 = bodies.insert(RigidBodyBuilder::dynamic());    
            let rigid_body_2 = bodies.insert(RigidBodyBuilder::dynamic());

            let jt_world = RevoluteJointBuilder::new(Vector::x_axis())
                .local_anchor1(point![0., 0., 0.1])
                .local_anchor2(point![0., 0., 0.])
                .build();
            let jt_1 = RevoluteJointBuilder::new(Vector::x_axis())
                .local_anchor1(point![0., 0., 0.1])
                .local_anchor2(point![0., 0., 0.0])
                .build();

            multibody_joints.insert(world, rigid_body_1, jt_world, true);
            let multibody = multibody_joints.insert(rigid_body_1, rigid_body_2, jt_1, true).unwrap();

            let ee_pose = fwd_kin(&mut bodies,
                                  &mut multibody_joints,
                                  multibody,
                                  rigid_body_2,
                                  &vec![0.2,0.3,1.,1.,1.,1.,0.3,0.4]);
            println!("ee_pose: {:?}", ee_pose);

        }
    }

}
