pub mod kinematics{

    use rapier3d::prelude::*;
    
    pub fn fwd_kin(bodies: &mut RigidBodySet,
                   joints: &mut MultibodyJointSet,
                   multibody_handle: MultibodyJointHandle,
                   ee_link: RigidBodyHandle,
                   joint_angles: &[f32]
    ) -> Isometry<f32> {
        if let Some((multibody, link_id)) = joints.get_mut(multibody_handle) {
            //println!("multibody num links {:?} num dof {:?}",
            //         multibody.num_links(),
            //         multibody.ndofs()
            //);
            //println!("0 {:?} 1 {:?}  2 {:?} ",
            //         bodies.get(multibody.link(0).unwrap().rigid_body_handle()).unwrap().is_rotation_locked(),
            //         bodies.get(multibody.link(1).unwrap().rigid_body_handle()).unwrap().is_rotation_locked(),
            //         bodies.get(multibody.link(2).unwrap().rigid_body_handle()).unwrap().is_rotation_locked()
            //);
            multibody.apply_displacements(joint_angles);
            multibody.forward_kinematics(&bodies,false);
            multibody.update_rigid_bodies(bodies,false);
        }

        return *bodies.get(ee_link)
            .expect("ee_link not found in bodies.")
            .position()
    }
    
    pub fn inv_kin(bodies: &mut RigidBodySet,
                   joints: &mut MultibodyJointSet,
                   multibody_handle: MultibodyJointHandle,
                   ee_pose: Isometry<f32>
    ) -> DVector<f32> {
        let mut displacements = DVector::zeros(0);
        if let Some((multibody, link_id)) = joints.get_mut(multibody_handle) {
            displacements = DVector::zeros(multibody.ndofs());

            let options = InverseKinematicsOption {
                ..Default::default()
            };
            
            multibody.inverse_kinematics(
                &bodies,
                link_id,
                &options,
                &ee_pose,
                |lk| !lk.is_root(),
                &mut displacements
            );
        }
        return displacements
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
