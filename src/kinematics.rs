pub mod kinematics{

    use rapier3d::prelude::*;

    pub fn fwd_kin(bodies: &mut RigidBodySet,
                   joints: &mut MultibodyJointSet,
                   ee_link: MultibodyJointHandle,
                   joint_angles: &[f32]
    ) -> Isometry<f32> {
        if let Some((multibody, link_id)) = joints.get_mut(ee_link) {
            println!("multibody num links {:?} num dof {:?}",
                     multibody.num_links(),
                     multibody.ndofs()
            );
            multibody.apply_displacements(joint_angles);
            multibody.forward_kinematics(&bodies,false);
            multibody.update_rigid_bodies(bodies,false);
        }

        for (handle, body) in bodies.iter() {
            println!("body {:?}", body.position());
        }
        
        Isometry::identity()
    }

    /*pub fn inverse_kinematics(joints: &MultibodyJointSet,
                              bodies: &RigidBodySet,
                              ee_link: &MultibodyLink,
                              joint_angles: &[f32]
    ) -> &[f32] {             
        if let Some((multibody, link_id)) = joints.get_mut(ee_link) {
            displacements = DVector::zeros(multibody.ndofs());

            let options = InverseKinematicsOption::new();
            multibody.inverse_kinematics(
                &bodies,
                ee_link,
                &options,
                &Isometry::identity(),
                |_| true,
                &mut displacements
            );
        }
    }*/    
}
