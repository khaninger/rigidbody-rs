pub mod kinematics{

  use rapier3d::prelude::*;



  pub struct Multibody {
      bodies: RigidBodySet,
      joints: MultibodyJointSet       
  }

  pub fn fwd_kin(bodies: &RigidBodySet,
                 joints: &MultibodyJointSet,
                 joint_angles: &[f32]
  ) -> (Isometry<f32>) {     
      for (joint_handle, angle) in joints.iter().zip(joint_angles.iter()) {
          println!("anglearino {:?}", angle);
      }

      Isometry::identity()
  }
    
}
