use rapier3d::prelude::*;

mod kinematics;
use kinematics::kinematics::{fwd_kin, inv_kin};

fn main() {
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    
    let world = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.,0.,0.]));
    let rigid_body_1 = bodies.insert(RigidBodyBuilder::dynamic().can_sleep(false));    
    let rigid_body_2 = bodies.insert(RigidBodyBuilder::dynamic().can_sleep(false));

    //let mut colliders = ColliderSet::new();
    //let collider = ColliderBuilder::cuboid(0.2, 0.1, 0.2);
    //colliders.insert_with_parent(collider, world, &mut bodies);
    
    let jt_world = RevoluteJointBuilder::new(Vector::x_axis())
        .local_anchor1(point![0., 0., 0.1])
        .local_anchor2(point![0., 0., 0.])
        .build();
    let jt_1 = RevoluteJointBuilder::new(Vector::x_axis())
        .local_anchor1(point![0., 0., 0.1])
        .local_anchor2(point![0., 0., 0.0])
        .build();

    multibody_joints.insert(world, rigid_body_1, jt_world, true);
    let multibody_handle = multibody_joints.insert(rigid_body_1, rigid_body_2, jt_1, true).unwrap();

    println!("multibody_handle {:?}", multibody_handle);
    println!("multibody {:#?}", multibody_joints.get_mut(multibody_handle));
    
    let jt_angles = inv_kin(&mut bodies,
                          &mut multibody_joints,
                          multibody_handle,
                          Isometry::identity()
    );
    let ee_pose = fwd_kin(&mut bodies,
                          &mut multibody_joints,
                          multibody_handle,
                          rigid_body_2,
                          jt_angles.as_slice());
    println!("jt_angles: {:?}", jt_angles);
    println!("ee_pose:   {:?}", ee_pose);
}
