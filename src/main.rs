use rapier3d::prelude::*;

mod kinematics;
use kinematics::kinematics::fwd_kin;

fn main() {
    // The set that will contain our rigid-bodies.
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    //let mut collider_set = ColliderSet::new();
    //let mut physics_pipeline = PhysicsPipeline::new();
    //let gravity = vector![0.0, 0.0, -9.81];

    let rigid_body_1 = bodies.insert(RigidBodyBuilder::fixed());
    
    let rigid_body_2 = bodies.insert(RigidBodyBuilder::new(RigidBodyType::Dynamic)
                                     .additional_mass(0.2)
                                     .can_sleep(false)
                                     //.build()
    );

    let ax_1 = Vector::x_axis();
    let jt_1 = RevoluteJointBuilder::new(ax_1)
        .local_anchor1(point![0., 0., 0.3])
        .local_anchor2(point![0., 0., -0.3])
        .build()
        .data;

    let ee_link = joints.insert(rigid_body_1, rigid_body_2, jt_1, true).unwrap();
    
    fwd_kin(&mut bodies, &mut joints, ee_link, &vec![0.2,0.3,0.,0.,0.,0., 0.1]);
    
}
