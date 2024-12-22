use rapier3d::prelude::*;

mod kinematics;
use kinematics::kinematics::fwd_kin;

fn main() {
    // The set that will contain our rigid-bodies.
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    let mut collider_set = ColliderSet::new();

    let rigid_body_1 = bodies.insert(RigidBodyBuilder::new(RigidBodyType::Dynamic)
        .additional_mass_properties(MassProperties::new(
            point![0.0, 1.0, 0.0],
            0.5,
            vector![0.3, 0.2, 0.1],
        ))
        .build());
    
    let rigid_body_2 = bodies.insert(RigidBodyBuilder::new(RigidBodyType::Dynamic)
        .additional_mass(0.2)
        .build());

    let ax_1 = Vector::x_axis();
    let jt_1 = RevoluteJointBuilder::new(ax_1)
        .local_anchor1(point![0., 0., 0.])
        .local_anchor2(point![0., 0., 0.]);
    joints.insert(rigid_body_1, rigid_body_2, jt_1, true);

    let gravity = vector![0.0, 0.0, -9.81];
    fwd_kin(&bodies, &joints, &vec![0.1, 0.2]);
    
}
