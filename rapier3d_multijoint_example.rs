use rapier3d::prelude::*;


fn main() {
    // The set that will contain our rigid-bodies.
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    let mut collider_set = ColliderSet::new();
    
    // Builder for a body with a status specified by an enum.
    let rigid_body_1 = bodies.insert(RigidBodyBuilder::new(RigidBodyType::Dynamic)
        .position(Isometry::new(
            vector![1.0, 3.0, 2.0],
            vector![0.0, 0.0, 0.4],
        ))
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
    let integration_parameters = IntegrationParameters::default();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    for _ in 0..200 {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut joints,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler
        );
        let pos1 = bodies[rigid_body_1].position().translation;
        let pos2 = bodies[rigid_body_2].position().translation;
        println!("Body 2 Position: {:?}", pos2);
    }
    
}
