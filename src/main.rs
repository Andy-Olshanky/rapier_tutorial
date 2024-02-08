use nalgebra::{point, vector, Point2, Vector2, Isometry2};
use glam::{Quat, Vec2};
use rapier2d::prelude::*;

fn main() {
    let _v = Vector2::new(1.0, 2.0);
    let _v = vector![1.0, 2.0];
    let v: Vector2<f64> = [1.0, 2.0].into();

    assert_eq!(v.x, 1.0);
    assert_eq!(v.y, 2.0);
    assert_eq!(v[0], 1.0);
    assert_eq!(v[1], 2.0);

    let _pt = Point2::new(1.0, 2.0);
    let _pt = point![1.0, 2.0];
    let pt: Point2<f64> = [1.0, 2.0].into();

    assert_eq!(pt.x, 1.0);
    assert_eq!(pt.y, 2.0);
    assert_eq!(pt[0], 1.0);
    assert_eq!(pt[1], 2.0);
    
    let _iso = Isometry2::translation(1.0, 2.0);
    let _iso = Isometry2::rotation(0.5);
    let iso = Isometry2::new(vector![1.0, 2.0], 0.5);

    assert_eq!(iso.rotation.angle(), 0.5);
    assert_eq!(iso.translation.vector.x, 1.0);
    assert_eq!(iso.translation.vector.y, 2.0);

    // These into statements don't work for some reason...
    // let na_vector = vector![1.0, 2.0];
    // let glam_vector: Vec2 = na_vector.into();

    // let glam_vecotr = Vec2::new(1.0, 2.0);
    // let na_point: Point2<f32> = glam_vecotr.into();

    // let na_isometry: Isometry2<f32> = (Vec2::new(0.1, 0.2), Quat::from_rotation_x(0.4)).into();


    // let mut rigid_body_set = RigidBodySet::new();
    // let mut collider_set = ColliderSet::new();

    // /* Create the ground. */
    // let collider = ColliderBuilder::cuboid(100.0, 0.1).build();
    // collider_set.insert(collider);

    // /* Create the bouncing ball. */
    // let rigid_body = RigidBodyBuilder::dynamic()
    //     .translation(vector![0.0, 10.0])
    //     .build();
    // let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    // let ball_body_handle = rigid_body_set.insert(rigid_body);
    // collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    // /* Create other structures necessary for the simulation. */
    // let gravity = vector![0.0, -9.81];
    // let integration_parameters = IntegrationParameters::default();
    // let mut physics_pipeline = PhysicsPipeline::new();
    // let mut island_manager = IslandManager::new();
    // let mut broad_phase = BroadPhase::new();
    // let mut narrow_phase = NarrowPhase::new();
    // let mut impulse_joint_set = ImpulseJointSet::new();
    // let mut multibody_joint_set = MultibodyJointSet::new();
    // let mut ccd_solver = CCDSolver::new();
    // let mut query_pipeline = QueryPipeline::new();
    // let physics_hooks = ();
    // let event_handler = ();

    // /* Run the game loop, stepping the simulation once per frame. */
    // for _ in 0..200 {
    //     physics_pipeline.step(
    //         &gravity,
    //         &integration_parameters,
    //         &mut island_manager,
    //         &mut broad_phase,
    //         &mut narrow_phase,
    //         &mut rigid_body_set,
    //         &mut collider_set,
    //         &mut impulse_joint_set,
    //         &mut multibody_joint_set,
    //         &mut ccd_solver,
    //         Some(&mut query_pipeline),
    //         &physics_hooks,
    //         &event_handler,
    //         );

    //         let ball_body = &rigid_body_set[ball_body_handle];
    //         println!(
    //         "Ball altitude: {}",
    //         ball_body.translation().y
    //     );
    // }
}
