use ggez::event;
use ggez::graphics::{Canvas, Color, DrawParam, Drawable, Image, Rect};
use ggez::mint::Point2 as P2;
use ggez::{Context, GameResult};
#[allow(unused_imports)]
use glam::{Quat, Vec2};
#[allow(unused_imports)]
use nalgebra::{point, vector, Isometry2, Point2, Vector2};
use rapier2d::prelude::*;

struct MainState {
    ball_image1: Image,
    frame_index: usize,
    frame_width1: f32,
    frame_height1: f32,
    frame_count: usize,
    ball_image2: Image,
    frame_width2: f32,
    frame_height2: f32,
    floor: Image,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        // 1 row x 6 column sprite sheet
        let ball_image1 = Image::from_path(ctx, "/ball_sheet.png")?;
        let dimensions1 = ball_image1.dimensions(ctx).unwrap();
        let frame_size = (32.0, 32.0);
        let frame_count = 6;
        let frame_width1 = frame_size.0 / dimensions1.w;

        let ball_image2 = Image::from_path(ctx, "/ball_sheet2.png")?;
        let dimensions2 = ball_image2.dimensions(ctx).unwrap();
        let frame_height2 = frame_size.1 / dimensions2.h;
        let frame_width2 = frame_size.0 / dimensions2.w;

        let floor = Image::from_path(ctx, "/floor.png")?;

        Ok(MainState {
            ball_image1,
            frame_index: 0,
            frame_width1,
            frame_height1: dimensions1.h / frame_size.1,
            frame_count,
            ball_image2,
            frame_width2,
            frame_height2,
            floor,
        })
    }
}

impl event::EventHandler for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time(5) {
            self.frame_index += 1;
            self.frame_index %= self.frame_count;
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = Canvas::from_frame(ctx, Color::BLACK);

        let ball_rect1 = Rect::new(
            self.frame_index as f32 * self.frame_width1,
            0.0,
            self.frame_width1,
            self.frame_height1,
        );

        let ball_rect2 = Rect::new(
            (self.frame_index % 3) as f32 * self.frame_width2,
            (self.frame_index / (self.frame_count / 2)) as f32 / 2.0,
            self.frame_width2,
            self.frame_height2,
        );

        canvas.draw(
            &self.ball_image1,
            DrawParam::new().src(ball_rect1).dest(P2 { x: 9.0, y: 9.0 }),
        );

        canvas.draw(
            &self.ball_image2,
            DrawParam::new()
                .src(ball_rect2)
                .dest(P2 { x: 109.0, y: 109.0 }),
        );

        let repitions = 800 / self.floor.width();

        for i in 0..repitions {
            canvas.draw(
                &self.floor,
                DrawParam::new().dest([(i * self.floor.width()) as f32, 600.0 - self.floor.height() as f32]),
            );
        }

        canvas.finish(ctx)?;
        Ok(())
    }
}

pub fn main() -> GameResult {
    let (mut ctx, event_loop) = ggez::ContextBuilder::new("my_game", "My Name").build()?;
    let state = MainState::new(&mut ctx)?;
    event::run(ctx, event_loop, state);
    // let _v = Vector2::new(1.0, 2.0);
    // let _v = vector![1.0, 2.0];
    // let v: Vector2<f64> = [1.0, 2.0].into();

    // assert_eq!(v.x, 1.0);
    // assert_eq!(v.y, 2.0);
    // assert_eq!(v[0], 1.0);
    // assert_eq!(v[1], 2.0);

    // let _pt = Point2::new(1.0, 2.0);
    // let _pt = point![1.0, 2.0];
    // let pt: Point2<f64> = [1.0, 2.0].into();

    // assert_eq!(pt.x, 1.0);
    // assert_eq!(pt.y, 2.0);
    // assert_eq!(pt[0], 1.0);
    // assert_eq!(pt[1], 2.0);

    // let _iso = Isometry2::translation(1.0, 2.0);
    // let _iso = Isometry2::rotation(0.5);
    // let iso = Isometry2::new(vector![1.0, 2.0], 0.5);

    // assert_eq!(iso.rotation.angle(), 0.5);
    // assert_eq!(iso.translation.vector.x, 1.0);
    // assert_eq!(iso.translation.vector.y, 2.0);

    // // These into statements don't work for some reason...
    // // let na_vector = vector![1.0, 2.0];
    // // let glam_vector: Vec2 = na_vector.into();

    // // let glam_vecotr = Vec2::new(1.0, 2.0);
    // // let na_point: Point2<f32> = glam_vecotr.into();

    // // let na_isometry: Isometry2<f32> = (Vec2::new(0.1, 0.2), Quat::from_rotation_x(0.4)).into();

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
    //     );

    //     let ball_body = &rigid_body_set[ball_body_handle];
    //     println!("Ball altitude: {}", ball_body.translation().y);
    // }

    // let ray = Ray::new(point![1.0, 2.0], vector![0.0, 1.0]);
    // let max_toi = 4.0;
    // let solid = true;
    // let filter = QueryFilter::default();

    // if let Some((handle, toi)) =
    //     query_pipeline.cast_ray(&rigid_body_set, &collider_set, &ray, max_toi, solid, filter)
    // {
    //     // The first collider hit has the handle `handle` and it hit after
    //     // the ray travelled a distance equal to `ray.dir * toi`.
    //     let hit_point = ray.point_at(toi); // Same as: `ray.origin + ray.dir * toi`
    //     println!("Collider {:?} hit at point {}", handle, hit_point);
    // }

    // if let Some((handle, intersection)) = query_pipeline.cast_ray_and_get_normal(
    //     &rigid_body_set,
    //     &collider_set,
    //     &ray,
    //     max_toi,
    //     solid,
    //     filter,
    // ) {
    //     // This is similar to `QueryPipeline::cast_ray` illustrated above except
    //     // that it also returns the normal of the collider shape at the hit point.
    //     let hit_point = ray.point_at(intersection.toi);
    //     let hit_normal = intersection.normal;
    //     println!(
    //         "Collider {:?} hit at point {} with normal {}",
    //         handle, hit_point, hit_normal
    //     );
    // }

    // query_pipeline.intersections_with_ray(
    //     &rigid_body_set,
    //     &collider_set,
    //     &ray,
    //     max_toi,
    //     solid,
    //     filter,
    //     |handle, intersection| {
    //         // Callback called on each collider hit by the ray.
    //         let hit_point = ray.point_at(intersection.toi);
    //         let hit_normal = intersection.normal;
    //         println!(
    //             "Collider {:?} hit at point {} with normal {}",
    //             handle, hit_point, hit_normal
    //         );
    //         true // Return `false` instead if we want to stop searching for other hits.
    //     },
    // );

    // let shape = Cuboid::new(vector![1.0, 2.0]);
    // let shape_pos = Isometry::new(vector![0.0, 1.0], 0.8);
    // let shape_vel = vector![0.1, 0.4];
    // let max_toi = 4.0;
    // let filter = QueryFilter::default();

    // if let Some((handle, hit)) = query_pipeline.cast_shape(
    //     &rigid_body_set,
    //     &collider_set,
    //     &shape_pos,
    //     &shape_vel,
    //     &shape,
    //     max_toi,
    //     true,
    //     filter,
    // ) {
    //     // The first collider hit has the handle `handle`. The `hit` is a
    //     // structure containing details about the hit configuration.
    //     println!(
    //         "Hit the collider {:?} with the configuration: {:?}",
    //         handle, hit
    //     );
    // }
}
