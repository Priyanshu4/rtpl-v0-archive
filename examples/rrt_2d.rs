//! # Rapidly-exploring Random Tree (RRT) Example in 2 Dimensions
//!
//! ## Usage
//! Run the program with:
//! ```bash
//! cargo run --example rrt2d
//! ```

use macroquad::prelude::*;
use rtpl::base::collision::CollisionRegionDiscretizationType;
use rtpl::base::{collision::CollisionRegion, region::UnionRegion, Motion, Region};
use rtpl::real_vector::euclidean::{planners::RRT, region::Sphere, EuclideanSteering};
use rtpl::real_vector::{sampling::GoalBiasedUniformDistribution, RealVectorState};

const SCREEN_HEIGHT: i32 = 600;
const SCREEN_WIDTH: i32 = 600;

fn window_conf() -> Conf {
    Conf {
        window_title: "RRT in a 2D Environment with Spherical Obstacles".to_string(),
        window_width: SCREEN_HEIGHT,
        window_height: SCREEN_WIDTH,
        window_resizable: false,
        fullscreen: false,
        ..Default::default()
    }
}

type Circle = Sphere<f32, 2>;

#[macroquad::main(window_conf)]
async fn main() {
    // Define the obstacles
    let circles: Vec<Circle> = vec![
        Circle::new(RealVectorState::new([400.0, 400.0]), 50.0),
        Circle::new(RealVectorState::new([400.0, 320.0]), 50.0),
        Circle::new(RealVectorState::new([200.0, 200.0]), 100.0),
        Circle::new(RealVectorState::new([300.0, 200.0]), 100.0),
        Circle::new(RealVectorState::new([200.0, 420.0]), 100.0),
        Circle::new(RealVectorState::new([400.0, 200.0]), 100.0),
    ];

    let boxed_circles: Vec<Box<dyn Region<RealVectorState<f32, 2>>>> = circles
        .iter()
        .map(|circle| Box::new(circle.clone()) as Box<dyn Region<_>>)
        .collect();

    let validity_checker = CollisionRegion::new(
        Box::new(UnionRegion::new(boxed_circles)),
        CollisionRegionDiscretizationType::Resolution(5.0),
    );

    // Define the start and goal points.
    let start = RealVectorState::new([100.0, 100.0]);
    let goal_state = RealVectorState::new([500.0, 500.0]);
    let goal_tolerance = 10.0;
    let goal_region = Circle::new(goal_state, goal_tolerance);

    // Define the steering function.
    let steering = EuclideanSteering::new(10.0);

    // Use a uniform sampling distribution with 5% goal bias.
    let ranges = [(0.0, SCREEN_WIDTH as f32), (0.0, SCREEN_HEIGHT as f32)];
    let goal_bias = 0.05;
    let sampling_distribution = GoalBiasedUniformDistribution::new(ranges, goal_state, goal_bias)
        .expect("Failed to create sampling distribution.");

    // Create the RRT planner.
    let mut rrt = RRT::new(
        start.clone(),
        Box::new(goal_region),
        Box::new(validity_checker),
        Box::new(sampling_distribution),
        Box::new(steering),
    );

    loop {
        // Clear the screen
        clear_background(WHITE);

        // Draw the obstacles
        for circle in &circles {
            draw_circle(
                circle.center()[0],
                circle.center()[1],
                circle.radius(),
                BLACK,
            );
        }

        // Draw the start and goal points.
        draw_circle(start[0], start[1], 5.0, BLUE);
        draw_circle(goal_state[0], goal_state[1], goal_tolerance, GREEN);

        if !rrt.solved() {
            rrt.run_iterations(1);
        }

        // Draw each node and the edge to its parent.
        let nodes = rrt.get_nodes();
        for node in nodes {
            let state = node.state();
            if let Some(parent_index) = node.parent() {
                let parent = &nodes[parent_index];
                let parent_state = parent.state();
                draw_line(
                    state[0],
                    state[1],
                    parent_state[0],
                    parent_state[1],
                    1.0,
                    BLACK,
                );
            }
            draw_circle(state[0], state[1], 2.0, BLACK);
        }

        // Draw the path if a solution was found.
        if let Some(path) = rrt.get_path() {
            // Raw path
            for i in 0..path.len() - 1 {
                let a = path[i].state();
                let b = path[i + 1].state();
                draw_line(a[0], a[1], b[0], b[1], 2.0, RED);
            }
        }

        next_frame().await;
    }
}
