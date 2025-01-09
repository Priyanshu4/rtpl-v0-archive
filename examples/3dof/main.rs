use std::path::Path;

mod robot;
mod robot_sphere_collision;

use robot::Robot;
use robot_sphere_collision::RobotSphereCollisionChecker;

use rtpl::base::{Motion, ValidityChecker};
use rtpl::real_vector::euclidean::{planners::RRTstar, region::Sphere, EuclideanSteering};
use rtpl::real_vector::{sampling::GoalBiasedUniformDistribution, RealVectorState};

fn main() {
    // Create a robot instance
    let urdf_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .canonicalize()
        .expect("Failed to get canonical path")
        .join("examples/3dof/cylinder_three.urdf");

    let joint_limits = [(-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57)];
    let robot = Robot::new(&urdf_path, 3, joint_limits.to_vec());
    let robot = robot.expect("Failed to create robot.");

    // Create spherical obstacles
    let spheres = vec![
        Sphere::new(RealVectorState::new([0.5, 0.9, 2.0]), 0.3),
        Sphere::new(RealVectorState::new([0.8, 1.1, 2.0]), 0.4),
        Sphere::new(RealVectorState::new([0.8, 0.4, 1.1]), 0.5),
    ];

    // Create a collision checker
    let validity_checker = RobotSphereCollisionChecker::new(robot, spheres.clone(), 10);

    // Define the start state. Check that the start state is valid.
    let start = RealVectorState::new([0.0, 0.0, 0.0]);
    if !validity_checker.is_state_valid(&start) {
        println!("Start state is invalid.");
        return;
    }

    // Define the goal region. Check that the goal state is valid.
    let goal_state = RealVectorState::new([1.57, 1.57, 1.57]);
    let goal_tolerance = 0.05;
    let goal_region = Sphere::new(goal_state, goal_tolerance);
    if !validity_checker.is_state_valid(goal_region.center()) {
        println!("Goal state is invalid.");
        return;
    }

    // Define the steering function.
    let steering = EuclideanSteering::new(0.05);

    // Use a uniform sampling distribution with 5% goal bias.
    let goal_bias = 0.0;
    let sampling_distribution =
        GoalBiasedUniformDistribution::new(joint_limits, goal_state, goal_bias)
            .expect("Failed to create sampling distribution.");

    // Create the RRT planner.
    let mut rrt_star: RRTstar<f32, 3> = RRTstar::new(
        start.clone(),
        Box::new(goal_region),
        Box::new(validity_checker),
        Box::new(sampling_distribution),
        Box::new(steering),
        0.1,
        rtpl::planners::rrt_star::optimal_gamma(3.14 * 3.14 * 3.14, 3),
    );

    rrt_star.run_iterations(20000);

    // Print the path
    let path = rrt_star.get_path();
    if path.is_none() {
        println!("No path found.");
        return;
    }
    let path = path.unwrap();

    // Write the spheres, urdf path and planned path to a json file
    // Create a json object
    let mut data = json::object! {
        spheres: json::JsonValue::new_array(),
        urdf_path: urdf_path.to_str().unwrap(),
        start: json::JsonValue::new_array(),
        goal: json::JsonValue::new_array(),
        path: json::JsonValue::new_array(),
    };
    for sphere in spheres {
        let mut json_sphere = json::object! {
            center: json::JsonValue::new_array(),
            radius: sphere.radius(),
        };
        for i in 0..3 {
            json_sphere["center"]
                .push(sphere.center()[i])
                .expect("JSON error");
        }
        data["spheres"].push(json_sphere).expect("JSON error");
    }
    for i in 0..3 {
        data["start"].push(start[i]).expect("JSON error");
        data["goal"].push(goal_state[i]).expect("JSON error");
    }
    for motion in path {
        let mut json_state = json::JsonValue::new_array();
        let state = motion.state();
        for i in 0..3 {
            json_state.push(state[i]).expect("JSON error");
        }
        data["path"].push(json_state).expect("JSON error");
    }

    // Write the json object to a file
    let json_string = data.dump();
    let file_out = Path::new(env!("CARGO_MANIFEST_DIR"))
        .canonicalize()
        .expect("Failed to get canonical path")
        .join("examples/3dof/path.json");
    std::fs::write(file_out, json_string).expect("Failed to write path to file.");

    println!("Path written to examples/3dof/path.json");
}
