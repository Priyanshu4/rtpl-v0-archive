use std::path::Path;

mod robot;
mod robot_sphere_collision;

use robot::Robot;
use robot_sphere_collision::RobotSphereCollisionChecker;

use rtpl::base::termination::MaxIterationsTermination;
use rtpl::base::{Motion, ValidityChecker};
use rtpl::real_vector::euclidean::{planners::RRTstar, region::Sphere, EuclideanSteering};
use rtpl::real_vector::sampling::{GoalBiasedUniformDistribution, UniformDistribution};
use rtpl::real_vector::RealVectorState;

use k::{nalgebra as na, Real};
use na::{Isometry3, Translation3, UnitQuaternion};

fn main() {
    // Create a robot instance
    let urdf_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .canonicalize()
        .expect("Failed to get canonical path")
        .join("examples/ur3e/ur_description/urdf/ur3e_spheres_collision.urdf");

    let joint_limits = [(-3.14, 3.14); 6];
    let robot = Robot::new(&urdf_path, "ur_ee_fixed_joint", joint_limits.to_vec());
    let mut robot = robot.expect("Failed to create robot.");

    // Define the start state. Check that the start state is valid.
    let start = RealVectorState::new([0.0; 6]);

    // Define the goal region. Check that the goal state is valid.
    // Pick random goal state

    let goal_state = RealVectorState::new([-1.57, -1.57, -1.57, 1.57, 1.57, 1.57]);
    let goal_tolerance = 0.05;
    let goal_region = Sphere::new(goal_state, goal_tolerance);

    // Just to test the IK
    robot
        .set_joint_positions(goal_state.values())
        .expect("Fail to set joint positions");
    robot
        .inverse_kinematics(robot.end_transform())
        .expect("IK Fail");
    println!("End Effector Target {:?}", robot.end_transform());

    // Create spherical obstacles
    let spheres = vec![
        Sphere::new(RealVectorState::new([0.0, 0.1, 0.6]), 0.05),
        Sphere::new(RealVectorState::new([0.0, -0.2, 0.4]), 0.1),
        Sphere::new(RealVectorState::new([0.3, 0.0, 0.4]), 0.05),
        Sphere::new(RealVectorState::new([0.3, -0.2, 0.6]), 0.05),
    ];

    // Create a collision checker
    let validity_checker = RobotSphereCollisionChecker::new(robot, spheres.clone(), 10);
    if !validity_checker.is_state_valid(&start) {
        println!("Start state is invalid.");
        return;
    }
    if !validity_checker.is_state_valid(goal_region.center()) {
        println!("Goal state is invalid.");
        return;
    }

    // Define the steering function.
    let steering = EuclideanSteering::new(0.05);

    // Use a uniform sampling distribution with 5% goal bias.
    let goal_bias = 0.05;
    let sampling_distribution =
        GoalBiasedUniformDistribution::new(joint_limits, goal_state, goal_bias)
            .expect("Failed to create sampling distribution.");

    // Create the RRT planner.
    let mut rrt_star: RRTstar<f32, 6> = RRTstar::new(
        start.clone(),
        Box::new(goal_region),
        Box::new(validity_checker),
        Box::new(sampling_distribution),
        Box::new(steering),
        0.1,
        rtpl::planners::rrt_star::optimal_gamma(3.14_f32.powi(6), 6),
    );

    let mut termination = MaxIterationsTermination::new(10000);

    // Remove the goal bias after solved
    rrt_star.plan_until_solved(&mut termination);
    println!(
        "{} iterations used to find an initial solution",
        termination.current_iteration()
    );
    rrt_star.set_sampling_distribution(Box::new(UniformDistribution::new(joint_limits)));
    rrt_star.plan_until(&mut termination);
    println!(
        "Planned for {} iterations in total",
        termination.current_iteration()
    );

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
        .join("examples/ur3e/path.json");
    std::fs::write(file_out, json_string).expect("Failed to write path to file.");

    println!("Path written to examples/ur3e/path.json");
}
