mod robot;
mod robot_sphere_collision;

use robot::Robot;
use robot_sphere_collision::RobotSphereCollisionChecker;

use rtpl::real_vector::euclidean::{planners::RRTstar, region::Sphere, EuclideanSteering};
use rtpl::real_vector::{sampling::GoalBiasedUniformDistribution, RealVectorState};

fn main() {
    // Create a robot instance
    let joint_limits = [(-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57)];
    let robot = Robot::new(
        "examples/3dof/cylinder_three.urdf",
        3,
        joint_limits.clone().to_vec(),
    );
    let robot = robot.expect("Failed to create robot.");

    // Create spherical obstacles
    let spheres = vec![
        Sphere::new(RealVectorState::new([10.0, 0.0, 0.0]), 0.5),
        Sphere::new(RealVectorState::new([10.0, 0.0, 1.0]), 0.5),
        Sphere::new(RealVectorState::new([10.0, 0.0, 2.0]), 0.5),
    ];

    // Create a collision checker
    let validity_checker = RobotSphereCollisionChecker::new(robot, spheres, 10);

    // Define the start and goal points.
    let start = RealVectorState::new([0.0, 0.0, 0.0]);
    let goal_state = RealVectorState::new([1.57, 1.57, 1.57]);
    let goal_tolerance = 0.05;
    let goal_region = Sphere::new(goal_state, goal_tolerance);

    // Define the steering function.
    let steering = EuclideanSteering::new(0.05);

    // Use a uniform sampling distribution with 5% goal bias.
    let goal_bias = 0.05;
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

    rrt_star.run_iterations(1000);

    // Print the path
    let path = rrt_star.get_path();
    if path.is_none() {
        println!("No path found.");
        return;
    }
    let path = path.unwrap();

    for state in path {
        println!("{:?}", state);
    }
}
