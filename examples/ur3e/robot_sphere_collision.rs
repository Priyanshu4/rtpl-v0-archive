use super::robot::{Robot, Sphere};
use k::nalgebra as na;
use rtpl::base::motion::Discretizable;
use rtpl::base::ValidityChecker;
use rtpl::real_vector::euclidean::region::Sphere as CollisionSphere;
use rtpl::real_vector::euclidean::EuclideanMotion;
use rtpl::real_vector::RealVectorState;
use std::cell::RefCell;

pub struct RobotSphereCollisionChecker {
    robot: RefCell<Robot>,
    spheres: Vec<CollisionSphere<f32, 3>>,
    discretization_steps: usize,
}

impl RobotSphereCollisionChecker {
    pub fn new(
        robot: Robot,
        spheres: Vec<CollisionSphere<f32, 3>>,
        discretization_steps: usize,
    ) -> Self {
        Self {
            robot: RefCell::new(robot),
            spheres,
            discretization_steps,
        }
    }
}

impl<const N: usize> ValidityChecker<RealVectorState<f32, N>, EuclideanMotion<f32, N>>
    for RobotSphereCollisionChecker
{
    fn is_state_valid(&self, state: &RealVectorState<f32, N>) -> bool {
        let mut robot = self.robot.borrow_mut();
        let result = robot.forward_kinematics(state.values());
        result.expect("Failed to set joint positions");

        let link_spheres = robot.spheres();
        let sphere_poses = robot.sphere_poses();

        for collision_sphere in &self.spheres {
            let sphere_center = collision_sphere.center();
            let sphere_center =
                na::Point3::new(sphere_center[0], sphere_center[1], sphere_center[2]);
            let sphere_radius = collision_sphere.radius();

            for (link_sphere, pose) in link_spheres.iter().zip(sphere_poses.iter()) {
                if sphere_sphere_intersect(
                    link_sphere.radius,
                    pose.0.translation.vector,
                    sphere_radius,
                    sphere_center,
                ) {
                    return false;
                }
            }
        }

        true
    }

    fn is_motion_valid(
        &self,
        initial_state: &RealVectorState<f32, N>,
        motion: &EuclideanMotion<f32, N>,
    ) -> bool {
        let states = motion.discretize(initial_state, self.discretization_steps);
        states.iter().all(|state| self.is_state_valid(state))
    }
}

/// Checks if two spheres intersect.
/// The first sphere is represented by its radius and center (`na::Vector3<f32>`).
/// The second sphere is represented by its radius and center (`na::Point3<f32>`).
fn sphere_sphere_intersect(
    radius1: f32,
    center1: na::Vector3<f32>,
    radius2: f32,
    center2: na::Point3<f32>,
) -> bool {
    let center1 = na::Point3::new(center1.x, center1.y, center1.z);
    let distance_squared = (center1 - center2).norm_squared();
    let combined_radius = radius1 + radius2;

    distance_squared <= combined_radius.powi(2)
}
