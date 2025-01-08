use super::robot::{Cylinder, Robot};
use k::nalgebra as na;
use rtpl::base::motion::Discretizable;
use rtpl::base::Motion;
use rtpl::base::ValidityChecker;
use rtpl::real_vector::euclidean::region::Sphere;
use rtpl::real_vector::euclidean::EuclideanMotion;
use rtpl::real_vector::RealVectorState;
use std::cell::RefCell;

pub struct RobotSphereCollisionChecker {
    robot: RefCell<Robot>,
    spheres: Vec<Sphere<f32, 3>>,
    discretization_steps: usize,
}

impl RobotSphereCollisionChecker {
    pub fn new(robot: Robot, spheres: Vec<Sphere<f32, 3>>, discretization_steps: usize) -> Self {
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

        let cylinders: &[Cylinder] = robot.cylinders();
        let cylinder_poses = robot.cylinder_poses();

        for sphere in &self.spheres {
            let sphere_center = sphere.center();
            let sphere_center =
                na::Point3::new(sphere_center[0], sphere_center[1], sphere_center[2]);
            let sphere_radius = sphere.radius();

            for (cylinder, pose) in cylinders.iter().zip(cylinder_poses.iter()) {
                if cylinder_sphere_intersect(
                    cylinder.length,
                    cylinder.radius,
                    pose.clone(),
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

/// Checks if a cylinder and a sphere intersect.
/// The cylinder is represented by its length, radius, and pose (`na::Isometry3<f32>`).
/// The cylinder pose is for its base.
/// The sphere is represented by its radius and center (`na::Point3<f32>`).
fn cylinder_sphere_intersect(
    cylinder_length: f32,
    cylinder_radius: f32,
    cylinder_pose: na::Isometry3<f32>,
    sphere_radius: f32,
    sphere_center: na::Point3<f32>,
) -> bool {
    // Compute the cylinder center by translating the base pose along the local z-axis
    let cylinder_center_pose = cylinder_pose
        * na::Isometry3::new(
            na::Vector3::new(0.0, 0.0, cylinder_length / 2.0), // Translation along z-axis
            na::Vector3::zeros(),                              // No rotation adjustment
        );

    // Transform the sphere center to the cylinder's local space
    let local_sphere_center = cylinder_center_pose.inverse_transform_point(&sphere_center);

    // Check if the sphere center's projection onto the z-axis is within the cylinder's length
    let half_length = cylinder_length / 2.0;
    if local_sphere_center.z < -half_length - sphere_radius
        || local_sphere_center.z > half_length + sphere_radius
    {
        return false;
    }

    // Compute the distance from the sphere center to the cylinder's axis (x-y plane distance)
    let distance_to_axis_squared = local_sphere_center.x.powi(2) + local_sphere_center.y.powi(2);
    let max_distance = cylinder_radius + sphere_radius;

    // Check if the sphere intersects the cylinder
    distance_to_axis_squared <= max_distance.powi(2)
}
