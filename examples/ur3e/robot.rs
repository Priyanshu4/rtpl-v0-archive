use k::{nalgebra as na, Chain, InverseKinematicsSolver, SerialChain};
use urdf_rs::{read_file, Geometry};

#[derive(Debug, Clone)]
pub struct Sphere {
    pub radius: f32,
    pub offset: na::Vector3<f32>,
    /// The name of the link this sphere belongs to
    pub link_name: String,
}

pub struct Robot {
    chain: SerialChain<f32>,
    spheres: Vec<Sphere>,
    solver: k::JacobianIkSolver<f32>,
    joint_limits: Vec<(f32, f32)>,
}

impl Robot {
    /// Creates a new Robot instance, parsing the chain and extracting spheres.
    pub fn new<P: AsRef<std::path::Path>>(
        urdf_path: P,
        end_effector_joint_name: &str,
        joint_limits: Vec<(f32, f32)>,
    ) -> Result<Self, String> {
        // Parse URDF
        let urdf_robot =
            read_file(urdf_path).map_err(|e| format!("Failed to load URDF: {:?}", e))?;
        let chain = Chain::from(&urdf_robot);
        let end_joint = chain.find(end_effector_joint_name).ok_or_else(|| {
            format!(
                "End effector {} not found in the URDF",
                end_effector_joint_name
            )
        })?;
        let chain = SerialChain::from_end(end_joint);

        // Extract spheres from collision geometry
        let mut spheres = Vec::new();
        for link in &urdf_robot.links {
            for collision in &link.collision {
                if let Geometry::Sphere { radius } = &collision.geometry {
                    let offset = na::Vector3::new(
                        collision.origin.xyz[0] as f32,
                        collision.origin.xyz[1] as f32,
                        collision.origin.xyz[2] as f32,
                    );
                    spheres.push(Sphere {
                        radius: *radius as f32,
                        offset,
                        link_name: link.name.clone(),
                    });
                }
            }
        }

        let solver = k::JacobianIkSolver::default();
        Ok(Self {
            chain,
            spheres,
            solver,
            joint_limits,
        })
    }

    /// Gets the joint angles of the robot.
    pub fn joint_positions(&self) -> Vec<f32> {
        self.chain.joint_positions().to_vec()
    }

    /// Sets the joint angles of the robot and updates the chain.
    pub fn set_joint_positions(&mut self, joint_positions: &[f32]) -> Result<(), String> {
        self.forward_kinematics(joint_positions)
    }

    /// Sets joint positions and updates the chain.
    pub fn forward_kinematics(&mut self, joint_positions: &[f32]) -> Result<(), String> {
        self.chain
            .set_joint_positions(joint_positions)
            .map_err(|e| format!("Failed to set joint positions: {:?}", e))
    }

    /// Sets joint positions to reach the target positions using inverse kinematics.
    pub fn inverse_kinematics(&self, target_pose: na::Isometry3<f32>) -> Result<(), String> {
        self.solver
            .solve(&self.chain, &target_pose)
            .map_err(|e| format!("Failed to solve IK: {:?}", e))
    }

    /// Returns the position and orientation of each sphere.
    pub fn sphere_poses(&self) -> Vec<(na::Isometry3<f32>, Sphere)> {
        self.chain.update_transforms();
        self.spheres
            .iter()
            .map(|sphere| {
                let link = self
                    .chain
                    .find_link(&sphere.link_name)
                    .expect(&format!("Link {} not found", &sphere.link_name));
                let world_transform = link.world_transform().unwrap();
                (
                    world_transform * na::Translation3::from(sphere.offset),
                    sphere.clone(),
                )
            })
            .collect()
    }

    pub fn spheres(&self) -> &[Sphere] {
        &self.spheres
    }

    pub fn joint_limits(&self) -> &[(f32, f32)] {
        &self.joint_limits
    }
}
