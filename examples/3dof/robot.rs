use k::{nalgebra as na, Chain, InverseKinematicsSolver, SerialChain};
use urdf_rs::{read_file, Geometry};

#[derive(Debug)]
pub struct Cylinder {
    pub radius: f32,
    pub length: f32,
    pub name: String,
}

pub struct Robot {
    chain: SerialChain<f32>,
    cylinders: Vec<Cylinder>,
    solver: k::JacobianIkSolver<f32>,
    joint_limits: Vec<(f32, f32)>,
}

impl Robot {
    /// Creates a new Robot instance, parsing the chain and extracting cylinders.
    pub fn new<P: AsRef<std::path::Path>>(
        urdf_path: P,
        expected_cylinders: usize,
        joint_limits: Vec<(f32, f32)>,
    ) -> Result<Self, String> {
        // Parse URDF
        let urdf_robot =
            read_file(urdf_path).map_err(|e| format!("Failed to load URDF: {:?}", e))?;
        let chain = Chain::from(&urdf_robot);
        let chain = SerialChain::try_new(chain);
        if chain.is_none() {
            return Err("Robot chain has branches".to_string());
        }
        let chain = chain.unwrap();

        // Extract cylinders
        let mut cylinders = Vec::new();
        for link in &urdf_robot.links {
            if let Some(visual) = link.visual.first() {
                if let Geometry::Cylinder { radius, length } = &visual.geometry {
                    cylinders.push(Cylinder {
                        radius: *radius as f32,
                        length: *length as f32,
                        name: link.name.clone(),
                    });
                }
            }
        }
        if cylinders.len() != expected_cylinders {
            return Err(format!(
                "Expected {} cylinders but found {} in the URDF",
                expected_cylinders,
                cylinders.len()
            ));
        }

        let solver = k::JacobianIkSolver::default();
        Ok(Self {
            chain,
            cylinders,
            solver,
            joint_limits,
        })
    }

    /// Gets the joint angles of the robot.
    pub fn joint_positions(&self) -> Vec<f32> {
        self.chain.joint_positions().to_vec()
    }

    /// Sets the joint angles of the robot ajd updates the chain.
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

    /// Returns the position and orientation of each cylinder.
    pub fn cylinder_poses(&self) -> Vec<na::Isometry3<f32>> {
        self.chain.update_transforms();
        self.cylinders
            .iter()
            .map(|cylinder| {
                let link = self
                    .chain
                    .find_link(&cylinder.name)
                    .expect(&format!("Link {} not found", &cylinder.name));
                link.world_transform().unwrap()
            })
            .collect()
    }

    pub fn cylinders(&self) -> &[Cylinder] {
        &self.cylinders
    }

    pub fn joint_limits(&self) -> &[(f32, f32)] {
        &self.joint_limits
    }
}
