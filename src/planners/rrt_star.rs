use core::panic;

use crate::base::motion::{HasCost, Motion};
use crate::base::NearestNeighbors;
use crate::base::Region;
use crate::base::SamplingDistribution;
use crate::base::State;
use crate::base::Steering;
use crate::base::TerminationCondition;
use crate::base::ValidityChecker;
use num_traits::Float;

/// A node in the RRT* tree.
#[derive(Clone)]
pub struct Node<S, M, F> {
    /// The motion in the state space.
    motion: M,
    /// The index of the parent node (None if the node is the root).
    parent: Option<usize>,
    /// Cost from the root to this node.
    cumulative_cost: F,

    /// phantom data
    _state: std::marker::PhantomData<S>,
}

impl<F: Float, S: State, M: Motion<S> + HasCost<F>> Node<S, M, F> {
    pub fn new(motion: M, parent: Option<usize>, cumulative_cost: F) -> Self {
        Self {
            motion,
            parent,
            cumulative_cost,
            _state: std::marker::PhantomData,
        }
    }

    pub fn motion(&self) -> &M {
        &self.motion
    }

    pub fn state(&self) -> &S {
        self.motion.state()
    }

    pub fn parent(&self) -> Option<usize> {
        self.parent
    }

    pub fn cumulative_cost(&self) -> F {
        self.cumulative_cost
    }
}

/// A Rapidly-exploring Random Tree Star (RRT*) planner.
pub struct RRTstar<S, M, F, NN> {
    goal_region: Box<dyn Region<S>>,
    validity_checker: Box<dyn ValidityChecker<S, M>>,
    sampling_distribution: Box<dyn SamplingDistribution<S>>,
    steering: Box<dyn Steering<S, M>>,
    nearest_neighbors: NN,
    nodes: Vec<Node<S, M, F>>,
    solution: Option<usize>,
    max_connection_radius: F,
    gamma: F,
}

impl<S, M, F, NN> RRTstar<S, M, F, NN>
where
    S: State + Clone,
    M: Motion<S> + HasCost<F> + Clone,
    F: Float,
    NN: NearestNeighbors<S, F> + Default,
{
    ///
    /// Parameters:
    /// - `start`: The start state.
    /// - `goal`: The goal region.
    /// - `goal_tolerance`: The tolerance for reaching the goal.
    /// - `validity_checker`: Checks if the edges or nodes as valid.
    /// - `sampling_distribution`: The sampling distribution.
    /// - `steering`: The steering function.
    /// Returns the RRT planner.
    pub fn new(
        start: S,
        goal_region: Box<dyn Region<S>>,
        validity_checker: Box<dyn ValidityChecker<S, M>>,
        sampling_distribution: Box<dyn SamplingDistribution<S>>,
        steering: Box<dyn Steering<S, M>>,
        max_connection_radius: F,
        gamma: F,
    ) -> Self {
        let mut rrt = Self {
            goal_region,
            validity_checker,
            sampling_distribution,
            steering,
            nearest_neighbors: NN::default(),
            nodes: Vec::new(),
            solution: None,
            max_connection_radius,
            gamma,
        };

        let root = Node::new(M::zero(start), None, F::zero());
        rrt.add_node(root);
        rrt
    }

    /// Attempts to find a solution until a termination condition is met.
    ///
    /// Parameters:
    /// - `termination_condition`: The termination condition.
    /// Returns true if a solution was found.
    pub fn plan_until_solved<TC>(&mut self, termination_condition: &mut TC) -> bool
    where
        TC: TerminationCondition,
    {
        while !termination_condition.evaluate() {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
    }

    /// Plan until a termination condition is met. Does not terminate early if a solution is found.
    ///
    /// Parameters:
    /// - `termination_condition`: The termination condition.
    pub fn plan_until<TC>(&mut self, termination_condition: &mut TC)
    where
        TC: TerminationCondition,
    {
        while !termination_condition.evaluate() {
            self.iteration();
        }
    }

    /// Run a fixed number of iterations of the RRT algorithm. Does not terminate early if a solution is found.
    ///    
    /// Parameters:
    /// - `iterations`: The number of iterations to run.
    pub fn run_iterations(&mut self, iterations: u32) {
        for _ in 0..iterations {
            self.iteration();
        }
    }

    /// Returns true if a solution was found.
    pub fn solved(&self) -> bool {
        self.solution.is_some()
    }

    /// Returns the path from the start to the goal, if a solution was found.
    pub fn get_path(&self) -> Option<Vec<M>> {
        if !self.solved() {
            return None;
        }

        let mut path = Vec::new();
        let mut current_index = self.solution.unwrap();

        // Reconstruct the path by backtracking up the tree (following the parent pointers).
        while let Some(parent_index) = self.nodes[current_index].parent {
            path.push(self.nodes[current_index].motion.clone());
            current_index = parent_index;
        }
        path.push(self.nodes[current_index].motion.clone());

        // Reverse the path so that it goes from the start to the goal.
        path.reverse();
        Some(path)
    }

    /// Returns the vector of nodes in the tree.
    pub fn get_nodes(&self) -> &Vec<Node<S, M, F>> {
        &self.nodes
    }

    /// Expands the tree by one iteration.
    fn iteration(&mut self) {
        // Sample a point from the sampling distribution.
        let sample = self.sampling_distribution.sample();

        // Find the nearest node in the tree to the sample point.
        let nearest_node_index = self.nearest_neighbors.nearest_one(&sample).unwrap();
        let nearest_node = &self.nodes[nearest_node_index];
        let nearest_state = self.nodes[nearest_node_index].state();

        // Steer the nearest node towards the sample point to get a new point.
        let new_motion = self.steering.steer_towards(nearest_state, &sample);

        // If the new motion is invalid, return.
        if !self
            .validity_checker
            .is_motion_valid(nearest_state, &new_motion)
        {
            return;
        }

        // Create the new node and add it to the tree.
        let cumulative_cost = nearest_node.cumulative_cost + new_motion.cost();
        let new_node = Node::new(new_motion, Some(nearest_node_index), cumulative_cost);

        // Find the neighbors of the new node.
        let neighbors = self
            .nearest_neighbors
            .within_radius(&new_node.state(), self.rewiring_radius());

        // Rewire the new node to the best parent and add to the tree.
        let new_node = self.rewire_to_best_parent(new_node, &neighbors);
        let new_node_index = self.add_node(new_node);

        // Rewire the neighbors of the new node.
        self.rewire_neighbors(new_node_index, &neighbors);

        // If the goal is reached at a lower cost update the solution node.
        if !self
            .goal_region
            .contains(self.nodes[new_node_index].state())
        {
            return;
        }

        if self.solution.is_none()
            || self.nodes[new_node_index].cumulative_cost
                < self.nodes[self.solution.unwrap()].cumulative_cost
        {
            self.solution = Some(new_node_index);
        }
    }

    /// Find the best parent for the new node and rewire it to the new node.
    fn rewire_to_best_parent(
        &mut self,
        node: Node<S, M, F>,
        neighbors: &Vec<usize>,
    ) -> Node<S, M, F> {
        let mut node = node;
        for &neighbor_index in neighbors {
            if node.parent.is_some() && neighbor_index == node.parent.unwrap() {
                // Skip node which is already the parent.
                continue;
            }

            let neighbor = &self.nodes[neighbor_index];
            let neighbor_state = neighbor.state();
            let new_motion = self.steering.steer_exact(neighbor_state, &node.state());

            if new_motion.is_none() {
                continue;
            }
            let new_motion = new_motion.unwrap();

            if self
                .validity_checker
                .is_motion_valid(neighbor_state, &new_motion)
            {
                let cumulative_cost = neighbor.cumulative_cost + new_motion.cost();
                if cumulative_cost < node.cumulative_cost {
                    node = Node::new(new_motion, Some(neighbor_index), cumulative_cost);
                }
            }
        }
        return node;
    }

    /// Checks if the new node is a better parent for the neighbors of the new node.
    fn rewire_neighbors(&mut self, node_index: usize, neighbors: &Vec<usize>) {
        for &neighbor_index in neighbors {
            let neighbor = &self.nodes[neighbor_index];
            let neighbor_state = neighbor.state();
            let new_motion = self
                .steering
                .steer_exact(&self.nodes[node_index].state(), neighbor_state);

            if new_motion.is_none() {
                continue;
            }
            let new_motion = new_motion.unwrap();

            if self
                .validity_checker
                .is_motion_valid(&self.nodes[node_index].state(), &new_motion)
            {
                let cumulative_cost = self.nodes[node_index].cumulative_cost + new_motion.cost();
                if cumulative_cost < neighbor.cumulative_cost {
                    self.nodes[neighbor_index] =
                        Node::new(new_motion, Some(node_index), cumulative_cost);
                }
            }
        }
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<S, M, F>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.state().clone(), index);
        self.nodes.push(node);
        index
    }

    fn rewiring_radius(&self) -> F {
        let card_v = F::from(self.nodes.len()).unwrap();
        let d = F::from(self.nodes[0].state().dimension()).unwrap();
        let radius = self.gamma * (card_v.ln() / card_v).powf(F::one() / d);
        if radius > self.max_connection_radius {
            self.max_connection_radius
        } else {
            radius
        }
    }

    pub fn sampling_distribution(&self) -> &Box<dyn SamplingDistribution<S>> {
        &self.sampling_distribution
    }

    pub fn set_sampling_distribution(
        &mut self,
        sampling_distribution: Box<dyn SamplingDistribution<S>>,
    ) {
        self.sampling_distribution = sampling_distribution;
    }
}

/// Computes gamma value to achieve asymptotic optimality for the RRT* algorithm.
/// Parameters:
/// - `free_space_volume`: The volume of the free space.
/// - `dimension`: The dimension of the state space.
///
/// Returns:
/// The optimal gamma value.
pub fn optimal_gamma(free_space_volume: f32, dimension: usize) -> f32 {
    if free_space_volume <= 0.0 {
        panic!("The free space volume must be positive.");
    }

    if dimension == 0 {
        panic!("The dimension must be positive.");
    }

    let unit_ball_volume = (std::f32::consts::PI.powf((dimension as f32) / 2.0))
        / special::Gamma::gamma(1.0 + (dimension as f32) / 2.0);

    let gamma = (2.0 * (1.0 + 1.0 / dimension as f32) * free_space_volume / unit_ball_volume)
        .powf(1.0 / dimension as f32);

    gamma
}
