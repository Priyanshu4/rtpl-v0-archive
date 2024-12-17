use crate::collision::ValidityChecker;
use crate::motion::Motion;
use crate::nearest_neighbors::NearestNeighbors;
use crate::region::Region;
use crate::sampling::SamplingDistribution;
use crate::state::State;
use crate::steering::Steering;

/// A node in the RRT tree.
#[derive(Clone)]
pub struct Node<S: State, M: Motion<S>> {
    /// The motion in the state space.
    motion: M,
    /// The index of the parent node (None if the node is the root).
    parent: Option<usize>,
    /// phantom data to store the state type
    _state: std::marker::PhantomData<S>,
}

impl<S: State, M: Motion<S>> Node<S, M> {
    /// Constructs a new node.
    /// Parameters:
    /// - `motion`: The motion in the state space.
    /// - `parent`: The index of the parent node (None if the node is the root).
    pub fn new(motion: M, parent: Option<usize>) -> Self {
        Self {
            motion,
            parent,
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
}

/// A Rapidly-exploring Random Tree (RRT) planner.
/// Template Parameters:
/// - `S`: The type of state.
/// - `M`: The type of motion.
/// - `GR`: The goal region.
/// - `VC`: The validity checker.
/// - `SD`: The sampling distribution.
/// - `ST`: The steering function.
/// - `NN`: The nearest neighbors data structure.
/// - `F`: The floating point type.
pub struct RRT<S, M, GR, VC, SD, ST, NN, F>
where
    S: State,
    M: Motion<S>,
    GR: Region<S>,
    VC: ValidityChecker<S, M>,
    SD: SamplingDistribution<S>,
    ST: Steering,
    F: PartialOrd,
    NN: NearestNeighbors<S, F>,
{
    /// The goal region.
    goal_region: GR,
    /// The nodes in the tree.
    nodes: Vec<Node<S, M>>,
    /// Index of the solution node (None if no solution has been found).
    solution: Option<usize>,
    validity_checker: VC,
    sampling_distribution: SD,
    steering: ST,
    nearest_neighbors: NN,
    _float: std::marker::PhantomData<F>,
}

impl<S, M, GR, VC, SD, ST, NN, F> RRT<S, M, GR, VC, SD, ST, NN, F>
where
    S: State + Clone,
    M: Motion<S> + Clone,
    GR: Region<S>,
    VC: ValidityChecker<S, M>,
    SD: SamplingDistribution<S>,
    ST: Steering<State = S, Motion = M>,
    F: PartialOrd,
    NN: NearestNeighbors<S, F> + Default,
{
    /// Constructs a new RRT planner.
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
        goal_region: GR,
        validity_checker: VC,
        sampling_distribution: SD,
        steering: ST,
    ) -> Self {
        let mut rrt = Self {
            goal_region,
            solution: None,
            nodes: Vec::new(),
            validity_checker,
            sampling_distribution,
            steering,
            nearest_neighbors: NN::default(),
            _float: std::marker::PhantomData,
        };
        let root = Node::new(M::zero(start), None);
        rrt.add_node(root);
        rrt
    }

    /// Attempts to find a solution within a maximum number of iterations.
    ///
    /// Terminates and returns true when a solution is found. Otherwise, returns false.
    ///
    /// Parameters:
    /// - `max_iterations`: The maximum number of iterations.
    pub fn plan(&mut self, max_iterations: u32) -> bool {
        for _ in 0..max_iterations {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
    }

    /// Run a fixed number of iterations of the RRT algorithm. Does not terminate early if a solution is found.
    ///
    /// Returns true if the RRT found a solution.
    ///
    /// Parameters:
    /// - `iterations`: The number of iterations to run.
    pub fn run_iterations(&mut self, iterations: u32) -> bool {
        for _ in 0..iterations {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
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
            path.push(self.nodes[current_index].motion().clone());
            current_index = parent_index;
        }
        path.push(self.nodes[current_index].motion().clone());

        // Reverse the path so that it goes from the start to the goal.
        path.reverse();
        Some(path)
    }

    pub fn get_nearest_neighbors(&self) -> &NN {
        &self.nearest_neighbors
    }

    pub fn get_sampling_distribution(&self) -> &SD {
        &self.sampling_distribution
    }

    pub fn get_validity_checker(&self) -> &VC {
        &self.validity_checker
    }

    /// Returns the vector of nodes in the tree.
    pub fn get_tree(&self) -> &Vec<Node<S, M>> {
        &self.nodes
    }

    /// Expands the tree by one iteration.
    ///
    /// Each iteration of the RRT algorithm consists of the following steps:
    /// 1. Sample a point from the sampling distribution.
    /// 2. Find the nearest node in the tree to the sample point.
    /// 3. Steer the nearest node towards the sample point.
    /// 4. Add the new node to as a child of the nearest node if the edge is valid.
    /// 5. If the goal is reached, update the solution node.
    fn iteration(&mut self) {
        // Sample a point from the sampling distribution.
        let sample = self.sampling_distribution.sample();

        // Find the nearest node in the tree to the sample point.
        let nearest_node_index = self.nearest_neighbors.nearest_one(&sample).unwrap();
        let nearest_node = self.nodes[nearest_node_index].motion().state();

        // Steer the nearest node towards the sample point to get a new point.
        let new_motion = self.steering.steer(nearest_node, &sample);

        // If the new motion is invalid, return.
        if !self.validity_checker.is_motion_valid(&new_motion) {
            return;
        }

        // Add the new node to as a child of the nearest node.
        let new_node = Node::new(new_motion, Some(nearest_node_index));
        let new_node_index = self.add_node(new_node);

        // If the goal is reached, update the solution node.
        if self
            .goal_region
            .contains(self.nodes[new_node_index].state())
        {
            self.solution = Some(new_node_index);
        }
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<S, M>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.state().clone(), index);
        self.nodes.push(node);
        index
    }
}
