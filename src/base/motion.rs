use crate::base::state::State;

/// A trait for motions in the state space of a system.
pub trait Motion<T: State> {
    /// Returns the state at the end of the motion.
    fn state(&self) -> &T;

    /// Create a new motion that is the zero motion from the given state.
    fn zero(state: T) -> Self;
}

pub struct BasicMotion<T: State> {
    state: T,
}

impl<T: State> BasicMotion<T> {
    pub fn new(state: T) -> Self {
        Self { state }
    }
}

impl<T: State> Motion<T> for BasicMotion<T> {
    fn state(&self) -> &T {
        &self.state
    }

    fn zero(state: T) -> Self {
        Self::new(state)
    }
}

pub trait HasCost<F> {
    fn cost(&self) -> F;
}

pub trait Discretizable<T> {
    /// Discretizes the motion from the initial state to the final state.
    /// Both initial state and final state are included, in addition to the specified interior points.
    fn discretize(&self, initial_state: &T, num_interior_points: usize) -> Vec<T>;

    /// Discretizes the motion from the initial state to the final state with a given minimum resolution.
    /// Both initial state and final state are included.
    fn discretize_with_resolution(&self, initial_state: &T, resolution: f64) -> Vec<T>;
}
