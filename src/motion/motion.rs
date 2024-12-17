use crate::state::State;

/// A trait for motions in the state space of a system.
pub trait Motion<T: State> {
    /// Returns the state at the end of the motion.
    fn state(&self) -> &T;

    /// Create a new motion that is the zero motion from the given state.
    fn zero(state: T) -> Self;
}

pub trait MotionWithCost<T: State, F>: Motion<T> {
    fn state(&self) -> &T;
    fn cost(&self) -> F;
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
