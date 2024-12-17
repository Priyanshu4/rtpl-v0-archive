use crate::motion::Motion;
use crate::state::State;

/// Checks if a state or motion is valid (i.e., not in collision).
pub trait ValidityChecker<S: State, M: Motion<S>> {
    /// Checks if a state is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `state`: The state to check.
    ///
    /// Returns:
    /// Whether the state is valid.
    fn is_state_valid(&self, state: &S) -> bool;

    /// Checks if an motion is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `motion`: The motion to check.
    ///
    /// Returns:
    /// Whether the motion is valid.
    fn is_motion_valid(&self, motion: &M) -> bool;
}
