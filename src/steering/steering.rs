use crate::motion::Motion;
use crate::state::State;

/// A trait for steering the robot from one point to another.
/// Allows considering the robot's kinematics and dynamics in the RRT.
pub trait Steering<S: State, M: Motion<S>> {
    /// Steers the robot from one state towards another.
    /// Parameters:
    /// - `from`: The start state.
    /// - `to`: The state to steer towards.
    ///
    /// Returns:
    /// The  motion that steers the robot towards the goal state.
    fn steer_towards(&self, from: &S, to: &S) -> M;

    /// Steers the robot from one state exactly to another.
    /// Parameters:
    /// - `from`: The start state.
    /// - `to`: The state to steer towards.
    ///
    /// Returns:
    /// The motion that steers the robot exactly to the goal state.
    fn steer_exact(&self, from: &S, to: &S) -> Option<M>;
}
