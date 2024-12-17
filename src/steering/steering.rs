use crate::motion::Motion;
use crate::state::State;

/// A trait for steering the robot from one point to another.
/// Allows considering the robot's kinematics and dynamics in the RRT.
pub trait Steering {
    type State: State;
    type Motion: Motion<Self::State>;

    /// Steers the robot from one state towards another.
    /// Parameters:
    /// - `from`: The start state.
    /// - `to`: The state to steer towards.
    ///
    /// Returns:
    /// The  motion that steers the robot towards the goal state.
    fn steer(&self, from: &Self::State, to: &Self::State) -> Self::Motion;
}
