use crate::base::motion::{Discretizable, Motion};
use crate::base::region::Region;
use crate::base::state::State;

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
    /// - `initial_state`: The initial state of the motion.
    /// - `motion`: The motion to check.
    ///
    /// Returns:
    /// Whether the motion is valid.
    fn is_motion_valid(&self, initial_state: &S, motion: &M) -> bool;
}

pub struct AlwaysValid {}

impl AlwaysValid {
    pub fn new() -> Self {
        Self {}
    }
}

impl<S: State, M: Motion<S>> ValidityChecker<S, M> for AlwaysValid {
    fn is_state_valid(&self, _state: &S) -> bool {
        true
    }

    fn is_motion_valid(&self, _initial_state: &S, _motion: &M) -> bool {
        true
    }
}

/// Checks if a state or motion is valid (i.e., not in collision) with a given region.
///
/// Motions are checked using discretization.
/// Discretization can be given in two ways:
/// - `discretization_steps`: The number of interior points to discretize the motion into, not counting endpoints.
/// - `discretization_resolution`: The minimum resolution of discretization.
/// Only one of the two should be provided.
///
/// Regardless of which one is used, the end state is always checked, but the initial state is not.
pub struct CollisionRegion<S: State> {
    region: Box<dyn Region<S>>,
    discretization: CollisionRegionMotionDiscretizationType,
}

#[derive(Clone, Copy, Debug)]
pub enum CollisionRegionMotionDiscretizationType {
    Steps(usize),
    Resolution(f64),
}

impl<S: State> CollisionRegion<S> {
    pub fn new(
        region: Box<dyn Region<S>>,
        discretization: CollisionRegionMotionDiscretizationType,
    ) -> Self {
        Self {
            region,
            discretization,
        }
    }
}

impl<S: State + Clone, M: Motion<S> + Discretizable<S>> ValidityChecker<S, M>
    for CollisionRegion<S>
{
    fn is_state_valid(&self, state: &S) -> bool {
        !self.region.contains(state)
    }

    fn is_motion_valid(&self, initial_state: &S, motion: &M) -> bool {
        let states: Vec<S> = match &self.discretization {
            CollisionRegionMotionDiscretizationType::Steps(discretization_steps) => {
                motion.discretize(initial_state, *discretization_steps)
            }
            CollisionRegionMotionDiscretizationType::Resolution(discretization_resolution) => {
                motion.discretize_with_resolution(initial_state, *discretization_resolution)
            }
        };
        let states = &states[1..];
        states.iter().all(|state| !self.region.contains(state))
    }
}
