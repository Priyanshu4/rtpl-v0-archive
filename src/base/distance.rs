use crate::base::state::State;

/// A trait for distance metrics.
pub trait DistanceMetric<T: State, R> {
    /// Returns the distance between two states.
    fn distance(&self, a: &T, b: &T) -> R;
}
