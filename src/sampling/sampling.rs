use crate::state::State;

/// A trait for sampling distributions.
pub trait SamplingDistribution<T: State> {
    /// Samples a point from the distribution.
    fn sample(&mut self) -> T;
}
