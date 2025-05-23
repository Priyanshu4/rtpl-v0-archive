use crate::base::distance::DistanceMetric;
use crate::real_vector::RealVectorState;
use num_traits::Float;
use serde::{Deserialize, Serialize};

/// Euclidean distance between two vectors.
#[derive(Default, Clone, Copy, Serialize, Deserialize)]
pub struct EuclideanDistance {}

impl<F: Float, const N: usize> DistanceMetric<RealVectorState<F, N>, F> for EuclideanDistance {
    fn distance(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> F {
        (a - b).norm()
    }
}
