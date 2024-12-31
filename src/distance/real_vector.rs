use crate::distance::DistanceMetric;
use crate::distance::KdTreeDistanceMetric;
use crate::state::RealVectorState;
use kiddo::float::kdtree::Axis;
use num_traits::Float;

/// Euclidean distance between two vectors.
#[derive(Default, Clone, Copy)]
pub struct Euclidean {}

impl<F: Float, const N: usize> DistanceMetric<RealVectorState<F, N>, F> for Euclidean {
    fn distance(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> F {
        (a - b).norm()
    }
}

#[derive(Default)]
pub struct SquaredEuclidean {}

impl<F: Float, const N: usize> DistanceMetric<RealVectorState<F, N>, F> for SquaredEuclidean {
    fn distance(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> F {
        (a - b).norm_squared()
    }
}

impl<A: Axis + Float, const K: usize> KdTreeDistanceMetric<A, K> for SquaredEuclidean {
    fn dist(a: &[A; K], b: &[A; K]) -> A {
        let mut sum = A::zero();
        for i in 0..K {
            sum = sum + (a[i] - b[i]) * (a[i] - b[i]);
        }
        sum
    }

    fn dist1(a: A, b: A) -> A {
        a - b
    }
}
