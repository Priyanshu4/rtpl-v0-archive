use crate::distance::DistanceMetric;
use crate::distance::KdTreeDistanceMetric;
use crate::state::RealVectorState;
use kiddo::float::kdtree::Axis;
use num_traits::Float;

/// Euclidean distance between two vectors.
#[derive(Default)]
pub struct EuclideanDistance {}

impl<F: Float, const N: usize> DistanceMetric<RealVectorState<F, N>, F> for EuclideanDistance {
    fn distance(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> F {
        (a - b).norm()
    }
}

#[derive(Default)]
pub struct EuclideanDistanceSquared {}

impl<F: Float, const N: usize> DistanceMetric<RealVectorState<F, N>, F>
    for EuclideanDistanceSquared
{
    fn distance(&self, a: &RealVectorState<F, N>, b: &RealVectorState<F, N>) -> F {
        (a - b).norm_squared()
    }
}

impl<A: Axis + Float, const K: usize> KdTreeDistanceMetric<A, K> for EuclideanDistanceSquared {
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
