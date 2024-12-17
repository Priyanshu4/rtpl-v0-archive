use crate::region::Region;
use crate::state::RealVectorState;
use num_traits::Float;

pub struct EuclideanSphere<F: Float, const N: usize> {
    center: RealVectorState<F, N>,
    radius: F,
}

impl<F: Float, const N: usize> EuclideanSphere<F, N> {
    pub fn new(center: RealVectorState<F, N>, radius: F) -> Self {
        Self { center, radius }
    }
}

impl<F: Float, const N: usize> Region<RealVectorState<F, N>> for EuclideanSphere<F, N> {
    fn contains(&self, state: &RealVectorState<F, N>) -> bool {
        (state - &self.center).norm_squared() <= self.radius * self.radius
    }
}
