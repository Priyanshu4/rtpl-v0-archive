use crate::motion::Motion;
use crate::motion::MotionWithCost;
use crate::state::RealVectorState;
use num_traits::float::Float;

pub struct RealEuclideanMotion<F: Float, const N: usize> {
    state: RealVectorState<F, N>,
}

impl<F: Float, const N: usize> RealEuclideanMotion<F, N> {
    pub fn new(state: RealVectorState<F, N>) -> Self {
        Self { state }
    }
}

impl<F: Float, const N: usize> Motion<RealVectorState<F, N>> for RealEuclideanMotion<F, N> {
    fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    fn zero(state: RealVectorState<F, N>) -> Self {
        Self::new(state)
    }
}

pub struct RealEuclideanMotionWithCost<F: Float, const N: usize> {
    state: RealVectorState<F, N>,
    cost: F,
}

impl<F: Float, const N: usize> RealEuclideanMotionWithCost<F, N> {
    pub fn new(state: RealVectorState<F, N>, cost: F) -> Self {
        Self { state, cost }
    }
}

impl<F: Float, const N: usize> Motion<RealVectorState<F, N>> for RealEuclideanMotionWithCost<F, N> {
    fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    fn zero(state: RealVectorState<F, N>) -> Self {
        Self::new(state, F::zero())
    }
}

impl<F: Float, const N: usize> MotionWithCost<RealVectorState<F, N>, F>
    for RealEuclideanMotionWithCost<F, N>
{
    fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    fn cost(&self) -> F {
        self.cost
    }
}
