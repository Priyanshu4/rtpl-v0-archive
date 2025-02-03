use crate::base::motion::Discretizable;
use crate::base::motion::HasCost;
use crate::base::motion::Motion;
use crate::real_vector::RealVectorState;
use num_traits::float::Float;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct EuclideanMotion<F: Float, const N: usize> {
    state: RealVectorState<F, N>,
    cost: F,
}

impl<F: Float, const N: usize> EuclideanMotion<F, N> {
    pub fn new(state: RealVectorState<F, N>, cost: F) -> Self {
        Self { state, cost }
    }
}

impl<F: Float, const N: usize> Motion<RealVectorState<F, N>> for EuclideanMotion<F, N> {
    fn state(&self) -> &RealVectorState<F, N> {
        &self.state
    }

    fn zero(state: RealVectorState<F, N>) -> Self {
        Self::new(state, F::zero())
    }
}

impl<F: Float, const N: usize> HasCost<F> for EuclideanMotion<F, N> {
    fn cost(&self) -> F {
        self.cost
    }
}

impl<F: Float, const N: usize> Discretizable<RealVectorState<F, N>> for EuclideanMotion<F, N> {
    fn discretize(
        &self,
        initial_state: &RealVectorState<F, N>,
        num_steps: usize,
    ) -> Vec<RealVectorState<F, N>> {
        let direction = self.state - *initial_state;
        let n = F::from(num_steps).expect("usize to F conversion failed");
        let step = &direction / n;
        (0..num_steps)
            .map(|i| initial_state + &(&step * F::from(i).expect("usize to F conversion failed")))
            .collect()
    }
}
