use crate::base::steering::Steering;
use crate::real_vector::euclidean::EuclideanMotion;
use crate::real_vector::RealVectorState;
use num_traits::Float;

/// A steering strategy that moves the robot in a straight line towards the goal.
#[derive(Clone, Copy, Debug)]
pub struct EuclideanSteering<F, const N: usize> {
    range: F, // The maximum distance for steer towards.
}

impl<F: Float, const N: usize> Steering<RealVectorState<F, N>, EuclideanMotion<F, N>>
    for EuclideanSteering<F, N>
{
    fn steer_towards(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> EuclideanMotion<F, N> {
        let direction = to - from;
        let distance = direction.norm();
        if distance <= self.range {
            EuclideanMotion::new(to.clone(), distance)
        } else {
            EuclideanMotion::new(from + &(&direction / distance * self.range), self.range)
        }
    }

    fn steer_exact(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> Option<EuclideanMotion<F, N>> {
        let direction = to - from;
        let distance = direction.norm();
        Some(EuclideanMotion::new(to.clone(), distance))
    }
}

impl<F: Float, const N: usize> EuclideanSteering<F, N> {
    /// Constructs a new Euclidean steering function which moves the robot in a straight line.
    /// Parameters:
    /// - `range`: The maximum distance the robot can move in one step.
    /// Returns:
    /// The Euclidean steering strategy.
    pub fn new(range: F) -> Self {
        Self { range }
    }
}
