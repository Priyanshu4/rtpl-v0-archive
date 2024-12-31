use crate::motion::RealEuclideanMotion;
use crate::state::RealVectorState;
use crate::steering::Steering;
use num_traits::Float;

/// A steering strategy that moves the robot in a straight line towards the goal.
pub struct EuclideanSteering<F, const N: usize> {
    range: F, // The maximum distance for steer towards.
}

impl<F: Float, const N: usize> Steering<RealVectorState<F, N>, RealEuclideanMotion<F, N>>
    for EuclideanSteering<F, N>
{
    fn steer_towards(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> RealEuclideanMotion<F, N> {
        let direction = to - from;
        let distance = direction.norm();
        let ratio = if distance <= self.range {
            F::one()
        } else {
            self.range / distance
        };
        let state = from + &(&direction * ratio);
        RealEuclideanMotion::new(state, distance)
    }

    fn steer_exact(
        &self,
        from: &RealVectorState<F, N>,
        to: &RealVectorState<F, N>,
    ) -> Option<RealEuclideanMotion<F, N>> {
        let direction = to - from;
        let distance = direction.norm();
        Some(RealEuclideanMotion::new(to.clone(), distance))
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
