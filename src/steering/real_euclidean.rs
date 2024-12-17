use crate::motion::RealEuclideanMotion;
use crate::motion::RealEuclideanMotionWithCost;
use crate::state::RealVectorState;
use crate::steering::Steering;
use num_traits::Float;

/// A steering strategy that moves the robot in a straight line towards the goal.
pub struct EuclideanSteering<F, const N: usize> {
    range: F,
}

impl<F: Float, const N: usize> Steering for EuclideanSteering<F, N> {
    type State = RealVectorState<F, N>;
    type Motion = RealEuclideanMotion<F, N>;

    fn steer(&self, from: &Self::State, to: &Self::State) -> Self::Motion {
        let direction = to - from;
        let distance = direction.norm();
        let ratio = if distance <= self.range {
            F::one()
        } else {
            self.range / distance
        };
        let state = from + &(&direction * ratio);
        RealEuclideanMotion::new(state)
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

/// A steering strategy that moves the robot in a straight line towards the goal.
/// The cost of the motion is the distance moved.
pub struct EuclideanSteeringWithCost<F, const N: usize> {
    range: F,
}

impl<F: Float, const N: usize> Steering for EuclideanSteeringWithCost<F, N> {
    type State = RealVectorState<F, N>;
    type Motion = RealEuclideanMotionWithCost<F, N>;

    fn steer(&self, from: &Self::State, to: &Self::State) -> Self::Motion {
        let direction = to - from;
        let distance = direction.norm();
        let ratio = if distance <= self.range {
            F::one()
        } else {
            self.range / distance
        };
        let state = from + &(&direction * ratio);
        RealEuclideanMotionWithCost::new(state, distance)
    }
}

impl<F: Float, const N: usize> EuclideanSteeringWithCost<F, N> {
    /// Constructs a new Euclidean steering function which moves the robot in a straight line.
    /// Parameters:
    /// - `range`: The maximum distance the robot can move in one step.
    /// Returns:
    /// The Euclidean steering strategy.
    pub fn new(range: F) -> Self {
        Self { range }
    }
}
