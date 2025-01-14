use crate::base::sampling::SamplingDistribution;
use crate::real_vector::RealVectorState;
use num_traits::Float;
use rand::distributions::{uniform::SampleUniform, Bernoulli, Distribution, Uniform};

/// A uniform distribution for float vectors
/// Each dimension has a range of values.
pub struct UniformDistribution<F: Float + SampleUniform, const N: usize> {
    ranges: [(F, F); N],
    uniforms: [Uniform<F>; N],
    rng: rand::rngs::ThreadRng,
}

impl<F: Float + SampleUniform, const N: usize> UniformDistribution<F, N> {
    /// Constructs a new uniform distribution.
    /// Parameters:
    /// - `ranges`: The ranges for each dimension.
    /// Returns:
    /// The uniform distribution.
    pub fn new(ranges: [(F, F); N]) -> Self {
        let uniforms: [Uniform<F>; N] =
            std::array::from_fn(|i| Uniform::new_inclusive(ranges[i].0, ranges[i].1));
        Self {
            ranges,
            uniforms,
            rng: rand::thread_rng(),
        }
    }

    pub fn ranges(&self) -> [(F, F); N] {
        self.ranges
    }
}

impl<F: Float + SampleUniform, const N: usize> SamplingDistribution<RealVectorState<F, N>>
    for UniformDistribution<F, N>
{
    fn sample(&mut self) -> RealVectorState<F, N> {
        let values: [F; N] = std::array::from_fn(|i| self.uniforms[i].sample(&mut self.rng));
        RealVectorState::new(values)
    }
}

impl<F: Float + SampleUniform + Clone, const N: usize> Clone for UniformDistribution<F, N> {
    fn clone(&self) -> Self {
        UniformDistribution::new(self.ranges)
    }
}

/// A uniform distribution that occasionally samples the goal with a given goal_bias probability.
#[derive(Clone)]
pub struct GoalBiasedUniformDistribution<F: Float + SampleUniform, const N: usize> {
    uniform: UniformDistribution<F, N>, // Uniform distribution for sampling RealVectorStates.
    bernoulli: Bernoulli,               // Bernoulli distribution for goal bias.
    goal: RealVectorState<F, N>,        // The goal state.
    rng: rand::rngs::ThreadRng,
}

impl<F: Float + SampleUniform, const N: usize> GoalBiasedUniformDistribution<F, N> {
    /// Constructs a new goal-biased uniform distribution.
    /// Parameters:
    /// - `ranges`: The ranges for each dimension.
    /// - `goal`: The goal RealVectorState.
    /// - `goal_bias`: The probability of sampling the goal.
    /// Returns:
    /// The goal-biased uniform distribution.
    pub fn new(
        ranges: [(F, F); N],
        goal: RealVectorState<F, N>,
        goal_bias: f64,
    ) -> Result<Self, &'static str> {
        if goal_bias < 0.0 || goal_bias > 1.0 {
            return Err("goal_bias must be in the range [0, 1]");
        }
        Ok(Self {
            uniform: UniformDistribution::new(ranges),
            bernoulli: Bernoulli::new(goal_bias).unwrap(),
            goal,
            rng: rand::thread_rng(),
        })
    }

    pub fn set_goal_bias(&mut self, goal_bias: f64) -> Result<(), &'static str> {
        if goal_bias < 0.0 || goal_bias > 1.0 {
            return Err("goal_bias must be in the range [0, 1]");
        }
        self.bernoulli = Bernoulli::new(goal_bias).unwrap();
        Ok(())
    }
}

impl<F: Float + SampleUniform, const N: usize> SamplingDistribution<RealVectorState<F, N>>
    for GoalBiasedUniformDistribution<F, N>
{
    fn sample(&mut self) -> RealVectorState<F, N> {
        if self.bernoulli.sample(&mut self.rng) {
            self.goal.clone()
        } else {
            self.uniform.sample()
        }
    }
}
