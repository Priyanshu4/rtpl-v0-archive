use crate::distance::DistanceMetric;
use crate::state::State;

/// A region is a set of states in the state space of a system.
pub trait Region<T: State> {
    /// Returns true if the state is inside the region.
    fn contains(&self, state: &T) -> bool;
}

pub struct SingleStateRegion<T: State + PartialEq> {
    state: T,
}

impl<T: State + PartialEq> SingleStateRegion<T> {
    pub fn new(state: T) -> Self {
        Self { state }
    }
}

impl<T: State + PartialEq> Region<T> for SingleStateRegion<T> {
    fn contains(&self, state: &T) -> bool {
        *state == self.state
    }
}

pub struct BallRegion<T, R, D> {
    center: T,
    radius: R,
    distance_metric: D,
}

impl<T, R, D: Default> BallRegion<T, R, D> {
    pub fn new(center: T, radius: R) -> Self {
        Self {
            center,
            radius,
            distance_metric: D::default(),
        }
    }
}

impl<T, R, D> Region<T> for BallRegion<T, R, D>
where
    T: State,
    R: PartialOrd,
    D: DistanceMetric<T, R>,
{
    fn contains(&self, state: &T) -> bool {
        self.distance_metric.distance(state, &self.center) <= self.radius
    }
}

pub struct UnionRegion<T, R1, R2> {
    region1: R1,
    region2: R2,
    _phantom: std::marker::PhantomData<T>,
}

impl<T, R1, R2> UnionRegion<T, R1, R2> {
    pub fn new(region1: R1, region2: R2) -> Self {
        Self {
            region1,
            region2,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<T, R1, R2> Region<T> for UnionRegion<T, R1, R2>
where
    T: State,
    R1: Region<T>,
    R2: Region<T>,
{
    fn contains(&self, state: &T) -> bool {
        self.region1.contains(state) || self.region2.contains(state)
    }
}

pub struct IntersectionRegion<T, R1, R2> {
    region1: R1,
    region2: R2,
    _phantom: std::marker::PhantomData<T>,
}

impl<T, R1, R2> IntersectionRegion<T, R1, R2> {
    pub fn new(region1: R1, region2: R2) -> Self {
        Self {
            region1,
            region2,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<T, R1, R2> Region<T> for IntersectionRegion<T, R1, R2>
where
    T: State,
    R1: Region<T>,
    R2: Region<T>,
{
    fn contains(&self, state: &T) -> bool {
        self.region1.contains(state) && self.region2.contains(state)
    }
}

pub struct InvertedRegion<T, R> {
    region: R,
    _phantom: std::marker::PhantomData<T>,
}

impl<T, R> InvertedRegion<T, R> {
    pub fn new(region: R) -> Self {
        Self {
            region,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<T, R> Region<T> for InvertedRegion<T, R>
where
    T: State,
    R: Region<T>,
{
    fn contains(&self, state: &T) -> bool {
        !self.region.contains(state)
    }
}

pub struct EmptyRegion<T> {
    _phantom: std::marker::PhantomData<T>,
}

impl<T> EmptyRegion<T> {
    pub fn new() -> Self {
        Self {
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<T> Region<T> for EmptyRegion<T>
where
    T: State,
{
    fn contains(&self, _state: &T) -> bool {
        false
    }
}
