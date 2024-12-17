use crate::distance::DistanceMetric;
use crate::nearest_neighbors::NearestNeighbors;
use crate::state::State;

/// A nearest neighbor data structure that uses a linear search to find the nearest neighbors.
/// This is useful for small datasets.
pub struct LinearNearestNeighbors<S: State, F: PartialOrd, D: DistanceMetric<S, F>> {
    states: Vec<(S, usize)>,
    distance_metric: D,
    phantom: std::marker::PhantomData<F>,
}

impl<S: State, F: PartialOrd, D: DistanceMetric<S, F> + Default> Default
    for LinearNearestNeighbors<S, F, D>
{
    fn default() -> Self {
        LinearNearestNeighbors {
            states: Vec::new(),
            distance_metric: D::default(),
            phantom: std::marker::PhantomData,
        }
    }
}

impl<S: State, F: PartialOrd, D: DistanceMetric<S, F>> NearestNeighbors<S, F>
    for LinearNearestNeighbors<S, F, D>
{
    fn add(&mut self, state: S, item: usize) {
        self.states.push((state, item));
    }

    fn nearest_one(&self, state: &S) -> Option<usize> {
        self.states
            .iter()
            .map(|(p, i)| (self.distance_metric.distance(&p, &state), *i))
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
            .map(|(_, i)| i)
    }

    fn nearest_k(&self, state: &S, k: usize) -> Vec<usize> {
        let mut nearest = self
            .states
            .iter()
            .map(|(p, i)| (self.distance_metric.distance(&p, &state), *i))
            .collect::<Vec<_>>();
        nearest.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        nearest.into_iter().take(k).map(|(_, i)| i).collect()
    }

    fn within_radius(&self, state: &S, radius: F) -> Vec<usize> {
        self.states
            .iter()
            .filter(|(p, _)| self.distance_metric.distance(&p, &state) <= radius)
            .map(|(_, i)| *i)
            .collect()
    }
}
