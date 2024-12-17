use crate::distance::KdTreeDistanceMetric;
use crate::nearest_neighbors::NearestNeighbors;
use crate::state::RealVectorState;
use kiddo::float::kdtree::Axis;
use kiddo::float::kdtree::KdTree;
use num_traits::Float;

#[derive(Default)]
pub struct KdTreeNearestNeighbors<F: Float + Axis, const N: usize, D: KdTreeDistanceMetric<F, N>> {
    kdtree: KdTree<F, usize, N, 32, u32>,
    phantom: std::marker::PhantomData<D>,
}

impl<F: Float + Axis, const N: usize, D: KdTreeDistanceMetric<F, N>>
    NearestNeighbors<RealVectorState<F, N>, F> for KdTreeNearestNeighbors<F, N, D>
{
    fn add(&mut self, state: RealVectorState<F, N>, item: usize) {
        self.kdtree.add(state.values(), item);
    }

    fn nearest_one(&self, state: &RealVectorState<F, N>) -> Option<usize> {
        let neighbor = self.kdtree.nearest_one::<D>(state.values());
        Some(neighbor.item)
    }

    fn nearest_k(&self, state: &RealVectorState<F, N>, k: usize) -> Vec<usize> {
        self.kdtree
            .nearest_n::<D>(state.values(), k)
            .iter()
            .map(|n| n.item)
            .collect()
    }

    fn within_radius(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        self.kdtree
            .within::<D>(state.values(), radius)
            .iter()
            .map(|n| n.item)
            .collect()
    }
}
