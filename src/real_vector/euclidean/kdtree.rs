use crate::base::NearestNeighbors;
use crate::real_vector::RealVectorState;
use kiddo::float::distance::SquaredEuclidean;
use kiddo::float::kdtree::Axis;
use kiddo::float::kdtree::KdTree;
use num_traits::Float;

#[derive(Default)]
pub struct EuclideanKdTree<F: Float + Axis, const N: usize> {
    kdtree: KdTree<F, usize, N, 32, u32>,
}

impl<F: Float + Axis, const N: usize> NearestNeighbors<RealVectorState<F, N>, F>
    for EuclideanKdTree<F, N>
{
    fn add(&mut self, state: RealVectorState<F, N>, item: usize) {
        self.kdtree.add(state.values(), item);
    }

    fn nearest_one(&self, state: &RealVectorState<F, N>) -> Option<usize> {
        let neighbor = self.kdtree.nearest_one::<SquaredEuclidean>(state.values());
        Some(neighbor.item)
    }

    fn nearest_k(&self, state: &RealVectorState<F, N>, k: usize) -> Vec<usize> {
        self.kdtree
            .nearest_n::<SquaredEuclidean>(state.values(), k)
            .iter()
            .map(|n| n.item)
            .collect()
    }

    fn within_radius(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        self.kdtree
            .within_unsorted::<SquaredEuclidean>(state.values(), radius * radius)
            .iter()
            .map(|n| n.item)
            .collect()
    }

    fn within_radius_sorted(&self, state: &RealVectorState<F, N>, radius: F) -> Vec<usize> {
        self.kdtree
            .within::<SquaredEuclidean>(state.values(), radius * radius)
            .iter()
            .map(|n| n.item)
            .collect()
    }

    fn clear(&mut self) {
        self.kdtree = KdTree::new();
    }
}
