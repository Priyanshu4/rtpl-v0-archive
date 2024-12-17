use crate::state::State;

/// A trait for a nearest neighbor data structure that supports nearest neighbors and radius queries.
/// Stores states and a usize index along with them.
pub trait NearestNeighbors<S: State, F: PartialOrd> {
    /// Adds a state to the data structure.
    ///
    /// Parameters:
    /// - `state`: The state to add.
    /// - `item`: The index of the state.
    fn add(&mut self, state: S, item: usize);

    /// Gets the nearest neighbor to the given state.
    ///
    /// Parameters:
    /// - `state`: The state to find the nearest neighbor to.
    ///
    /// Returns:
    /// The item/index of the nearest neighbor, if any.
    fn nearest_one(&self, state: &S) -> Option<usize> {
        let nearest_vec = self.nearest_k(state, 1);
        if nearest_vec.is_empty() {
            None
        } else {
            Some(nearest_vec[0])
        }
    }

    /// Gets the k nearest neighbors to the given state.
    ///
    /// Parameters:
    /// - `state`: The state to find the nearest neighbors to.
    /// - `k`: The number of neighbors to find.
    ///
    /// Returns:
    /// The items/indices of the k nearest neighbors.
    fn nearest_k(&self, state: &S, k: usize) -> Vec<usize>;

    /// Gets all states within a given radius of the given state.
    ///
    /// Parameters:
    /// - `state`: The state to find the neighbors of.
    /// - `radius`: The radius within which to find neighbors.
    ///
    /// Returns:
    /// The items/indices of the states within the radius.
    fn within_radius(&self, state: &S, radius: F) -> Vec<usize>;
}
