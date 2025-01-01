use crate::planners::rrt::RRT as RRTBase;
use crate::planners::rrt_star::RRTstar as RRTstarBase;
use crate::real_vector::euclidean::{EuclideanKdTree, EuclideanMotion};
use crate::real_vector::RealVectorState;

pub type RRT<F, const N: usize> =
    RRTBase<RealVectorState<F, N>, EuclideanMotion<F, N>, F, EuclideanKdTree<F, N>>;

pub type RRTstar<F, const N: usize> =
    RRTstarBase<RealVectorState<F, N>, EuclideanMotion<F, N>, F, EuclideanKdTree<F, N>>;
