pub type Sphere<F, const N: usize> = crate::base::region::BallRegion<
    crate::real_vector::RealVectorState<F, N>,
    F,
    crate::real_vector::euclidean::EuclideanDistance,
>;
