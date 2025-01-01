/// Represents a state in the state space of a system.
pub trait State {
    fn dimension(&self) -> usize;
}
