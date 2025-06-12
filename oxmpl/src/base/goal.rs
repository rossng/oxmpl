use crate::base::state::State;

pub trait Goal<S: State> {
    fn is_satisfied(&self, state: &S) -> bool;
    // Optional: distance_goal, sample_goal, etc.
}
