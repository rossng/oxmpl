use crate::base::{
    goal::Goal, problem_definition::ProblemDefinition, space::StateSpace, state::State,
    validity::StateValidityChecker,
};
use std::{sync::Arc, time::Duration};

// Define Path type, e.g., Vec<S> or a dedicated struct
pub struct Path<S: State>(pub Vec<S>);

pub enum PlanningError {
    // Should be OxMPLError or similar
    Timeout,
    NoSolutionFound,
    // Other error variants
}

pub trait Planner<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    fn setup(
        &mut self,
        problem_def: Arc<ProblemDefinition<S, SP, G>>,
        validity_checker: Arc<dyn StateValidityChecker<S> + Send + Sync>,
    );
    fn solve(&self, timeout: Duration) -> Result<Path<S>, PlanningError>; // Use OxMPLError here
}
