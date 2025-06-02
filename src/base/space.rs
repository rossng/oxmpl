use crate::base::state;
pub trait StateSpace {
    type StateType: state::State;
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64;
    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        state: &mut Self::StateType,
    );
}

pub struct RealVectorStateSpace {
    pub dimension: usize,
}

impl StateSpace for RealVectorStateSpace {
    type StateType = state::RealVectorState;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        assert_eq!(
            state1.values.len(),
            self.dimension,
            "State1 has incorrect dimension for this space."
        );
        assert_eq!(
            state2.values.len(),
            self.dimension,
            "State2 has incorrect dimension for this space."
        );
        state1
            .values
            .iter()
            .zip(state2.values.iter())
            .map(|(v1, v2)| (v1 - v2).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        out_state: &mut Self::StateType,
    ) {
        assert_eq!(from.values.len(), self.dimension, "From-state has incorrect dimension.");
        assert_eq!(to.values.len(), self.dimension, "To-state has incorrect dimension."
        );
        assert_eq!(
            out_state.values.len(),
            self.dimension,
            "Out-state has incorrect dimension or not properly initialized."
        );
        for i in 0..from.values.len() {
            out_state.values[i] = from.values[i] + (to.values[i] - from.values[i]) * t;
        }
    }
}


#[cfg(test)]
#[path = "unittests/rvss_tests.rs"]
mod rvss_tests;
