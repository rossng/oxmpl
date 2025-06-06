use crate::base::state;
use std::{error, f64::consts::PI, fmt};

#[derive(Debug, PartialEq)]
pub enum StateSpaceError {
    /// The length of the provided bounds does not match the dimension.
    DimensionMismatch { expected: usize, found: usize },
    /// A lower bound is greater than or equal to its corresponding upper bound.
    InvalidBound { lower: f64, upper: f64 },
    /// A 0-dimensional space was requested without explicit (empty) bounds.
    ZeroDimensionUnbounded,
}
impl fmt::Display for StateSpaceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DimensionMismatch { expected, found } => write!(
                f,
                "provided bounds length ({}) does not match specified dimension ({}).",
                found, expected
            ),
            Self::InvalidBound { lower, upper } => {
                write!(
                    f,
                    "Lower bound {} is greater than upper bound {}.",
                    lower, upper
                )
            }
            Self::ZeroDimensionUnbounded => {
                write!(f, "Cannot create 0-dimensional unbounded space.")
            }
        }
    }
}
impl error::Error for StateSpaceError {}

pub trait StateSpace {
    /// StateType defines what is acceptable in current StateSpace
    type StateType: state::State;
    /// Find distance between current state1 and target state2. Depends on StateSpace.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64;
    /// Find state interpolated between `from` and `to` states given 0<=`t`<=1.
    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        state: &mut Self::StateType,
    );
    /// Modifies the given state to ensure it conforms to the space's defined bounds.
    fn enforce_bounds(&self, state: &mut Self::StateType);

    fn satisfies_bounds(&self, state: &Self::StateType) -> bool;
}

pub struct RealVectorStateSpace {
    /// n-Dimensionality of VectorStateSpace i.e. R^n.
    pub dimension: usize,
    pub bounds: Vec<(f64, f64)>,
}

impl RealVectorStateSpace {
    pub fn new(
        dimension: usize,
        bounds_option: Option<Vec<(f64, f64)>>,
    ) -> Result<Self, StateSpaceError> {
        let bounds = match bounds_option {
            Some(explicit_bounds) => {
                if explicit_bounds.len() != dimension {
                    return Err(StateSpaceError::DimensionMismatch {
                        expected: dimension,
                        found: explicit_bounds.len(),
                    });
                }
                for bound in &explicit_bounds {
                    if bound.0 >= bound.1 {
                        return Err(StateSpaceError::InvalidBound {
                            lower: bound.0,
                            upper: bound.1,
                        });
                    }
                }
                explicit_bounds
            }
            None => {
                if dimension == 0 {
                    return Err(StateSpaceError::ZeroDimensionUnbounded);
                }
                vec![(f64::NEG_INFINITY, f64::INFINITY); dimension]
            }
        };

        Ok(Self { dimension, bounds })
    }
}

impl StateSpace for RealVectorStateSpace {
    type StateType = state::RealVectorState;

    /// Find distance between current state1 and target state2. Depends on StateSpace.
    /// In RealVectorStateSpace, this refers to the L2-norm.
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
        assert_eq!(
            from.values.len(),
            self.dimension,
            "From-state has incorrect dimension."
        );
        assert_eq!(
            to.values.len(),
            self.dimension,
            "To-state has incorrect dimension."
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

    fn enforce_bounds(&self, state: &mut Self::StateType) {
        if state.values.len() != self.dimension {
            assert_eq!(
                state.values.len(),
                self.dimension,
                "State and space dimension mismatch when enforcing bounds."
            );
        }
        for (i, value) in state.values.iter_mut().enumerate() {
            if i < self.bounds.len() {
                let (lower, upper) = self.bounds[i];
                *value = value.clamp(lower, upper);
            }
        }
    }

    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        if state.values.len() != self.dimension {
            assert_eq!(
                state.values.len(),
                self.dimension,
                "State and space dimension mismatch when checking bound satisfaction."
            );
        }
        for i in 0..self.dimension {
            let (lower, upper) = self.bounds[i];
            if state.values[i] - f64::EPSILON > upper || state.values[i] + f64::EPSILON < lower {
                return false;
            }
        }
        true
    }
}

pub struct SO2StateSpace {
    // pub bounds: Vec<(f64, f64)>,
}

impl StateSpace for SO2StateSpace {
    type StateType = state::SO2State;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let mut diff = state1.value - state2.value;
        diff = (diff + PI).rem_euclid(2.0 * PI) - PI;
        diff.abs()
    }

    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        out_state: &mut Self::StateType,
    ) {
        let mut diff_to_from = to.clone().normalise().value - from.clone().normalise().value;
        if diff_to_from > PI {
            diff_to_from -= 2.0 * PI;
        } else if diff_to_from < -PI {
            diff_to_from += 2.0 * PI;
        }
        out_state.value = from.value + diff_to_from * t;
        out_state.value = out_state.normalise().value;
    }

    // TODO
    fn enforce_bounds(&self, _state: &mut Self::StateType) {}

    // TODO
    fn satisfies_bounds(&self, _state: &Self::StateType) -> bool {
        false
    }
}

// #[cfg(test)]
// #[path = "unittests/rvss_tests.rs"]
// mod rvss_tests;

// #[cfg(test)
// #[path = "unittests/so2ss_tests.rs"]
// mod so2ss_tests;
