use crate::base::state;
use std::{error, fmt};

pub use crate::base::spaces::real_vector_state_space::RealVectorStateSpace;
pub use crate::base::spaces::so2_state_space::SO2StateSpace;

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

// #[cfg(test)]
// #[path = "unittests/rvss_tests.rs"]
// mod rvss_tests;

// #[cfg(test)
// #[path = "unittests/so2ss_tests.rs"]
// mod so2ss_tests;
