use std::{error, fmt};

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
                "provided bounds length ({found}) does not match specified dimension ({expected})."
            ),
            Self::InvalidBound { lower, upper } => {
                write!(
                    f,
                    "Lower bound {lower} is greater than upper bound {upper}."
                )
            }
            Self::ZeroDimensionUnbounded => {
                write!(f, "Cannot create 0-dimensional unbounded space.")
            }
        }
    }
}
impl error::Error for StateSpaceError {}

#[derive(Debug, PartialEq)]
pub enum StateSamplingError {
    UnboundedDimension { dimension_index: usize },
}

impl fmt::Display for StateSamplingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StateSamplingError::UnboundedDimension { dimension_index } => {
                write!(
                    f,
                    "Cannot sample uniformly because dimension {dimension_index} is unbounded."
                )
            }
        }
    }
}

impl error::Error for StateSamplingError {}
