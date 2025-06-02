pub trait State: Clone + Send + Sync + 'static {}

#[derive(Clone, Debug, PartialEq)]
pub struct RealVectorState {
    pub values: Vec<f64>,
}
impl State for RealVectorState {}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_real_vector_state_creation() {
        let state = RealVectorState {
            values: vec![1.0, 2.0],
        };
        assert_eq!(state.values, vec![1.0, 2.0]);
    }

    #[test]
    fn test_real_vector_state_clone() {
        let state1 = RealVectorState {
            values: vec![1.0, 2.0],
        };
        let state2 = state1.clone();
        assert_eq!(state1, state2);
    }
}
