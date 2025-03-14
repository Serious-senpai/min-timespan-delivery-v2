use std::rc::Rc;

use crate::solutions::Solution;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Neighborhood {
    Move10,
    Move11,
    Move20,
    Move21,
    Move22,
    TwoOpt,
}

impl ToString for Neighborhood {
    fn to_string(&self) -> String {
        match self {
            Neighborhood::Move10 => "Move (1, 0)".to_string(),
            Neighborhood::Move11 => "Move (1, 1)".to_string(),
            Neighborhood::Move20 => "Move (2, 0)".to_string(),
            Neighborhood::Move21 => "Move (2, 1)".to_string(),
            Neighborhood::Move22 => "Move (2, 2)".to_string(),
            Neighborhood::TwoOpt => "2-opt".to_string(),
        }
    }
}

impl Neighborhood {
    pub fn inter_route(&self, solution: Rc<Solution>) -> Rc<Solution> {
        solution
    }
}
