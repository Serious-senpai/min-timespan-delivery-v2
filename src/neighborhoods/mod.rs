use std::rc::Rc;

use crate::config::CONFIG;
use crate::routes::Route;
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
    pub fn inter_route(
        &self,
        solution: &Solution,
        tabu_list: &Vec<Vec<usize>>,
        aspiration_cost: f64,
    ) -> (Solution, Vec<usize>) {
        (solution.clone(), vec![])
    }

    pub fn intra_route(
        self,
        solution: &Solution,
        tabu_list: &Vec<Vec<usize>>,
        aspiration_cost: f64,
    ) -> (Solution, Vec<usize>) {
        let mut max_time = std::f64::MIN;
        let mut vehicle = 0;
        let mut is_truck = true;

        for (truck, &time) in solution.truck_working_time.iter().enumerate() {
            if time > max_time {
                max_time = time;
                vehicle = truck;
                is_truck = true;
            }
        }

        for (drone, &time) in solution.drone_working_time.iter().enumerate() {
            if time > max_time {
                max_time = time;
                vehicle = drone;
                is_truck = false;
            }
        }

        let mut truck_cloned = solution.truck_routes.clone();
        let mut drone_cloned = solution.drone_routes.clone();

        let mut min_cost = std::f64::MAX;
        let mut result = (solution.clone(), vec![]);

        macro_rules! search_route {
            ($original_routes:expr, $cloned_routes:expr) => {
                for (i, route) in $original_routes[vehicle].iter().enumerate() {
                    for (new_route, tabu) in route.intra_route(self).iter() {
                        // Save old route for restoration later
                        let old_route = route.clone();

                        // Temporary assign new route
                        $cloned_routes[vehicle][i] = new_route.clone();

                        // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                        // and get them back later during restoration
                        let s = Solution::new(truck_cloned, drone_cloned);
                        let cost = s.cost();
                        if cost < aspiration_cost || (!tabu_list.contains(tabu) && cost < min_cost) {
                            min_cost = cost;
                            result = (s.clone(), tabu.clone());
                        }

                        // Restore old route
                        truck_cloned = s.truck_routes;
                        drone_cloned = s.drone_routes;
                        $cloned_routes[vehicle][i] = old_route;
                    }
                }
            };
        }

        if is_truck {
            search_route!(solution.truck_routes, truck_cloned);
        } else {
            search_route!(solution.drone_routes, drone_cloned);
        }

        result
    }

    pub fn search(
        &self,
        solution: &Solution,
        tabu_list: &mut Vec<Vec<usize>>,
        tabu_size: usize,
        aspiration_cost: f64,
    ) -> Solution {
        let intra = self.intra_route(solution, tabu_list, aspiration_cost);
        let inter = self.inter_route(solution, tabu_list, aspiration_cost);
        let (result, tabu) = if intra.0.cost() < inter.0.cost() {
            intra
        } else {
            inter
        };

        match tabu_list.iter().position(|x| x == &tabu) {
            Some(index) => {
                tabu_list[index..].rotate_left(1);
            }
            None => {
                tabu_list.push(tabu.clone());
                if tabu_list.len() > tabu_size {
                    tabu_list.remove(0);
                }
            }
        }

        result
    }
}
