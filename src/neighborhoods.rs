use std::fmt::Display;

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

impl Display for Neighborhood {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::Move10 => "Move (1, 0)".to_string(),
                Self::Move11 => "Move (1, 1)".to_string(),
                Self::Move20 => "Move (2, 0)".to_string(),
                Self::Move21 => "Move (2, 1)".to_string(),
                Self::Move22 => "Move (2, 2)".to_string(),
                Self::TwoOpt => "2-opt".to_string(),
            }
        )
    }
}

impl Neighborhood {
    fn _find_decisive_vehicle(solution: &Solution) -> (usize, bool) {
        let mut max_time = f64::MIN;
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

        (vehicle, is_truck)
    }

    pub fn inter_route(
        self,
        solution: &Solution,
        tabu_list: &[Vec<usize>],
        aspiration_cost: f64,
    ) -> (Solution, Vec<usize>) {
        let (vehicle_i, is_truck) = Self::_find_decisive_vehicle(solution);

        let mut truck_cloned = solution.truck_routes.clone();
        let mut drone_cloned = solution.drone_routes.clone();

        let mut min_cost = f64::MAX;
        let mut require_feasible = false;
        let mut result = (solution.clone(), vec![]);

        /// Opposite of `Vec::swap_remove` - push an element to the end of the vector
        /// and swap it with the element at the given index.
        fn swap_push<T>(vec: &mut Vec<T>, index: usize, element: T) {
            let l = vec.len();
            vec.push(element);
            vec.swap(index, l);
        }

        macro_rules! search_route {
            ($original_routes_i:expr, $cloned_routes_i:expr) => {
                let routes_i = &$original_routes_i[vehicle_i];
                for (route_idx_i, route_i) in routes_i.iter().enumerate() {
                    macro_rules! search_other_routes {
                        ($original_routes_j:expr, $cloned_routes_j:expr) => {
                            for (vehicle_j, routes_j) in $original_routes_j.iter().enumerate() {
                                for (route_idx_j, route_j) in routes_j.iter().enumerate() {
                                    if std::ptr::addr_eq(route_i, route_j) {
                                        continue;
                                    }

                                    let mut neighbors = route_i.inter_route(route_j.clone(), self);
                                    let asymmetric = self == Self::Move10 || self == Self::Move20 || self == Self::Move21;
                                    if asymmetric {
                                        neighbors.extend(
                                            route_j
                                                .inter_route(route_i.clone(), self)
                                                .into_iter()
                                                .map(|t| (t.1, t.0, t.2)),
                                        );
                                    }

                                    for (new_route_i, new_route_j, tabu) in neighbors
                                    {
                                        // Temporary assign new routes.
                                        // Make use of `swap_remove` due to its O(1) complexity and the route order
                                        // of each vehicle is not important.
                                        let mut route_idx_j_after_swap_remove = route_idx_j;
                                        match &new_route_i {
                                            Some(new_route_i) => {
                                                $cloned_routes_i[vehicle_i][route_idx_i] = new_route_i.clone();
                                            }
                                            None => {
                                                $cloned_routes_i[vehicle_i].swap_remove(route_idx_i);
                                                if std::ptr::addr_eq(routes_i, routes_j) && route_idx_j == routes_j.len() - 1 {
                                                    route_idx_j_after_swap_remove = route_idx_i;
                                                }
                                            }
                                        }
                                        match &new_route_j {
                                            Some(new_route_j) => {
                                                $cloned_routes_j[vehicle_j][route_idx_j_after_swap_remove] = new_route_j.clone();
                                            }
                                            None => {
                                                $cloned_routes_j[vehicle_j].swap_remove(route_idx_j_after_swap_remove);
                                            }
                                        }

                                        // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                                        // and get them back later during restoration
                                        let s = Solution::new(truck_cloned, drone_cloned);
                                        let cost = s.cost();

                                        if !(require_feasible && !s.feasible) && cost < aspiration_cost || (!tabu_list.contains(&tabu) && cost < min_cost)
                                        {
                                            min_cost = cost;
                                            result = (s.clone(), tabu);
                                            if cost < aspiration_cost && s.feasible
                                            {
                                                require_feasible = true;
                                            }
                                        }

                                        // Restore old routes
                                        truck_cloned = s.truck_routes;
                                        drone_cloned = s.drone_routes;
                                        match new_route_j {
                                            Some(_) => {
                                                $cloned_routes_j[vehicle_j][route_idx_j_after_swap_remove] = route_j.clone();
                                            }
                                            None => {
                                                swap_push(
                                                    &mut $cloned_routes_j[vehicle_j],
                                                    route_idx_j_after_swap_remove,
                                                    route_j.clone(),
                                                );
                                            }
                                        }
                                        match new_route_i {
                                            Some(_) => {
                                                $cloned_routes_i[vehicle_i][route_idx_i] = route_i.clone();
                                            }
                                            None => {
                                                swap_push(
                                                    &mut $cloned_routes_i[vehicle_i],
                                                    route_idx_i,
                                                    route_i.clone(),
                                                );
                                            }
                                        }
                                    }
                                }
                            }
                        };
                    }

                    search_other_routes!(solution.truck_routes, truck_cloned);
                    search_other_routes!(solution.drone_routes, drone_cloned);
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

    pub fn intra_route(
        self,
        solution: &Solution,
        tabu_list: &[Vec<usize>],
        aspiration_cost: f64,
    ) -> (Solution, Vec<usize>) {
        let (vehicle, is_truck) = Self::_find_decisive_vehicle(solution);

        let mut truck_cloned = solution.truck_routes.clone();
        let mut drone_cloned = solution.drone_routes.clone();

        let mut min_cost = f64::MAX;
        let mut require_feasible = false;
        let mut result = (solution.clone(), vec![]);

        macro_rules! search_route {
            ($original_routes:expr, $cloned_routes:expr) => {
                for (i, route) in $original_routes[vehicle].iter().enumerate() {
                    for (new_route, tabu) in route.intra_route(self).iter() {
                        // Temporary assign new route
                        $cloned_routes[vehicle][i] = new_route.clone();

                        // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                        // and get them back later during restoration
                        let s = Solution::new(truck_cloned, drone_cloned);
                        let cost = s.cost();
                        if !(require_feasible && !s.feasible) && cost < aspiration_cost || (!tabu_list.contains(&tabu) && cost < min_cost)
                        {
                            min_cost = cost;
                            result = (s.clone(), tabu.clone());
                            if cost < aspiration_cost && s.feasible
                            {
                                require_feasible = true;
                            }
                        }

                        // Restore old route
                        truck_cloned = s.truck_routes;
                        drone_cloned = s.drone_routes;
                        $cloned_routes[vehicle][i] = route.clone();
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
