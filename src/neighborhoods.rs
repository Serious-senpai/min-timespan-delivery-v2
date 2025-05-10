use std::fmt::Display;

use crate::config::CONFIG;
use crate::routes::{DroneRoute, Route, TruckRoute};
use crate::solutions::Solution;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Neighborhood {
    Move10,
    Move11,
    Move20,
    Move21,
    Move22,
    TwoOpt,
    EjectionChain,
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
                Self::EjectionChain => "Ejection-chain".to_string(),
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

    fn _internal_update(
        tabu_list: &[Vec<usize>],
        aspiration_cost: &mut f64,
        min_cost: &mut f64,
        require_feasible: &mut bool,
        result: &mut (Solution, Vec<usize>),
        solution: &Solution,
        tabu: &Vec<usize>,
    ) {
        let feasible = solution.feasible;
        if *require_feasible && !feasible {
            return;
        }

        let cost = solution.cost();
        let new_best_global_solution = cost < *aspiration_cost && feasible;
        if new_best_global_solution || (!tabu_list.contains(tabu) && cost < *min_cost) {
            *min_cost = cost;
            *result = (solution.clone(), tabu.clone());
            if new_best_global_solution {
                *aspiration_cost = cost;
                *require_feasible = true;
            }
        }
    }

    pub fn inter_route(
        self,
        solution: &Solution,
        tabu_list: &[Vec<usize>],
        mut aspiration_cost: f64,
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

        match self {
            Self::Move10
            | Self::Move11
            | Self::Move20
            | Self::Move21
            | Self::Move22
            | Self::TwoOpt => {
                macro_rules! iterate_route_i {
                    ($original_routes_i:expr, $cloned_routes_i:expr, $one_customer_per_route_i:expr) => {
                        let routes_i = &$original_routes_i[vehicle_i];
                        for (route_idx_i, route_i) in routes_i.iter().enumerate() {
                            macro_rules! iterate_route_j {
                                ($original_routes_j:expr, $cloned_routes_j:expr, $one_customer_per_route_j:expr) => {
                                    for (vehicle_j, routes_j) in $original_routes_j.iter().enumerate() {
                                        for (route_idx_j, route_j) in routes_j.iter().enumerate() {
                                            // Dirty trick to compare 2 routes (because each customer can only be served exactly once)
                                            if route_i.data().customers[1] == route_j.data().customers[1] {
                                                continue;
                                            }

                                            let mut neighbors = route_i.inter_route(route_j.clone(), self);
                                            let asymmetric = self == Self::Move10 || self == Self::Move20 || self == Self::Move21;
                                            if asymmetric && std::ptr::addr_eq(routes_i, routes_j) /* same vehicle */ {
                                                // We only run assymetric operations if j is also the decisive vehicle (i.e. i = j).
                                                // Otherwise (vehicle j is not the decisive one), route i must "give" customers to route j
                                                // in order to reduce its working time, thus provide a better solution overall.
                                                //
                                                // Let the current working time be W = W_i of vehicle i. If we were to move customers from j
                                                // to i, it is undoubtedly that W' = W_i' > W = W_i.
                                                //
                                                // When moving customers from i to j however, we obtain W_i' < W_i = W. However, j must serve
                                                // more customers, thus W_j' > W_j. Therefore, W' = max(W_i', W_j'). Since W_i' < W, we know
                                                // that W' > W if and only if W' = W_j' > W_i' (j becomes the new decisive vehicle).
                                                //
                                                // It is unlikely that the second case produces a value of W_j' greater than W_i' in the first
                                                // case (metrics?). Hence, we can just skip the first case (i.e. do not move customers from j
                                                // to i).
                                                // In this code here, the identical vehicle check allows moving customers among different
                                                // routes of the current decisive vehicle.
                                                neighbors.extend(
                                                    route_j
                                                        .inter_route(route_i.clone(), self)
                                                        .into_iter()
                                                        .map(|t| (t.1, t.0, t.2)),
                                                );
                                            }

                                            for (new_route_i, new_route_j, tabu) in neighbors
                                            {
                                                if let Some(ref new_route_i) = new_route_i {
                                                    if $one_customer_per_route_i && new_route_i.data().customers.len() != 3 {
                                                        continue;
                                                    }
                                                }
                                                if let Some(ref new_route_j) = new_route_j {
                                                    if $one_customer_per_route_j && new_route_j.data().customers.len() != 3 {
                                                        continue;
                                                    }
                                                }

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
                                                        if std::ptr::addr_eq(routes_i, routes_j) /* same vehicle */ && route_idx_j == routes_j.len() - 1 {
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

                                                Self::_internal_update(
                                                    tabu_list,
                                                    &mut aspiration_cost,
                                                    &mut min_cost,
                                                    &mut require_feasible,
                                                    &mut result,
                                                    &s,
                                                    &tabu,
                                                );

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

                            iterate_route_j!(solution.truck_routes, truck_cloned, false);
                            iterate_route_j!(solution.drone_routes, drone_cloned, CONFIG.single_drone_route);
                        }
                    };
                }

                if is_truck {
                    iterate_route_i!(solution.truck_routes, truck_cloned, false);
                } else {
                    iterate_route_i!(
                        solution.drone_routes,
                        drone_cloned,
                        CONFIG.single_drone_route
                    );
                }

                macro_rules! iterate_route_i_extract {
                    ($original_routes_i:expr, $cloned_routes_i:expr) => {
                        let routes_i = &$original_routes_i[vehicle_i];
                        for (route_idx_i, route_i) in routes_i.iter().enumerate() {
                            macro_rules! iterate_route_j_append {
                                ($original_routes_j:expr, $cloned_routes_j:expr, $type_j:tt, $one_route_per_vehicle:expr, $one_customer_per_route:expr) => {
                                    for (new_route_i, new_route_j, tabu) in
                                        route_i.inter_route_extract::<$type_j>(self)
                                    {
                                        if $one_customer_per_route && new_route_j.data().customers.len() != 3 {
                                            continue;
                                        }

                                        $cloned_routes_i[vehicle_i][route_idx_i] = new_route_i;
                                        for vehicle_j in 0..$original_routes_j.len() {
                                            if $one_route_per_vehicle && !$cloned_routes_j[vehicle_j].is_empty()
                                            {
                                                continue;
                                            }

                                            $cloned_routes_j[vehicle_j].push(new_route_j.clone());

                                            let s = Solution::new(truck_cloned, drone_cloned);

                                            Self::_internal_update(
                                                tabu_list,
                                                &mut aspiration_cost,
                                                &mut min_cost,
                                                &mut require_feasible,
                                                &mut result,
                                                &s,
                                                &tabu,
                                            );

                                            // Restore old routes
                                            truck_cloned = s.truck_routes;
                                            drone_cloned = s.drone_routes;

                                            $cloned_routes_j[vehicle_j].pop();
                                        }
                                    }
                                };
                            }

                            iterate_route_j_append!(solution.truck_routes, truck_cloned, TruckRoute, CONFIG.single_truck_route, false);
                            iterate_route_j_append!(solution.drone_routes, drone_cloned, DroneRoute, false, CONFIG.single_drone_route);

                            $cloned_routes_i[vehicle_i][route_idx_i] = route_i.clone();
                        }
                    };
                }

                if is_truck {
                    iterate_route_i_extract!(solution.truck_routes, truck_cloned);
                } else {
                    iterate_route_i_extract!(solution.drone_routes, drone_cloned);
                }
            }

            Self::EjectionChain => {
                macro_rules! iterate_route_i {
                    ($original_routes_i:expr, $cloned_routes_i:expr) => {
                        let routes_i = &$original_routes_i[vehicle_i];
                        for (route_idx_i, route_i) in routes_i.iter().enumerate() {
                            macro_rules! iterate_route_j {
                                ($original_routes_j:expr, $cloned_routes_j:expr) => {
                                    for (vehicle_j, routes_j) in $original_routes_j.iter().enumerate() {
                                        for (route_idx_j, route_j) in routes_j.iter().enumerate() {
                                            // Dirty trick to compare 2 routes (because each customer can only be served exactly once)
                                            if route_i.data().customers[1] == route_j.data().customers[1] {
                                                continue;
                                            }

                                            macro_rules! iterate_route_k {
                                                ($original_routes_k:expr, $cloned_routes_k:expr, $one_customer_per_route_k:expr) => {
                                                    for (vehicle_k, routes_k) in $original_routes_k.iter().enumerate() {
                                                        for (route_idx_k, route_k) in routes_k.iter().enumerate() {
                                                            if route_i.data().customers[1] == route_k.data().customers[1] {
                                                                continue;
                                                            }

                                                            if route_j.data().customers[1] == route_k.data().customers[1] {
                                                                continue;
                                                            }

                                                            let neighbors = route_i.inter_route_3(route_j.clone(), route_k.clone(), self);
                                                            // No need for inter_route_3(k, j) because we are already looping through all possible pairs
                                                            for (new_route_i, new_route_j, new_route_k, tabu) in neighbors {
                                                                if $one_customer_per_route_k && new_route_k.data().customers.len() != 3 {
                                                                    continue;
                                                                }

                                                                // Temporary assign new routes.
                                                                // Make use of `swap_remove` due to its O(1) complexity and the route order
                                                                // of each vehicle is not important.
                                                                // **Note**: We assign route j and k first because they are guaranteed to not be empty.
                                                                $cloned_routes_k[vehicle_k][route_idx_k] = new_route_k.clone();
                                                                $cloned_routes_j[vehicle_j][route_idx_j] = new_route_j.clone();
                                                                match &new_route_i {
                                                                    Some(new_route_i) => {
                                                                        $cloned_routes_i[vehicle_i][route_idx_i] = new_route_i.clone();
                                                                    }
                                                                    None => {
                                                                        $cloned_routes_i[vehicle_i].swap_remove(route_idx_i);
                                                                    }
                                                                }

                                                                // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                                                                // and get them back later during restoration
                                                                let s = Solution::new(truck_cloned, drone_cloned);
                                                                Self::_internal_update(
                                                                    tabu_list,
                                                                    &mut aspiration_cost,
                                                                    &mut min_cost,
                                                                    &mut require_feasible,
                                                                    &mut result,
                                                                    &s,
                                                                    &tabu,
                                                                );

                                                                // Restore old routes
                                                                truck_cloned = s.truck_routes;
                                                                drone_cloned = s.drone_routes;
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
                                                                $cloned_routes_j[vehicle_j][route_idx_j] = route_j.clone();
                                                                $cloned_routes_k[vehicle_k][route_idx_k] = route_k.clone();
                                                            }
                                                        }
                                                    }
                                                }
                                            }

                                            iterate_route_k!(solution.truck_routes, truck_cloned, false);
                                            iterate_route_k!(solution.drone_routes, drone_cloned, CONFIG.single_drone_route);
                                        }
                                    }
                                };
                            }

                            iterate_route_j!(solution.truck_routes, truck_cloned);
                            iterate_route_j!(solution.drone_routes, drone_cloned);
                        }
                    };
                }

                if is_truck {
                    iterate_route_i!(solution.truck_routes, truck_cloned);
                } else {
                    iterate_route_i!(solution.drone_routes, drone_cloned);
                }
            }
        }

        result
    }

    pub fn intra_route(
        self,
        solution: &Solution,
        tabu_list: &[Vec<usize>],
        mut aspiration_cost: f64,
    ) -> (Solution, Vec<usize>) {
        let mut result = (solution.clone(), vec![]);
        if let Self::EjectionChain = self {
            return result;
        }

        let (vehicle, is_truck) = Self::_find_decisive_vehicle(solution);

        let mut truck_cloned = solution.truck_routes.clone();
        let mut drone_cloned = solution.drone_routes.clone();

        let mut min_cost = f64::MAX;
        let mut require_feasible = false;

        macro_rules! search_route {
            ($original_routes:expr, $cloned_routes:expr) => {
                for (i, route) in $original_routes[vehicle].iter().enumerate() {
                    for (new_route, tabu) in route.intra_route(self).iter() {
                        // Temporary assign new route
                        $cloned_routes[vehicle][i] = new_route.clone();

                        // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                        // and get them back later during restoration
                        let s = Solution::new(truck_cloned, drone_cloned);

                        Self::_internal_update(
                            tabu_list,
                            &mut aspiration_cost,
                            &mut min_cost,
                            &mut require_feasible,
                            &mut result,
                            &s,
                            &tabu,
                        );

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
    ) -> Option<Solution> {
        let intra = self.intra_route(solution, tabu_list, aspiration_cost);
        let inter = self.inter_route(solution, tabu_list, aspiration_cost);

        #[allow(clippy::if_same_then_else)]
        let (result, mut tabu) = if intra.1.is_empty() {
            inter // Intra-route neighborhood is empty
        } else if inter.1.is_empty() {
            intra // Inter-route neighborhood is empty
        } else if intra.0.cost() < inter.0.cost() {
            intra
        } else {
            inter
        };

        if tabu.is_empty() {
            // Both neighborhoods are empty
            return None;
        }

        tabu.sort();
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

        Some(result)
    }
}
