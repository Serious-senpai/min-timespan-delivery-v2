use std::fmt::{self, Display};
use std::ptr;
use std::rc::Rc;

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
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
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

/// Opposite of `Vec::swap_remove` - push an element to the end of the vector
/// and swap it with the element at the given index.
fn _swap_push<T>(vec: &mut Vec<T>, index: usize, element: T) {
    let l = vec.len();
    vec.push(element);
    vec.swap(index, l);
}

struct _IterationState<'a> {
    pub original: &'a Solution,
    pub tabu_list: &'a [Vec<usize>],
    pub aspiration_cost: &'a mut f64,
    pub min_cost: &'a mut f64,
    pub require_feasible: &'a mut bool,
    pub result: &'a mut (Solution, Vec<usize>),
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

    fn _internal_update(state: &mut _IterationState, solution: &Solution, tabu: &Vec<usize>) {
        let feasible = solution.feasible;
        if *state.require_feasible && !feasible {
            return;
        }

        let cost = solution.cost();
        let new_best_global_solution = cost < *state.aspiration_cost && feasible;
        if new_best_global_solution || (!state.tabu_list.contains(tabu) && cost < *state.min_cost) {
            *state.min_cost = cost;
            *state.result = (solution.clone(), tabu.clone());
            if new_best_global_solution {
                *state.aspiration_cost = cost;
                *state.require_feasible = true;
            }
        }
    }

    fn _inter_route_internal<RI>(
        self,
        state: &mut _IterationState,
        mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
        mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
        vehicle_i: usize,
    ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
    where
        RI: Route,
    {
        fn iterate_route_j<RI, RJ>(
            neighborhood: Neighborhood,
            state: &mut _IterationState,
            mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
            mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
            vehicle_i: usize,
            route_idx_i: usize,
            route_i: &Rc<RI>,
        ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
        where
            RI: Route,
            RJ: Route,
        {
            let original_routes_i =
                RI::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);
            let original_routes_j =
                RJ::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);

            let routes_i = &original_routes_i[vehicle_i];
            for (vehicle_j, routes_j) in original_routes_j.iter().enumerate() {
                for (route_idx_j, route_j) in routes_j.iter().enumerate() {
                    // Dirty trick to compare 2 routes (because each customer can only be served exactly once)
                    if route_i.data().customers[1] == route_j.data().customers[1] {
                        continue;
                    }

                    let mut neighbors = route_i.inter_route(route_j.clone(), neighborhood);
                    let asymmetric = neighborhood == Neighborhood::Move10
                        || neighborhood == Neighborhood::Move20
                        || neighborhood == Neighborhood::Move21;
                    if asymmetric {
                        neighbors.extend(
                            route_j
                                .inter_route(route_i.clone(), neighborhood)
                                .into_iter()
                                .map(|t| (t.1, t.0, t.2)),
                        );
                    }

                    for (new_route_i, new_route_j, tabu) in neighbors {
                        if let Some(ref new_route_i) = new_route_i {
                            if RI::single_customer() && new_route_i.data().customers.len() != 3 {
                                continue;
                            }
                        }
                        if let Some(ref new_route_j) = new_route_j {
                            if RJ::single_customer() && new_route_j.data().customers.len() != 3 {
                                continue;
                            }
                        }

                        // Temporary assign new routes.
                        // Make use of `swap_remove` due to its O(1) complexity and the route order
                        // of each vehicle is not important.
                        let mut route_idx_j_after_swap_remove = route_idx_j;

                        // Wrap code blocks to drop the mutable references afterwards
                        {
                            let cloned_routes_i =
                                RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                            match &new_route_i {
                                Some(new_route_i) => {
                                    cloned_routes_i[vehicle_i][route_idx_i] = new_route_i.clone();
                                }
                                None => {
                                    cloned_routes_i[vehicle_i].swap_remove(route_idx_i);
                                    if ptr::addr_eq(routes_i, routes_j) /* same vehicle */ && route_idx_j == routes_j.len() - 1
                                    {
                                        route_idx_j_after_swap_remove = route_idx_i;
                                    }
                                }
                            }
                        }

                        {
                            let cloned_routes_j =
                                RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                            match &new_route_j {
                                Some(new_route_j) => {
                                    cloned_routes_j[vehicle_j][route_idx_j_after_swap_remove] =
                                        new_route_j.clone();
                                }
                                None => {
                                    cloned_routes_j[vehicle_j]
                                        .swap_remove(route_idx_j_after_swap_remove);
                                }
                            }
                        }

                        // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                        // and get them back later during restoration
                        let s = Solution::new(truck_cloned, drone_cloned);

                        Neighborhood::_internal_update(state, &s, &tabu);

                        // Restore old routes
                        truck_cloned = s.truck_routes;
                        drone_cloned = s.drone_routes;

                        {
                            let cloned_routes_j =
                                RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                            match new_route_j {
                                Some(_) => {
                                    cloned_routes_j[vehicle_j][route_idx_j_after_swap_remove] =
                                        route_j.clone();
                                }
                                None => {
                                    _swap_push(
                                        &mut cloned_routes_j[vehicle_j],
                                        route_idx_j_after_swap_remove,
                                        route_j.clone(),
                                    );
                                }
                            }
                        }

                        {
                            let cloned_routes_i =
                                RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                            match new_route_i {
                                Some(_) => {
                                    cloned_routes_i[vehicle_i][route_idx_i] = route_i.clone();
                                }
                                None => {
                                    _swap_push(
                                        &mut cloned_routes_i[vehicle_i],
                                        route_idx_i,
                                        route_i.clone(),
                                    );
                                }
                            }
                        }
                    }
                }
            }

            (truck_cloned, drone_cloned)
        }

        let original_routes_i =
            RI::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);
        for (route_idx_i, route_i) in original_routes_i[vehicle_i].iter().enumerate() {
            (truck_cloned, drone_cloned) = iterate_route_j::<RI, TruckRoute>(
                self,
                state,
                truck_cloned,
                drone_cloned,
                vehicle_i,
                route_idx_i,
                route_i,
            );
            (truck_cloned, drone_cloned) = iterate_route_j::<RI, DroneRoute>(
                self,
                state,
                truck_cloned,
                drone_cloned,
                vehicle_i,
                route_idx_i,
                route_i,
            );
        }

        (truck_cloned, drone_cloned)
    }

    fn _inter_route_extract_internal<RI>(
        self,
        state: &mut _IterationState,
        mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
        mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
        vehicle_i: usize,
    ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
    where
        RI: Route,
    {
        fn iterate_route_j_append<RI, RJ>(
            neighborhood: Neighborhood,
            state: &mut _IterationState,
            mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
            mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
            vehicle_i: usize,
            route_idx_i: usize,
            route_i: &Rc<RI>,
        ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
        where
            RI: Route,
            RJ: Route,
        {
            let original_routes_j =
                RJ::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);

            for (new_route_i, new_route_j, tabu) in route_i.inter_route_extract::<RJ>(neighborhood)
            {
                if RJ::single_customer() && new_route_j.data().customers.len() != 3 {
                    continue;
                }

                {
                    let cloned_routes_i =
                        RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                    cloned_routes_i[vehicle_i][route_idx_i] = new_route_i;
                }

                for vehicle_j in 0..original_routes_j.len() {
                    if RJ::single_route() && !original_routes_j[vehicle_j].is_empty() {
                        continue;
                    }

                    {
                        let cloned_routes_j =
                            RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                        cloned_routes_j[vehicle_j].push(new_route_j.clone());
                    }

                    let s = Solution::new(truck_cloned, drone_cloned);

                    Neighborhood::_internal_update(state, &s, &tabu);

                    // Restore old routes
                    truck_cloned = s.truck_routes;
                    drone_cloned = s.drone_routes;

                    let cloned_routes_j =
                        RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                    cloned_routes_j[vehicle_j].pop();
                }

                let cloned_routes_i =
                    RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                cloned_routes_i[vehicle_i][route_idx_i] = route_i.clone();
            }

            (truck_cloned, drone_cloned)
        }

        let original_routes_i =
            RI::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);
        for (route_idx_i, route_i) in original_routes_i[vehicle_i].iter().enumerate() {
            (truck_cloned, drone_cloned) = iterate_route_j_append::<RI, TruckRoute>(
                self,
                state,
                truck_cloned,
                drone_cloned,
                vehicle_i,
                route_idx_i,
                route_i,
            );
            (truck_cloned, drone_cloned) = iterate_route_j_append::<RI, DroneRoute>(
                self,
                state,
                truck_cloned,
                drone_cloned,
                vehicle_i,
                route_idx_i,
                route_i,
            );
        }

        (truck_cloned, drone_cloned)
    }

    fn _ejection_chain_internal<DR, RI>(
        self,
        state: &mut _IterationState,
        mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
        mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
        decisive: usize,
    ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
    where
        DR: Route,
        RI: Route,
    {
        fn iterate_route_j<DR, RI, RJ>(
            neighborhood: Neighborhood,
            state: &mut _IterationState,
            mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
            mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
            decisive: usize,
            vehicle_i: usize,
            route_idx_i: usize,
            route_i: &Rc<RI>,
        ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
        where
            DR: Route,
            RI: Route,
            RJ: Route,
        {
            fn iterate_route_k<RI, RJ, RK>(
                neighborhood: Neighborhood,
                state: &mut _IterationState,
                mut truck_cloned: Vec<Vec<Rc<TruckRoute>>>,
                mut drone_cloned: Vec<Vec<Rc<DroneRoute>>>,
                vehicle_i: usize,
                route_idx_i: usize,
                route_i: &Rc<RI>,
                vehicle_j: usize,
                route_idx_j: usize,
                route_j: &Rc<RJ>,
            ) -> (Vec<Vec<Rc<TruckRoute>>>, Vec<Vec<Rc<DroneRoute>>>)
            where
                RI: Route,
                RJ: Route,
                RK: Route,
            {
                let original_routes_k = RK::get_correct_route(
                    &state.original.truck_routes,
                    &state.original.drone_routes,
                );

                for (vehicle_k, routes_k) in original_routes_k.iter().enumerate() {
                    for (route_idx_k, route_k) in routes_k.iter().enumerate() {
                        if route_i.data().customers[1] == route_k.data().customers[1] {
                            continue;
                        }

                        if route_j.data().customers[1] == route_k.data().customers[1] {
                            continue;
                        }

                        let neighbors =
                            route_i.inter_route_3(route_j.clone(), route_k.clone(), neighborhood);
                        // No need for inter_route_3(k, j) because we are already looping through all possible pairs
                        for (new_route_i, new_route_j, new_route_k, tabu) in neighbors {
                            if RK::single_customer() && new_route_k.data().customers.len() != 3 {
                                continue;
                            }

                            // Temporary assign new routes.
                            // Make use of `swap_remove` due to its O(1) complexity and the route order
                            // of each vehicle is not important.
                            // **Note**: We assign route j and k first because they are guaranteed to not be empty.
                            {
                                let cloned_routes_k =
                                    RK::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                cloned_routes_k[vehicle_k][route_idx_k] = new_route_k.clone();
                            }

                            {
                                let cloned_routes_j =
                                    RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                cloned_routes_j[vehicle_j][route_idx_j] = new_route_j.clone();
                            }

                            {
                                let cloned_routes_i =
                                    RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                match &new_route_i {
                                    Some(new_route_i) => {
                                        cloned_routes_i[vehicle_i][route_idx_i] =
                                            new_route_i.clone();
                                    }
                                    None => {
                                        cloned_routes_i[vehicle_i].swap_remove(route_idx_i);
                                    }
                                }
                            }

                            // Construct the new solution: move `truck_cloned` and `drone_cloned` to the temp solution
                            // and get them back later during restoration
                            let s = Solution::new(truck_cloned, drone_cloned);
                            Neighborhood::_internal_update(state, &s, &tabu);

                            // Restore old routes
                            truck_cloned = s.truck_routes;
                            drone_cloned = s.drone_routes;
                            {
                                let cloned_routes_i =
                                    RI::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                match new_route_i {
                                    Some(_) => {
                                        cloned_routes_i[vehicle_i][route_idx_i] = route_i.clone();
                                    }
                                    None => {
                                        _swap_push(
                                            &mut cloned_routes_i[vehicle_i],
                                            route_idx_i,
                                            route_i.clone(),
                                        );
                                    }
                                }
                            }
                            {
                                let cloned_routes_j =
                                    RJ::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                cloned_routes_j[vehicle_j][route_idx_j] = route_j.clone();
                            }
                            {
                                let cloned_routes_k =
                                    RK::get_correct_route_mut(&mut truck_cloned, &mut drone_cloned);
                                cloned_routes_k[vehicle_k][route_idx_k] = route_k.clone();
                            }
                        }
                    }
                }

                (truck_cloned, drone_cloned)
            }

            let original_routes_j =
                RJ::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);

            for (vehicle_j, routes_j) in original_routes_j.iter().enumerate() {
                // Either i or j must be the decisive vehicle
                if (DR::ID == RI::ID && decisive == vehicle_i)
                    || (DR::ID == RJ::ID && decisive == vehicle_j)
                {
                    for (route_idx_j, route_j) in routes_j.iter().enumerate() {
                        // Dirty trick to compare 2 routes (because each customer can only be served exactly once)
                        if route_i.data().customers[1] == route_j.data().customers[1] {
                            continue;
                        }

                        (truck_cloned, drone_cloned) = iterate_route_k::<RI, RJ, TruckRoute>(
                            neighborhood,
                            state,
                            truck_cloned,
                            drone_cloned,
                            vehicle_i,
                            route_idx_i,
                            route_i,
                            vehicle_j,
                            route_idx_j,
                            route_j,
                        );
                        (truck_cloned, drone_cloned) = iterate_route_k::<RI, RJ, DroneRoute>(
                            neighborhood,
                            state,
                            truck_cloned,
                            drone_cloned,
                            vehicle_i,
                            route_idx_i,
                            route_i,
                            vehicle_j,
                            route_idx_j,
                            route_j,
                        );
                    }
                }
            }

            (truck_cloned, drone_cloned)
        }

        let original_routes_i =
            RI::get_correct_route(&state.original.truck_routes, &state.original.drone_routes);
        for (vehicle_i, routes_i) in original_routes_i.iter().enumerate() {
            for (route_idx_i, route_i) in routes_i.iter().enumerate() {
                (truck_cloned, drone_cloned) = iterate_route_j::<DR, RI, TruckRoute>(
                    self,
                    state,
                    truck_cloned,
                    drone_cloned,
                    decisive,
                    vehicle_i,
                    route_idx_i,
                    route_i,
                );
                (truck_cloned, drone_cloned) = iterate_route_j::<DR, RI, DroneRoute>(
                    self,
                    state,
                    truck_cloned,
                    drone_cloned,
                    decisive,
                    vehicle_i,
                    route_idx_i,
                    route_i,
                );
            }
        }

        (truck_cloned, drone_cloned)
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

        let mut state = _IterationState {
            original: solution,
            tabu_list,
            aspiration_cost: &mut aspiration_cost,
            min_cost: &mut min_cost,
            require_feasible: &mut require_feasible,
            result: &mut result,
        };

        match self {
            Self::Move10
            | Self::Move11
            | Self::Move20
            | Self::Move21
            | Self::Move22
            | Self::TwoOpt => {
                (truck_cloned, drone_cloned) = if is_truck {
                    self._inter_route_internal::<TruckRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    )
                } else {
                    self._inter_route_internal::<DroneRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    )
                };

                if is_truck {
                    self._inter_route_extract_internal::<TruckRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    );
                } else {
                    self._inter_route_extract_internal::<DroneRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    );
                }
            }

            Self::EjectionChain => {
                if is_truck {
                    (truck_cloned, drone_cloned) = self
                        ._ejection_chain_internal::<TruckRoute, TruckRoute>(
                            &mut state,
                            truck_cloned,
                            drone_cloned,
                            vehicle_i,
                        );
                    self._ejection_chain_internal::<TruckRoute, DroneRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    );
                } else {
                    (truck_cloned, drone_cloned) = self
                        ._ejection_chain_internal::<DroneRoute, TruckRoute>(
                            &mut state,
                            truck_cloned,
                            drone_cloned,
                            vehicle_i,
                        );
                    self._ejection_chain_internal::<DroneRoute, DroneRoute>(
                        &mut state,
                        truck_cloned,
                        drone_cloned,
                        vehicle_i,
                    );
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

        let mut state = _IterationState {
            original: solution,
            tabu_list,
            aspiration_cost: &mut aspiration_cost,
            min_cost: &mut min_cost,
            require_feasible: &mut require_feasible,
            result: &mut result,
        };

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
                            &mut state,
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
