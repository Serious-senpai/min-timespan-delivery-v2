use std::collections::{BTreeSet, BinaryHeap};
use std::marker::PhantomData;
use std::rc::Rc;
use std::sync::atomic::Ordering;
use std::sync::LazyLock;
use std::{cmp, fmt};

use rand::Rng;
use rand::{rng, seq::SliceRandom};
use serde::de::{SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::cli::Strategy;
use crate::clusterize;
use crate::config::CONFIG;
use crate::logger::Logger;
use crate::neighborhoods::Neighborhood;
use crate::routes::{DroneRoute, Route, TruckRoute};

fn _deserialize_routes<'de, R, D>(deserializer: D) -> Result<Vec<Vec<Rc<R>>>, D::Error>
where
    R: fmt::Debug + Route,
    D: Deserializer<'de>,
{
    struct RouteVisitor<R>(PhantomData<R>);
    impl<'de, R: fmt::Debug + Route> Visitor<'de> for RouteVisitor<R> {
        type Value = Vec<Vec<Rc<R>>>;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("Expected route data")
        }

        fn visit_seq<S>(self, mut seq: S) -> Result<Self::Value, S::Error>
        where
            S: SeqAccess<'de>,
        {
            let mut result = vec![];
            while let Some(routes) = seq.next_element::<Vec<Vec<usize>>>()? {
                let mut to_push = vec![];
                for route in routes {
                    to_push.push(R::new(route));
                }

                result.push(to_push);
            }

            Ok(result)
        }
    }

    let visitor = RouteVisitor(PhantomData);
    deserializer.deserialize_seq(visitor)
}

fn _serialize_routes<S>(routes: &[Vec<Rc<impl Route>>], serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    serializer.collect_seq(routes.iter().map(|r| {
        r.iter()
            .map(|r| r.data().customers.clone())
            .collect::<Vec<Vec<usize>>>()
    }))
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Solution {
    #[serde(
        deserialize_with = "_deserialize_routes",
        serialize_with = "_serialize_routes"
    )]
    pub truck_routes: Vec<Vec<Rc<TruckRoute>>>,
    #[serde(
        deserialize_with = "_deserialize_routes",
        serialize_with = "_serialize_routes"
    )]
    pub drone_routes: Vec<Vec<Rc<DroneRoute>>>,

    pub truck_working_time: Vec<f64>,
    pub drone_working_time: Vec<f64>,

    pub working_time: f64,
    pub energy_violation: f64,
    pub capacity_violation: f64,
    pub waiting_time_violation: f64,
    pub fixed_time_violation: f64,

    pub feasible: bool,
}

static PENALTY_COEFF: LazyLock<[atomic_float::AtomicF64; 4]> = LazyLock::new(|| {
    [
        atomic_float::AtomicF64::new(1.0),
        atomic_float::AtomicF64::new(1.0),
        atomic_float::AtomicF64::new(1.0),
        atomic_float::AtomicF64::new(1.0),
    ]
});

static NEIGHBORHOODS: LazyLock<[Neighborhood; 6]> = LazyLock::new(|| {
    [
        Neighborhood::Move10,
        Neighborhood::Move11,
        Neighborhood::Move20,
        Neighborhood::Move21,
        Neighborhood::Move22,
        Neighborhood::TwoOpt,
    ]
});

pub fn penalty_coeff<const N: usize>() -> f64 {
    PENALTY_COEFF[N].load(Ordering::Relaxed)
}

fn _update_violation<const N: usize>(violation: f64) {
    let mut value = PENALTY_COEFF[N].load(Ordering::Relaxed);
    if violation > 0.0 {
        value *= 1.5;
    } else {
        value /= 1.5;
    };

    PENALTY_COEFF[N].store(value.clamp(1.0, 1e3), Ordering::Relaxed)
}

impl Solution {
    pub fn new(
        truck_routes: Vec<Vec<Rc<TruckRoute>>>,
        drone_routes: Vec<Vec<Rc<DroneRoute>>>,
    ) -> Solution {
        let mut working_time: f64 = 0.0;
        let mut energy_violation = 0.0;
        let mut capacity_violation = 0.0;
        let mut waiting_time_violation = 0.0;
        let mut fixed_time_violation = 0.0;
        for routes in &truck_routes {
            working_time = working_time.max(routes.iter().map(|r| r.working_time()).sum());
            capacity_violation +=
                routes.iter().map(|r| r.capacity_violation()).sum::<f64>() / CONFIG.truck.capacity;
            waiting_time_violation += routes
                .iter()
                .map(|r| r.waiting_time_violation())
                .sum::<f64>();
        }
        for routes in &drone_routes {
            working_time = working_time.max(routes.iter().map(|r| r.working_time()).sum::<f64>());
            energy_violation += routes.iter().map(|r| r.energy_violation).sum::<f64>();
            capacity_violation += routes.iter().map(|r| r.capacity_violation()).sum::<f64>()
                / CONFIG.drone.capacity();
            waiting_time_violation += routes
                .iter()
                .map(|r| r.waiting_time_violation())
                .sum::<f64>();
            fixed_time_violation += routes.iter().map(|r| r.fixed_time_violation).sum::<f64>();
        }

        let truck_working_time = truck_routes
            .iter()
            .map(|r| r.iter().map(|r| r.working_time()).sum())
            .collect();
        let drone_working_time = drone_routes
            .iter()
            .map(|r| r.iter().map(|r| r.working_time()).sum())
            .collect();

        energy_violation /= CONFIG.drone.battery();
        waiting_time_violation /= CONFIG.waiting_time_limit;
        fixed_time_violation /= CONFIG.drone.fixed_time();

        Solution {
            truck_routes,
            drone_routes,
            working_time,
            energy_violation,
            capacity_violation,
            waiting_time_violation,
            fixed_time_violation,
            feasible: energy_violation == 0.0
                && capacity_violation == 0.0
                && waiting_time_violation == 0.0
                && fixed_time_violation == 0.0,
            truck_working_time,
            drone_working_time,
        }
    }

    pub fn verify(&self) {
        let mut customers = vec![false; CONFIG.customers_count + 1];
        customers[0] = true;

        for routes in &self.truck_routes {
            if CONFIG.single_truck_route && routes.len() > 1 {
                panic!("Truck routes {:?} have more than one route", routes);
            }

            for route in routes {
                for &c in &route.data().customers {
                    customers[c] = true;
                }
            }
        }

        for routes in &self.drone_routes {
            for route in routes {
                if CONFIG.single_drone_route && route.data().customers.len() != 3 {
                    panic!("Drone route {} has more than one customer", route);
                }

                for &c in &route.data().customers {
                    customers[c] = true;
                    if !CONFIG.dronable[c] {
                        panic!("Customer {} is not dronable", c);
                    }
                }
            }
        }

        for (c, served) in customers.iter().enumerate() {
            if !served {
                panic!("Customer {} is not served", c);
            }
        }
    }

    pub fn cost(&self) -> f64 {
        self.working_time
            * (1.0
                + penalty_coeff::<0>() * self.energy_violation
                + penalty_coeff::<1>() * self.capacity_violation
                + penalty_coeff::<2>() * self.waiting_time_violation
                + penalty_coeff::<3>() * self.fixed_time_violation)
                .powf(CONFIG.penalty_exponent)
    }

    pub fn hamming_distance(&self, other: &Solution) -> usize {
        fn fill_repr<T>(vehicle_routes: &Vec<Vec<Rc<T>>>, repr: &mut [usize])
        where
            T: Route,
        {
            for routes in vehicle_routes {
                for route in routes {
                    let customers = &route.data().customers;
                    for i in 1..customers.len() - 1 {
                        repr[customers[i]] = customers[i + 1];
                    }
                }
            }
        }

        let mut self_repr = vec![0; CONFIG.customers_count + 1];
        fill_repr(&self.truck_routes, &mut self_repr);
        fill_repr(&self.drone_routes, &mut self_repr);

        let mut other_repr = vec![0; CONFIG.customers_count + 1];
        fill_repr(&other.truck_routes, &mut other_repr);
        fill_repr(&other.drone_routes, &mut other_repr);

        self_repr
            .iter()
            .zip(other_repr.iter())
            .filter(|(a, b)| a != b)
            .count()
    }

    pub fn post_optimization(&self) -> Solution {
        let mut result = Rc::new(self.clone());

        let mut improved = true;
        while improved {
            improved = false;
            for neighborhood in NEIGHBORHOODS.iter() {
                if let Some(best) = neighborhood.search(&result, &mut vec![], 0, result.cost()) {
                    if best.cost() < result.cost() && best.feasible {
                        result = Rc::new(best);
                        improved = true;
                    }
                }
            }
        }

        Solution::clone(&result)
    }

    pub fn initialize() -> Solution {
        fn _sort_cluster_with_starting_point(
            cluster: &mut [usize],
            mut start: usize,
            distance: &[Vec<f64>],
        ) {
            if cluster.is_empty() {
                return;
            }

            for i in 0..cluster.len() {
                let mut min_distance = f64::INFINITY;
                let mut min_idx = 0;
                for (j, &customer) in cluster.iter().enumerate().skip(i) {
                    let d = distance[start][customer];
                    if d < min_distance {
                        min_distance = d;
                        min_idx = j;
                    }
                }

                start = cluster[min_idx];
                cluster.swap(i, min_idx);
            }
        }

        fn _feasible(
            truck_routes: Vec<Vec<Rc<TruckRoute>>>,
            drone_routes: Vec<Vec<Rc<DroneRoute>>>,
        ) -> bool {
            let solution = Solution::new(truck_routes, drone_routes);
            solution.feasible
        }

        let mut index = Vec::from_iter(1..CONFIG.customers_count + 1);
        let mut clusters = clusterize::clusterize(&mut index, CONFIG.trucks_count);

        let mut truck_routes = vec![vec![]; CONFIG.trucks_count];
        let mut drone_routes = vec![vec![]; CONFIG.trucks_count];

        let mut clusters_mapping = vec![0; CONFIG.customers_count + 1];
        for (i, cluster) in clusters.iter().enumerate() {
            for &customer in cluster {
                clusters_mapping[customer] = i;
            }
        }

        let mut truckable = vec![false; CONFIG.customers_count + 1];
        if CONFIG.trucks_count > 0 {
            truckable[0] = true;
            for (customer, truckable) in truckable
                .iter_mut()
                .enumerate()
                .skip(1)
                .take(CONFIG.customers_count)
            {
                truck_routes[0].push(TruckRoute::single(customer));
                *truckable = _feasible(truck_routes.clone(), drone_routes.clone());
                truck_routes[0].pop();
            }
        }

        let mut dronable = vec![false; CONFIG.customers_count + 1];
        if CONFIG.drones_count > 0 {
            dronable[0] = true;
            for (customer, dronable) in dronable
                .iter_mut()
                .enumerate()
                .skip(1)
                .take(CONFIG.customers_count)
            {
                if CONFIG.dronable[customer] {
                    drone_routes[0].push(DroneRoute::single(customer));
                    *dronable = _feasible(truck_routes.clone(), drone_routes.clone());
                    drone_routes[0].pop();
                }
            }
        }

        for customer in 1..CONFIG.customers_count + 1 {
            if !truckable[customer] && !dronable[customer] {
                panic!("Customer {customer} cannot be served by neither trucks nor drones")
            }
        }

        #[derive(Debug)]
        struct _State {
            working_time: f64,
            vehicle: usize,
            parent: usize,
            index: usize,
            is_truck: bool,
        }

        impl Ord for _State {
            fn cmp(&self, other: &Self) -> cmp::Ordering {
                self.working_time.total_cmp(&other.working_time).reverse()
            }
        }

        impl PartialOrd for _State {
            fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        impl PartialEq for _State {
            fn eq(&self, other: &Self) -> bool {
                self.cmp(other) == cmp::Ordering::Equal
            }
        }

        impl Eq for _State {}

        let mut queue = BinaryHeap::new();
        let mut rng = rng();
        for (i, cluster) in clusters.iter_mut().enumerate() {
            if cluster.is_empty() {
                continue;
            }

            cluster.shuffle(&mut rng);
            for &customer in cluster.iter() {
                if truckable[customer] {
                    queue.push(_State {
                        working_time: 0.0,
                        vehicle: i,
                        parent: 0,
                        index: *cluster.first().unwrap(),
                        is_truck: true,
                    });

                    break;
                }
            }

            cluster.sort_by(|&i, &j| {
                CONFIG.drone_distances[0][i].total_cmp(&CONFIG.drone_distances[0][j])
            });
            for &customer in cluster.iter() {
                if dronable[customer] {
                    queue.push(_State {
                        working_time: 0.0,
                        vehicle: i,
                        parent: 0,
                        index: cluster[0],
                        is_truck: false,
                    });

                    break;
                }
            }
        }

        let mut global = BTreeSet::from_iter(1..CONFIG.customers_count + 1);

        #[allow(clippy::too_many_arguments)]
        fn truck_next(
            truckable: &[bool],
            clusters: &[Vec<usize>],
            clusters_mapping: &[usize],
            queue: &mut BinaryHeap<_State>,
            global: &BTreeSet<usize>,
            truck_routes: &mut [Vec<Rc<TruckRoute>>],
            drone_routes: &[Vec<Rc<DroneRoute>>],
            parent: usize,
            vehicle: usize,
        ) {
            let mut min_distance = f64::INFINITY;
            let mut min_idx = 0;
            for &customer in &clusters[clusters_mapping[parent]] {
                if truckable[customer] && CONFIG.truck_distances[parent][customer] < min_distance {
                    min_distance = CONFIG.truck_distances[parent][customer];
                    min_idx = customer;
                }
            }

            if min_idx == 0 {
                for &customer in global.iter() {
                    if truckable[customer]
                        && CONFIG.truck_distances[parent][customer] < min_distance
                    {
                        min_distance = CONFIG.truck_distances[parent][customer];
                        min_idx = customer;
                    }
                }
            }

            if min_idx != 0 {
                let temp = Solution::new(truck_routes.to_vec(), drone_routes.to_vec());
                queue.push(_State {
                    working_time: temp.truck_working_time[vehicle],
                    vehicle,
                    parent,
                    index: min_idx,
                    is_truck: true,
                });
            }
        }

        #[allow(clippy::too_many_arguments)]
        fn drone_next(
            dronable: &[bool],
            clusters: &[Vec<usize>],
            clusters_mapping: &[usize],
            queue: &mut BinaryHeap<_State>,
            global: &BTreeSet<usize>,
            truck_routes: &[Vec<Rc<TruckRoute>>],
            drone_routes: &mut [Vec<Rc<DroneRoute>>],
            parent: usize,
            vehicle: usize,
        ) {
            let mut min_distance = f64::INFINITY;
            let mut min_idx = 0;
            for &customer in &clusters[clusters_mapping[parent]] {
                if dronable[customer] && CONFIG.drone_distances[parent][customer] < min_distance {
                    min_distance = CONFIG.drone_distances[parent][customer];
                    min_idx = customer;
                }
            }

            if min_idx == 0 {
                for &customer in global.iter() {
                    if dronable[customer] && CONFIG.drone_distances[parent][customer] < min_distance
                    {
                        min_distance = CONFIG.drone_distances[parent][customer];
                        min_idx = customer;
                    }
                }
            }

            if min_idx != 0 {
                let temp = Solution::new(truck_routes.to_vec(), drone_routes.to_vec());
                queue.push(_State {
                    working_time: temp.drone_working_time[vehicle],
                    vehicle,
                    parent,
                    index: min_idx,
                    is_truck: false,
                });
            }
        }

        while !global.is_empty() {
            let packed = queue.pop().unwrap_or_else(|| panic!("A trivial solution cannot be constructed during initialization.\nThe following customers cannot be served: {:?}", global));

            let cluster = clusters_mapping[packed.index];
            match clusters[cluster].iter().position(|&x| x == packed.index) {
                Some(index) => {
                    if packed.is_truck {
                        if packed.parent == 0 {
                            truck_routes[packed.vehicle].push(TruckRoute::single(packed.index));
                        } else {
                            let route = truck_routes[packed.vehicle].last_mut().unwrap();
                            *route = route.push(packed.index);
                        }
                    } else if packed.parent == 0 {
                        drone_routes[packed.vehicle].push(DroneRoute::single(packed.index));
                    } else {
                        let route = drone_routes[packed.vehicle].last_mut().unwrap();
                        *route = route.push(packed.index);
                    }

                    if _feasible(truck_routes.clone(), drone_routes.clone()) {
                        clusters[cluster].remove(index);
                        global.remove(&packed.index);

                        if packed.is_truck {
                            truck_next(
                                &truckable,
                                &clusters,
                                &clusters_mapping,
                                &mut queue,
                                &global,
                                &mut truck_routes,
                                &drone_routes,
                                packed.index,
                                packed.vehicle,
                            );
                        } else {
                            drone_next(
                                &dronable,
                                &clusters,
                                &clusters_mapping,
                                &mut queue,
                                &global,
                                &truck_routes,
                                &mut drone_routes,
                                if CONFIG.single_drone_route {
                                    0
                                } else {
                                    packed.index
                                },
                                packed.vehicle,
                            );
                        }
                    } else if packed.is_truck {
                        if packed.parent == 0 {
                            truck_routes[packed.vehicle].pop();
                        } else {
                            let route = truck_routes[packed.vehicle].last_mut().unwrap();
                            *route = route.pop();
                        }

                        if !CONFIG.single_truck_route {
                            truck_next(
                                &truckable,
                                &clusters,
                                &clusters_mapping,
                                &mut queue,
                                &global,
                                &mut truck_routes,
                                &drone_routes,
                                0,
                                packed.vehicle,
                            );
                        }
                    } else {
                        if packed.parent == 0 {
                            drone_routes[packed.vehicle].pop();
                        } else {
                            let route = drone_routes[packed.vehicle].last_mut().unwrap();
                            *route = route.pop();
                        }

                        drone_next(
                            &dronable,
                            &clusters,
                            &clusters_mapping,
                            &mut queue,
                            &global,
                            &truck_routes,
                            &mut drone_routes,
                            0,
                            packed.vehicle,
                        );
                    }
                }
                None => {
                    if packed.is_truck {
                        truck_next(
                            &truckable,
                            &clusters,
                            &clusters_mapping,
                            &mut queue,
                            &global,
                            &mut truck_routes,
                            &drone_routes,
                            packed.parent,
                            packed.vehicle,
                        );
                    } else {
                        drone_next(
                            &dronable,
                            &clusters,
                            &clusters_mapping,
                            &mut queue,
                            &global,
                            &truck_routes,
                            &mut drone_routes,
                            if CONFIG.single_drone_route {
                                0
                            } else {
                                packed.parent
                            },
                            packed.vehicle,
                        );
                    }
                }
            }
        }

        if CONFIG.drones_count > 0 {
            // Resize drone routes to `CONFIG.drones_count`
            let mut all_routes = vec![];
            for routes in &drone_routes {
                all_routes.extend(routes.iter().cloned());
            }
            all_routes.sort_by(|f, s| f.working_time().total_cmp(&s.working_time()).reverse());

            drone_routes.clear();
            drone_routes.resize(CONFIG.drones_count, vec![]);

            let mut working_time = vec![0.0; CONFIG.drones_count];
            for route in all_routes {
                let mut min_idx = 0;
                let mut min_time = f64::INFINITY;
                for (i, &time) in working_time.iter().enumerate() {
                    if time < min_time {
                        min_time = time;
                        min_idx = i;
                    }
                }

                drone_routes[min_idx].push(route.clone());
                working_time[min_idx] += route.working_time();
            }
        } else {
            drone_routes.clear();
        }

        Solution::new(truck_routes, drone_routes)
    }

    pub fn tabu_search(root: Solution, logger: &mut Logger) -> Solution {
        let mut total_vehicle = 0;
        for truck in &root.truck_routes {
            total_vehicle += !truck.is_empty() as usize;
        }
        for drone in &root.drone_routes {
            total_vehicle += !drone.is_empty() as usize;
        }
        let base_hyperparameter = CONFIG.customers_count as f64 / total_vehicle as f64;
        let tabu_size = (CONFIG.tabu_size_factor * base_hyperparameter) as usize;
        let reset_after = if CONFIG.fix_iteration.is_some() {
            i64::MAX as usize // usize::MAX cannot be stored in SQLite
        } else {
            std::cmp::min(
                (CONFIG.reset_after_factor * base_hyperparameter) as usize,
                500,
            )
        };

        let mut result = Rc::new(root);
        let mut last_improved = 0;

        if !CONFIG.dry_run {
            let mut current = result.clone();

            let mut elite_set = vec![];
            elite_set.push(result.clone());

            let mut neighborhood_idx = 0;

            let iteration_range = match CONFIG.fix_iteration {
                Some(iteration) => 1..iteration + 1,
                None => 1..usize::MAX,
            };
            let mut rng = rand::rng();

            let mut tabu_lists = vec![vec![]; NEIGHBORHOODS.len()];

            fn _record_new_solution(
                neighbor: &Rc<Solution>,
                result: &mut Rc<Solution>,
                last_improved: &mut usize,
                iteration: usize,
                elite_set: &mut Vec<Rc<Solution>>,
            ) {
                if neighbor.cost() < result.cost() && neighbor.feasible {
                    *result = neighbor.clone();
                    *last_improved = iteration;

                    if CONFIG.max_elite_size > 0 {
                        if elite_set.len() == CONFIG.max_elite_size {
                            let (idx, _) = elite_set
                                .iter()
                                .enumerate()
                                .min_by_key(|s| s.1.hamming_distance(result))
                                .unwrap();
                            elite_set.remove(idx);
                        }

                        elite_set.push(neighbor.clone());
                    }
                }
            }

            fn _update_violation_solution(s: &Solution) {
                _update_violation::<0>(s.energy_violation);
                _update_violation::<1>(s.capacity_violation);
                _update_violation::<2>(s.waiting_time_violation);
                _update_violation::<3>(s.fixed_time_violation);
            }

            for iteration in iteration_range {
                if CONFIG.verbose {
                    eprint!(
                        "Iteration #{} (reset in {}): {:.2}/{:.2}, elite set {}/{}     \r",
                        iteration,
                        reset_after.saturating_sub((iteration - last_improved) % reset_after),
                        current.cost(),
                        result.cost(),
                        elite_set.len(),
                        CONFIG.max_elite_size
                    );
                }

                let neighborhood = NEIGHBORHOODS[neighborhood_idx];

                let old_current = current.clone();
                if let Some(neighbor) = neighborhood.search(
                    &current,
                    &mut tabu_lists[neighborhood_idx],
                    tabu_size,
                    result.cost(),
                ) {
                    let neighbor = Rc::new(neighbor);
                    _record_new_solution(
                        &neighbor,
                        &mut result,
                        &mut last_improved,
                        iteration,
                        &mut elite_set,
                    );

                    current = neighbor;
                }

                if iteration != last_improved && (iteration - last_improved) % reset_after == 0 {
                    if elite_set.is_empty() {
                        break;
                    }

                    let i = rng.random_range(0..elite_set.len());
                    current = elite_set.swap_remove(i);
                    for tabu_list in &mut tabu_lists {
                        tabu_list.clear();
                    }

                    let mut ejection_chain_tabu_list = vec![]; // Still have to maintain a tabu list to avoid cycles
                    for _ in 0..CONFIG.ejection_chain_iterations {
                        if let Some(neighbor) = Neighborhood::EjectionChain.search(
                            &current,
                            &mut ejection_chain_tabu_list,
                            CONFIG.ejection_chain_iterations,
                            result.cost(),
                        ) {
                            current = Rc::new(neighbor);
                            _record_new_solution(
                                &current,
                                &mut result,
                                &mut last_improved,
                                iteration,
                                &mut elite_set,
                            );
                        }
                    }

                    _update_violation_solution(&current);
                    logger
                        .log(
                            &current,
                            Neighborhood::EjectionChain,
                            &ejection_chain_tabu_list,
                        )
                        .unwrap();
                } else {
                    _update_violation_solution(&current);
                    logger
                        .log(&current, neighborhood, &tabu_lists[neighborhood_idx])
                        .unwrap();
                }

                match CONFIG.strategy {
                    Strategy::Random => {
                        neighborhood_idx = rng.random_range(0..NEIGHBORHOODS.len());
                    }
                    Strategy::Cyclic => {
                        neighborhood_idx = (neighborhood_idx + 1) % NEIGHBORHOODS.len();
                    }
                    Strategy::Vns => {
                        if iteration == last_improved {
                            neighborhood_idx = 0;
                        } else {
                            neighborhood_idx = (neighborhood_idx + 1) % NEIGHBORHOODS.len();
                            if neighborhood_idx != 0 {
                                current = old_current;
                            }
                        }
                    }
                }
            }

            if CONFIG.verbose {
                eprintln!();
            }

            result = Rc::new(result.post_optimization());
        }

        logger
            .finalize(&result, tabu_size, reset_after, last_improved)
            .unwrap();

        Solution::clone(&result)
    }
}
