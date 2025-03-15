use std::collections::{BTreeSet, BinaryHeap, HashSet};
use std::marker::PhantomData;
use std::rc::Rc;
use std::sync::atomic::Ordering;
use std::sync::LazyLock;
use std::{cmp, fmt};

use atomic_float;
use rand::{rng, seq::SliceRandom};
use serde::de::{SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::logger::Logger;
use crate::{
    clusterize,
    config::CONFIG,
    neighborhoods::Neighborhood,
    routes::{DroneRoute, Route, TruckRoute},
};

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

fn _serialize_routes<S>(routes: &Vec<Vec<Rc<impl Route>>>, serializer: S) -> Result<S::Ok, S::Error>
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

const PENALTY_COEFF: LazyLock<[atomic_float::AtomicF64; 4]> = LazyLock::new(|| {
    [
        atomic_float::AtomicF64::new(0.0),
        atomic_float::AtomicF64::new(0.0),
        atomic_float::AtomicF64::new(0.0),
        atomic_float::AtomicF64::new(0.0),
    ]
});

const NEIGHBORHOODS: LazyLock<[Neighborhood; 6]> = LazyLock::new(|| {
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
            capacity_violation += routes.iter().map(|r| r.capacity_violation()).sum::<f64>();
            waiting_time_violation += routes
                .iter()
                .map(|r| r.waiting_time_violation())
                .sum::<f64>();
        }
        for routes in &drone_routes {
            working_time = working_time.max(routes.iter().map(|r| r.working_time()).sum::<f64>());
            energy_violation += routes.iter().map(|r| r.energy_violation).sum::<f64>();
            capacity_violation += routes.iter().map(|r| r.capacity_violation()).sum::<f64>();
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

    pub fn cost(&self) -> f64 {
        self.working_time
            + penalty_coeff::<0>() * self.energy_violation
            + penalty_coeff::<1>() * self.capacity_violation
            + penalty_coeff::<2>() * self.waiting_time_violation
            + penalty_coeff::<3>() * self.fixed_time_violation
    }

    pub fn initialize() -> Solution {
        fn _sort_cluster_with_starting_point(cluster: &mut Vec<usize>, mut start: usize) -> () {
            if cluster.is_empty() {
                return;
            }

            let distance = &CONFIG.distances;
            for i in 0..cluster.len() {
                let mut min_distance = std::f64::INFINITY;
                let mut min_idx = 0;
                for j in i..cluster.len() {
                    let d = distance[start][cluster[j]];
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
            for customer in 1..CONFIG.customers_count + 1 {
                truck_routes[0].push(TruckRoute::new(vec![0, customer, 0]));
                truckable[customer] = _feasible(truck_routes.clone(), drone_routes.clone());
                truck_routes[0].pop();
            }
        }

        let mut dronable = vec![false; CONFIG.customers_count + 1];
        if CONFIG.drones_count > 0 {
            dronable[0] = true;
            for customer in 1..CONFIG.customers_count + 1 {
                if CONFIG.dronable[customer] {
                    drone_routes[0].push(DroneRoute::new(vec![0, customer, 0]));
                    dronable[customer] = _feasible(truck_routes.clone(), drone_routes.clone());
                    drone_routes[0].pop();
                }
            }
        }

        for customer in 1..CONFIG.customers_count + 1 {
            if !truckable[customer] && !dronable[customer] {
                panic!("Customer {customer} cannot be served by both trucks and drones")
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

            cluster.sort_by(|&i, &j| CONFIG.distances[0][i].total_cmp(&CONFIG.distances[0][j]));
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

        fn truck_next(
            truckable: &Vec<bool>,
            clusters: &Vec<Vec<usize>>,
            clusters_mapping: &Vec<usize>,
            queue: &mut BinaryHeap<_State>,
            global: &BTreeSet<usize>,
            truck_routes: &mut Vec<Vec<Rc<TruckRoute>>>,
            drone_routes: &Vec<Vec<Rc<DroneRoute>>>,
            parent: usize,
            vehicle: usize,
        ) {
            let mut min_distance = std::f64::INFINITY;
            let mut min_idx = 0;
            for &customer in &clusters[clusters_mapping[parent]] {
                if truckable[customer] && CONFIG.distances[parent][customer] < min_distance {
                    min_distance = CONFIG.distances[parent][customer];
                    min_idx = customer;
                }
            }

            if min_idx == 0 {
                for &customer in global.iter() {
                    if truckable[customer] && CONFIG.distances[parent][customer] < min_distance {
                        min_distance = CONFIG.distances[parent][customer];
                        min_idx = customer;
                    }
                }
            }

            if min_idx != 0 {
                let temp = Solution::new(truck_routes.clone(), drone_routes.clone());
                queue.push(_State {
                    working_time: temp.truck_working_time[vehicle],
                    vehicle,
                    parent,
                    index: min_idx,
                    is_truck: true,
                });
            }
        }

        fn drone_next(
            dronable: &Vec<bool>,
            clusters: &Vec<Vec<usize>>,
            clusters_mapping: &Vec<usize>,
            queue: &mut BinaryHeap<_State>,
            global: &BTreeSet<usize>,
            truck_routes: &Vec<Vec<Rc<TruckRoute>>>,
            drone_routes: &mut Vec<Vec<Rc<DroneRoute>>>,
            parent: usize,
            vehicle: usize,
        ) {
            let mut min_distance = std::f64::INFINITY;
            let mut min_idx = 0;
            for &customer in &clusters[clusters_mapping[parent]] {
                if dronable[customer] && CONFIG.distances[parent][customer] < min_distance {
                    min_distance = CONFIG.distances[parent][customer];
                    min_idx = customer;
                }
            }

            if min_idx == 0 {
                for &customer in global.iter() {
                    if dronable[customer] && CONFIG.distances[parent][customer] < min_distance {
                        min_distance = CONFIG.distances[parent][customer];
                        min_idx = customer;
                    }
                }
            }

            if min_idx != 0 {
                let temp = Solution::new(truck_routes.clone(), drone_routes.clone());
                queue.push(_State {
                    working_time: temp.drone_working_time[vehicle],
                    vehicle: vehicle,
                    parent: parent,
                    index: min_idx,
                    is_truck: false,
                });
            }
        }

        while !global.is_empty() {
            let packed = queue.pop().unwrap();

            let cluster = clusters_mapping[packed.index];
            if let Some(index) = clusters[cluster].iter().position(|&x| x == packed.index) {
                if packed.is_truck {
                    if packed.parent == 0 {
                        truck_routes[packed.vehicle].push(TruckRoute::single(packed.index));
                    } else {
                        let route = truck_routes[packed.vehicle].last_mut().unwrap();
                        *route = route.push(packed.index);
                    }
                } else {
                    if packed.parent == 0 {
                        drone_routes[packed.vehicle].push(DroneRoute::single(packed.index));
                    } else {
                        let route = drone_routes[packed.vehicle].last_mut().unwrap();
                        *route = route.push(packed.index);
                    }
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
                            packed.index,
                            packed.vehicle,
                        );
                    }
                } else {
                    if packed.is_truck {
                        if packed.parent == 0 {
                            truck_routes[packed.vehicle].pop();
                        } else {
                            let route = truck_routes[packed.vehicle].last_mut().unwrap();
                            *route = route.pop();
                        }
                    } else {
                        if packed.parent == 0 {
                            drone_routes[packed.vehicle].pop();
                        } else {
                            let route = drone_routes[packed.vehicle].last_mut().unwrap();
                            *route = route.pop();
                        }
                    }
                }
            }

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
                    packed.parent,
                    packed.vehicle,
                );
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
                let mut min_time = std::f64::INFINITY;
                for (i, &time) in working_time.iter().enumerate() {
                    if time < min_time {
                        min_time = time;
                        min_idx = i;
                    }
                }

                drone_routes[min_idx].push(route.clone());
                working_time[min_idx] += route.working_time();
            }
        }

        Solution::new(truck_routes, drone_routes)
    }

    pub fn tabu_search(root: Solution, logger: &mut Logger) -> Solution {
        let result = root.clone();
        let current = root.clone();

        let mut elite_set = vec![];
        elite_set.push(&result);

        let neighborhood_idx = 0;
        for iteration in 1.. {
            let neighborhood = NEIGHBORHOODS[neighborhood_idx];
            let neighbor = neighborhood.search(&current, &mut vec![], 0, 0.0);

            if elite_set.is_empty() {
                break;
            }

            logger.log(&current, neighborhood).unwrap();
        }

        result
    }
}
