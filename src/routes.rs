use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::{Rc, Weak};

use crate::config::CONFIG;
use crate::neighborhoods::Neighborhood;

type _NeighborList<T> = Rc<Vec<Weak<T>>>;
type _NeighborhoodCache<T> = RefCell<HashMap<Neighborhood, _NeighborList<T>>>;

#[derive(Debug)]
struct _RouteDataValues {
    distance: f64,
    weight: f64,
}

#[derive(Debug)]
pub struct _RouteData {
    pub customers: Vec<usize>,
    value: _RouteDataValues,
}

impl _RouteData {
    fn _construct(customers: Vec<usize>) -> _RouteData {
        assert!(customers.first() == Some(&0));
        assert!(customers.last() == Some(&0));

        let mut distance = 0.0;
        let mut weight = 0.0;
        for i in 0..customers.len() - 1 {
            distance += CONFIG.distances[customers[i]][customers[i + 1]];
            weight += CONFIG.demands[customers[i]];
        }

        _RouteData {
            customers,
            value: _RouteDataValues { distance, weight },
        }
    }
}

pub trait Route {
    fn new(customers: Vec<usize>) -> Rc<Self>
    where
        Self: Sized;

    fn single(customer: usize) -> Rc<Self>
    where
        Self: Sized,
    {
        Self::new(vec![0, customer, 0])
    }

    fn data(&self) -> &_RouteData;
    fn working_time(&self) -> f64;
    fn capacity_violation(&self) -> f64;
    fn waiting_time_violation(&self) -> f64;

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self>
    where
        Self: Sized;

    fn _intra_route_impl(&self, neighborhood: Neighborhood) -> _NeighborList<Self>
    where
        Self: Sized,
    {
        let data = self.data();
        let length = data.customers.len();
        let mut results = vec![];
        let mut buffer = data.customers.clone();
        match neighborhood {
            Neighborhood::Move10 => {
                for i in 1..length - 2 {
                    for j in i..length - 2 {
                        buffer.swap(j, j + 1);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer[i..length - 1].rotate_right(1);
                }

                for i in 2..length - 1 {
                    for j in (2..i + 1).rev() {
                        buffer.swap(j - 1, j);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer[1..i + 1].rotate_left(1);
                }
            }
            Neighborhood::Move11 => {
                for i in 1..length - 2 {
                    for j in i..length - 2 {
                        buffer.swap(j, j + 1);
                        buffer.swap(i, j);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer.swap(i, length - 2);
                }
            }
            Neighborhood::Move20 => {
                for i in 1..length - 3 {
                    for j in i + 1..length - 2 {
                        buffer.swap(j, j + 1);
                        buffer.swap(j - 1, j);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer[i..length - 1].rotate_right(2);
                }

                for i in 2..length - 2 {
                    for j in (1..i).rev() {
                        buffer.swap(j + 1, j + 2);
                        buffer.swap(j, j + 2);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer[1..i + 2].rotate_left(2);
                }
            }
            Neighborhood::Move21 => {
                for i in 1..length - 3 {
                    for j in i..length - 3 {
                        buffer.swap(j + 1, j + 2);
                        buffer.swap(j, j + 1);
                        buffer.swap(i, j);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer.swap(i, length - 3);
                    buffer[i + 1..length - 1].rotate_right(1);
                }

                for i in 2..length - 2 {
                    for j in (1..i).rev() {
                        buffer.swap(j + 1, j + 2);
                        buffer.swap(j, j + 2);
                        buffer.swap(j + 2, i + 1);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer.swap(1, i + 1);
                    buffer[2..i + 2].rotate_left(1);
                }
            }
            Neighborhood::Move22 => {
                for i in 1..length - 4 {
                    {
                        buffer.swap(i, i + 2);
                        buffer.swap(i + 1, i + 3);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    for j in i + 3..length - 2 {
                        buffer.swap(i, i + 1);
                        buffer.swap(i + 1, j + 1);
                        buffer.swap(j, j + 1);
                        buffer.swap(j - 1, j);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer.swap(i, length - 3);
                    buffer.swap(i + 1, length - 2);
                }
            }
            Neighborhood::TwoOpt => {
                for i in 1..length - 2 {
                    {
                        buffer.swap(i, i + 1);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    for j in i + 2..length - 1 {
                        buffer[i..j + 1].rotate_right(1);

                        let ptr = Self::new(buffer.clone());
                        results.push(Rc::downgrade(&ptr));
                    }

                    buffer[i..length - 1].reverse();
                }
            }
        }

        Rc::new(results)
    }

    fn push(&self, customer: usize) -> Rc<Self>
    where
        Self: Sized,
    {
        let customers = &self.data().customers;
        let mut new_customers = customers.clone();
        new_customers.insert(customers.len() - 1, customer);
        Self::new(new_customers)
    }

    fn pop(&self) -> Rc<Self>
    where
        Self: Sized,
    {
        let customers = &self.data().customers;
        let mut new_customers = customers.clone();
        new_customers.remove(customers.len() - 2);
        Self::new(new_customers)
    }

    fn intra_route(&self, neighborhood: Neighborhood) -> _NeighborList<Self>
    where
        Self: Sized,
    {
        let mut cache = self
            ._intra_route_neighbors_cache()
            .try_borrow_mut()
            .ok()
            .unwrap();
        match cache.get(&neighborhood) {
            Some(value) => return value.clone(),
            None => {
                let values = self._intra_route_impl(neighborhood);
                cache.insert(neighborhood, values.clone());
                values
            }
        }
    }
}

#[derive(Debug)]
pub struct TruckRoute {
    _data: _RouteData,
    _working_time: f64,
    _neighbors: _NeighborhoodCache<TruckRoute>,
    _capacity_violation: f64,
    _waiting_time_violation: f64,
}

impl Route for TruckRoute {
    fn new(customers: Vec<usize>) -> Rc<TruckRoute> {
        thread_local! {
            static _CACHE: RefCell<HashMap<Vec<usize>, Rc<TruckRoute>>> = RefCell::new(HashMap::new());
        }

        let cached = _CACHE.with_borrow(|c| c.get(&customers).cloned());
        match cached {
            Some(value) => value,
            None => {
                let route = Rc::new(TruckRoute::_construct(_RouteData::_construct(
                    customers.clone(),
                )));
                _CACHE.with(|c| {
                    let mut r = c.borrow_mut();
                    r.insert(customers, route.clone())
                });

                route
            }
        }
    }

    fn data(&self) -> &_RouteData {
        &self._data
    }

    fn working_time(&self) -> f64 {
        self._working_time
    }

    fn capacity_violation(&self) -> f64 {
        self._capacity_violation
    }

    fn waiting_time_violation(&self) -> f64 {
        self._waiting_time_violation
    }

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self>
    where
        Self: Sized,
    {
        &self._neighbors
    }
}

impl TruckRoute {
    fn _calculate_waiting_time_violation(customers: &Vec<usize>, working_time: f64) -> f64 {
        let speed = CONFIG.truck.speed;
        let mut waiting_time_violation = 0.0;
        let mut accumulate_time = 0.0;
        for i in 1..customers.len() - 1 {
            accumulate_time += CONFIG.distances[customers[i - 1]][customers[i]] / speed;
            waiting_time_violation +=
                (working_time - accumulate_time - CONFIG.waiting_time_limit).max(0.0);
        }

        waiting_time_violation
    }

    fn _construct(data: _RouteData) -> TruckRoute {
        let speed = CONFIG.truck.speed;
        let _working_time = data.value.distance / speed;
        let _capacity_violation = (data.value.weight - CONFIG.truck.capacity).max(0.0);
        let _waiting_time_violation =
            Self::_calculate_waiting_time_violation(&data.customers, _working_time);

        TruckRoute {
            _data: data,
            _working_time,
            _neighbors: RefCell::new(HashMap::new()),
            _capacity_violation,
            _waiting_time_violation,
        }
    }
}

#[derive(Debug)]
pub struct DroneRoute {
    _data: _RouteData,
    _working_time: f64,
    _neighbors: _NeighborhoodCache<DroneRoute>,
    _capacity_violation: f64,
    _waiting_time_violation: f64,

    pub energy_violation: f64,
    pub fixed_time_violation: f64,
}

impl Route for DroneRoute {
    fn new(customers: Vec<usize>) -> Rc<DroneRoute> {
        thread_local! {
            static _CACHE: RefCell<HashMap<Vec<usize>, Rc<DroneRoute>>> = RefCell::new(HashMap::new());
        }

        let cached = _CACHE.with_borrow(|c| c.get(&customers).cloned());
        match cached {
            Some(value) => value,
            None => {
                let route = Rc::new(DroneRoute::_construct(_RouteData::_construct(
                    customers.clone(),
                )));
                _CACHE.with(|c| {
                    let mut r = c.borrow_mut();
                    r.insert(customers, route.clone())
                });
                route
            }
        }
    }

    fn data(&self) -> &_RouteData {
        &self._data
    }

    fn working_time(&self) -> f64 {
        self._working_time
    }

    fn capacity_violation(&self) -> f64 {
        self._capacity_violation
    }

    fn waiting_time_violation(&self) -> f64 {
        self._waiting_time_violation
    }

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self>
    where
        Self: Sized,
    {
        &self._neighbors
    }
}

impl DroneRoute {
    fn _construct(data: _RouteData) -> DroneRoute {
        let customers = &data.customers;
        let distances = &CONFIG.distances;
        let drone = &CONFIG.drone;

        let _working_time = CONFIG.drone.cruise_time(data.value.distance)
            + (CONFIG.drone.takeoff_time() + CONFIG.drone.landing_time())
                * (customers.len() as f64 - 1.0);
        let _capacity_violation = (data.value.weight - CONFIG.truck.capacity).max(0.0);

        let mut time = 0.0;
        let mut energy = 0.0;
        let mut weight = 0.0;
        let mut _waiting_time_violation = 0.0;
        for i in 0..customers.len() - 1 {
            let takeoff = drone.takeoff_time();
            let cruise = drone.cruise_time(distances[customers[i]][customers[i + 1]]);
            let landing = drone.landing_time();

            time += takeoff + cruise + landing;
            energy += drone.takeoff_power(weight) * takeoff
                + drone.cruise_power(weight) * cruise
                + drone.landing_power(weight) * landing;
            weight += CONFIG.demands[customers[i]];
            _waiting_time_violation += (_working_time - time - CONFIG.waiting_time_limit).max(0.0);
        }

        let energy_violation = (energy - CONFIG.drone.battery()).max(0.0);
        let fixed_time_violation = (_working_time - CONFIG.drone.fixed_time()).max(0.0);

        DroneRoute {
            _data: data,
            _working_time,
            _neighbors: RefCell::new(HashMap::new()),
            _capacity_violation,
            _waiting_time_violation,
            energy_violation,
            fixed_time_violation,
        }
    }
}
