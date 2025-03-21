use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::fmt;
use std::mem::swap;
use std::rc::Rc;

use crate::config::CONFIG;
use crate::neighborhoods::Neighborhood;

type _NeighborList<T> = Rc<Vec<(Rc<T>, Vec<usize>)>>;
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
        assert!(customers.len() >= 3);

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

pub trait Route: fmt::Display + Sized {
    fn new(customers: Vec<usize>) -> Rc<Self>;
    fn single(customer: usize) -> Rc<Self> {
        Self::new(vec![0, customer, 0])
    }

    fn data(&self) -> &_RouteData;
    fn working_time(&self) -> f64;
    fn capacity_violation(&self) -> f64;
    fn waiting_time_violation(&self) -> f64;

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self>;

    fn push(&self, customer: usize) -> Rc<Self> {
        let customers = &self.data().customers;
        let mut new_customers = customers.clone();
        new_customers.insert(customers.len() - 1, customer);
        Self::new(new_customers)
    }

    fn pop(&self) -> Rc<Self> {
        let customers = &self.data().customers;
        let mut new_customers = customers.clone();
        new_customers.remove(customers.len() - 2);
        Self::new(new_customers)
    }

    fn _servable(customer: usize) -> bool;

    /// Extract customer subsegments from this route to form a new route during an inter-route operation.
    ///
    /// Note that if the current route becomes empty after extracting the subsegment, the result set will be
    /// empty.
    fn inter_route_extract<T>(
        &self,
        neighborhood: Neighborhood,
    ) -> Vec<(Rc<Self>, Rc<T>, Vec<usize>)>
    where
        T: Route,
    {
        let customers = &self.data().customers;
        let mut results = vec![];
        let mut queue = VecDeque::new();
        let size = match neighborhood {
            Neighborhood::Move10 => 1,
            Neighborhood::Move20 => 2,
            _default => 0,
        };

        if size == 0 || customers.len() - 2 <= size {
            return results;
        }

        for i in 1..customers.len() - 1 {
            if T::_servable(customers[i]) {
                queue.push_back(customers[i]);
                if queue.len() > size {
                    queue.pop_front();
                }

                if queue.len() == size {
                    let mut original = customers[0..i - size + 1].to_vec();
                    original.extend(customers[i + 1..].iter().cloned());

                    let mut route = vec![0];
                    route.extend(queue.iter().copied());
                    route.push(0);

                    let tabu = customers[i - size + 1..i + 1].to_vec();
                    results.push((Self::new(original), T::new(route), tabu));
                }
            } else {
                queue.clear();
            }
        }

        results
    }

    #[allow(clippy::type_complexity)]
    /// Perform inter-route neighborhood search.
    ///
    /// This function is non-commutative (i.e. `r1.inter_route(r2, n) != r2.inter_route(r1, n)`). For example,
    /// `r1.inter_route(r2, Neighborhood::Move10)` will move 1 customer from `r1` to `r2`, but not from `r2` to `r1`.
    ///
    /// For symmetric neighborhoods (e.g. `Neighborhood::Move11`), this function will be commutative though.
    fn inter_route<T>(
        &self,
        other: Rc<T>,
        neighborhood: Neighborhood,
    ) -> Vec<(Option<Rc<Self>>, Option<Rc<T>>, Vec<usize>)>
    where
        T: Route,
    {
        let customers_i = &self.data().customers;
        let customers_j = &other.data().customers;

        let length_i = customers_i.len();
        let length_j = customers_j.len();

        let mut buffer_i = customers_i.clone();
        let mut buffer_j = customers_j.clone();

        let mut results = vec![];

        match neighborhood {
            Neighborhood::Move10 => {
                for (idx_i, &customer_i) in
                    customers_i.iter().enumerate().take(length_i - 1).skip(1)
                {
                    if !T::_servable(customer_i) {
                        continue;
                    }

                    let removed = buffer_i.remove(idx_i);
                    let route_i = if length_i == 3 {
                        None
                    } else {
                        Some(Self::new(buffer_i.clone()))
                    };
                    let tabu = vec![removed];

                    buffer_j.insert(1, removed);

                    for idx_j in 1..length_j {
                        let ptr = T::new(buffer_j.clone());
                        results.push((route_i.clone(), Some(ptr), tabu.clone()));

                        buffer_j.swap(idx_j, idx_j + 1);
                    }

                    buffer_i.insert(idx_i, removed);
                    buffer_j.pop();
                }
            }
            Neighborhood::Move11 => {
                for idx_i in 1..length_i - 1 {
                    if !T::_servable(buffer_i[idx_i]) {
                        continue;
                    }

                    for idx_j in 1..length_j - 1 {
                        if !Self::_servable(buffer_j[idx_j]) {
                            continue;
                        }

                        swap(&mut buffer_i[idx_i], &mut buffer_j[idx_j]);

                        let ptr_i = Self::new(buffer_i.clone());
                        let ptr_j = T::new(buffer_j.clone());
                        let tabu = vec![customers_i[idx_i], customers_j[idx_j]];
                        results.push((Some(ptr_i), Some(ptr_j), tabu));

                        swap(&mut buffer_i[idx_i], &mut buffer_j[idx_j]);
                    }
                }
            }
            Neighborhood::Move20 => {
                for idx_i in 1..length_i - 2 {
                    if !T::_servable(buffer_i[idx_i]) || !T::_servable(buffer_i[idx_i + 1]) {
                        continue;
                    }

                    let removed_x = buffer_i.remove(idx_i);
                    let removed_y = buffer_i.remove(idx_i);

                    let route_i = if length_i == 4 {
                        None
                    } else {
                        Some(Self::new(buffer_i.clone()))
                    };
                    let tabu = vec![removed_x, removed_y];

                    buffer_j.insert(1, removed_x);
                    buffer_j.insert(2, removed_y);

                    for idx_j in 1..length_j {
                        let ptr = T::new(buffer_j.clone());
                        results.push((route_i.clone(), Some(ptr), tabu.clone()));

                        buffer_j.swap(idx_j + 1, idx_j + 2);
                        buffer_j.swap(idx_j, idx_j + 1);
                    }

                    buffer_i.insert(idx_i, removed_x);
                    buffer_i.insert(idx_i + 1, removed_y);
                    buffer_j.pop();
                    buffer_j.pop();
                }
            }
            Neighborhood::Move21 => {
                for idx_i in 1..length_i - 2 {
                    if !T::_servable(buffer_i[idx_i]) || !T::_servable(buffer_i[idx_i + 1]) {
                        continue;
                    }

                    swap(&mut buffer_i[idx_i], &mut buffer_j[1]);
                    buffer_j.insert(2, buffer_i.remove(idx_i + 1));

                    for idx_j in 1..length_j - 1 {
                        if Self::_servable(buffer_j[idx_j]) {
                            let ptr_i = Self::new(buffer_i.clone());
                            let ptr_j = T::new(buffer_j.clone());
                            let tabu = vec![buffer_j[idx_j], buffer_j[idx_j + 1], buffer_i[idx_i]];
                            results.push((Some(ptr_i), Some(ptr_j), tabu));
                        }

                        swap(&mut buffer_i[idx_i], &mut buffer_j[idx_j + 2]);
                        buffer_j.swap(idx_j + 1, idx_j + 2);
                        buffer_j.swap(idx_j, idx_j + 1);
                    }

                    swap(&mut buffer_i[idx_i], &mut buffer_j[length_j - 1]);
                    buffer_i.insert(idx_i + 1, buffer_j.pop().unwrap());
                }
            }
            Neighborhood::Move22 => {
                for idx_i in 1..length_i - 2 {
                    if !T::_servable(buffer_i[idx_i]) || !T::_servable(buffer_i[idx_i + 1]) {
                        continue;
                    }

                    for idx_j in 1..length_j - 2 {
                        if !Self::_servable(buffer_j[idx_j])
                            || !Self::_servable(buffer_j[idx_j + 1])
                        {
                            continue;
                        }

                        swap(&mut buffer_i[idx_i], &mut buffer_j[idx_j]);
                        swap(&mut buffer_i[idx_i + 1], &mut buffer_j[idx_j + 1]);

                        let ptr_i = Self::new(buffer_i.clone());
                        let ptr_j = T::new(buffer_j.clone());
                        let tabu = vec![
                            buffer_i[idx_i],
                            buffer_i[idx_i + 1],
                            buffer_j[idx_j],
                            buffer_j[idx_j + 1],
                        ];
                        results.push((Some(ptr_i), Some(ptr_j), tabu));

                        swap(&mut buffer_i[idx_i], &mut buffer_j[idx_j]);
                        swap(&mut buffer_i[idx_i + 1], &mut buffer_j[idx_j + 1]);
                    }
                }
            }
            Neighborhood::TwoOpt => {
                let mut offset_i = length_i - 1;
                while offset_i > 1 && T::_servable(buffer_i[offset_i - 1]) {
                    offset_i -= 1;
                }

                let mut offset_j = length_j - 1;
                while offset_j > 1 && Self::_servable(buffer_j[offset_j - 1]) {
                    offset_j -= 1;
                }

                for idx_i in offset_i..length_i - 1 {
                    for idx_j in offset_j..length_j - 1 {
                        // Construct separate buffers from scratch
                        let mut buffer_i = customers_i[..idx_i].to_vec();
                        let mut buffer_j = customers_j[..idx_j].to_vec();

                        buffer_i.extend_from_slice(&customers_j[idx_j..]);
                        buffer_j.extend_from_slice(&customers_i[idx_i..]);

                        let tabu = vec![buffer_i[idx_i], buffer_j[idx_j]];

                        // Move the buffers to the new routes
                        let ptr_i = Self::new(buffer_i);
                        let ptr_j = T::new(buffer_j);
                        results.push((Some(ptr_i), Some(ptr_j), tabu));
                    }
                }
            }
        }

        results
    }

    /// Returns a pointer to the underlying cached intra-route neighbors.
    fn intra_route(&self, neighborhood: Neighborhood) -> _NeighborList<Self> {
        fn _intra_route_impl<T>(data: &_RouteData, neighborhood: Neighborhood) -> _NeighborList<T>
        where
            T: Route,
        {
            let length = data.customers.len();
            let mut results = vec![];
            let mut buffer = data.customers.clone();
            match neighborhood {
                Neighborhood::Move10 => {
                    for i in 1..length - 2 {
                        for j in i..length - 2 {
                            buffer.swap(j, j + 1);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer[i..length - 1].rotate_right(1);
                    }

                    for i in 2..length - 1 {
                        for j in (2..i + 1).rev() {
                            buffer.swap(j - 1, j);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer[1..i + 1].rotate_left(1);
                    }
                }
                Neighborhood::Move11 => {
                    for i in 1..length - 2 {
                        for j in i..length - 2 {
                            buffer.swap(j, j + 1);
                            buffer.swap(i, j);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i], data.customers[j + 1]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer.swap(i, length - 2);
                    }
                }
                Neighborhood::Move20 => {
                    for i in 1..length - 3 {
                        for j in i + 1..length - 2 {
                            buffer.swap(j, j + 1);
                            buffer.swap(j - 1, j);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i], data.customers[i + 1]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer[i..length - 1].rotate_right(2);
                    }

                    for i in 2..length - 2 {
                        for j in (1..i).rev() {
                            buffer.swap(j + 1, j + 2);
                            buffer.swap(j, j + 2);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i], data.customers[i + 1]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
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

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![
                                data.customers[i],
                                data.customers[i + 1],
                                data.customers[j + 2],
                            ];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer.swap(i, length - 3);
                        buffer[i + 1..length - 1].rotate_right(1);
                    }

                    for i in 2..length - 2 {
                        for j in (1..i).rev() {
                            buffer.swap(j + 1, j + 2);
                            buffer.swap(j, j + 2);
                            buffer.swap(j + 2, i + 1);

                            let ptr = T::new(buffer.clone());
                            let tabu =
                                vec![data.customers[i], data.customers[i + 1], data.customers[j]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer.swap(1, i + 1);
                        buffer[2..i + 2].rotate_left(1);
                    }
                }
                Neighborhood::Move22 => {
                    for i in 1..length.saturating_sub(4) {
                        {
                            buffer.swap(i, i + 2);
                            buffer.swap(i + 1, i + 3);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![
                                data.customers[i],
                                data.customers[i + 1],
                                data.customers[i + 2],
                                data.customers[i + 3],
                            ];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        for j in i + 3..length - 2 {
                            buffer.swap(i, i + 1);
                            buffer.swap(i + 1, j + 1);
                            buffer.swap(j, j + 1);
                            buffer.swap(j - 1, j);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![
                                data.customers[i],
                                data.customers[i + 1],
                                data.customers[j],
                                data.customers[j + 1],
                            ];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer.swap(i, length - 3);
                        buffer.swap(i + 1, length - 2);
                    }
                }
                Neighborhood::TwoOpt => {
                    for i in 1..length - 2 {
                        {
                            buffer.swap(i, i + 1);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i], data.customers[i + 1]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        for j in i + 2..length - 1 {
                            buffer[i..j + 1].rotate_right(1);

                            let ptr = T::new(buffer.clone());
                            let tabu = vec![data.customers[i], data.customers[j]];
                            // println!("buffer = {:?}, tabu = {:?}", buffer, tabu);
                            results.push((ptr, tabu));
                        }

                        buffer[i..length - 1].reverse();
                    }
                }
            }

            for (_, tabu) in results.iter_mut() {
                tabu.sort();
            }

            Rc::new(results)
        }

        let mut cache = self._intra_route_neighbors_cache().borrow_mut();
        match cache.get(&neighborhood) {
            Some(value) => value.clone(),
            None => {
                let values = _intra_route_impl::<Self>(self.data(), neighborhood);
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

impl fmt::Display for TruckRoute {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.data().customers)
    }
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

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self> {
        &self._neighbors
    }

    fn _servable(_customer: usize) -> bool {
        true
    }
}

impl TruckRoute {
    fn _calculate_waiting_time_violation(customers: &[usize], working_time: f64) -> f64 {
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

impl fmt::Display for DroneRoute {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.data().customers)
    }
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

    fn _intra_route_neighbors_cache(&self) -> &_NeighborhoodCache<Self> {
        &self._neighbors
    }

    fn _servable(customer: usize) -> bool {
        CONFIG.dronable[customer]
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
        let _capacity_violation = (data.value.weight - CONFIG.drone.capacity()).max(0.0);

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
