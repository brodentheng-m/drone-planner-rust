use crate::sim::drive::SimState;

pub fn move_forward(state: &mut SimState, dist: f64, speed: f64) {
    let _ = (state, dist, speed);
}

pub fn move_backward(state: &mut SimState, dist: f64, speed: f64) {
    let _ = (state, dist, speed);
}

pub fn move_left(state: &mut SimState, dist: f64, speed: f64) {
    let _ = (state, dist, speed);
}

pub fn move_right(state: &mut SimState, dist: f64, speed: f64) {
    let _ = (state, dist, speed);
}

pub fn turn_left(state: &mut SimState, deg: f64) {
    let _ = (state, deg);
}

pub fn turn_right(state: &mut SimState, deg: f64) {
    let _ = (state, deg);
}

pub fn turn_degree(state: &mut SimState, deg: f64, timeout: f64, p_value: f64) {
    let _ = (state, deg, timeout, p_value);
}
