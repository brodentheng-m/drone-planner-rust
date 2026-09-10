use crate::sim::drive::SimState;
use crate::sim::runtime::RuntimeState;

pub fn circle(state: &mut SimState, speed: f64, dir: &str) {
    let _ = (state, speed, dir);
}

pub fn circle_turn(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn square(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn square_turn(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn triangle(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn triangle_turn(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn spiral(state: &mut SimState, speed: f64, dir: &str) {
    let _ = (state, speed, dir);
}

pub fn sway(state: &mut SimState, speed: f64, secs: f64, dir: &str) {
    let _ = (state, speed, secs, dir);
}

pub fn keep_distance(state: &mut SimState, speed: f64, dist: f64) {
    let _ = (state, speed, dist);
}

pub fn avoid_wall(state: &mut SimState, speed: f64, dist: f64) {
    let _ = (state, speed, dist);
}

pub fn detect_wall(state: &mut SimState, var: &str, runtime: &mut RuntimeState) {
    let _ = (state, var, runtime);
}
