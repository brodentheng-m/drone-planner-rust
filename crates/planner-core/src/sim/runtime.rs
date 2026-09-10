use std::collections::BTreeMap;

pub const SENSOR_BATTERY: f64 = 80.0;
pub const SENSOR_FRONT_RANGE: f64 = 100.0;
pub const SENSOR_FRONT_COLOR: &str = "green";
pub const SENSOR_BACK_COLOR: &str = "blue";
pub const SENSOR_TEMPERATURE: f64 = 22.0;

#[derive(Debug, Clone, PartialEq)]
pub enum VarValue {
    Num(f64),
    Str(String),
    List(Vec<f64>),
}

#[derive(Debug, Clone, Default)]
pub struct RuntimeState {
    pub vars: BTreeMap<String, VarValue>,
}

impl RuntimeState {
    pub fn new() -> RuntimeState {
        RuntimeState::default()
    }
}

pub fn eval_expr(expr: &str, vars: &BTreeMap<String, VarValue>) -> f64 {
    let _ = (expr, vars);
    0.0
}

pub fn replace_vars(expr: &str, vars: &BTreeMap<String, VarValue>) -> String {
    let _ = (expr, vars);
    String::new()
}

pub fn get_battery() -> f64 {
    0.0
}

pub fn get_height(z: f64) -> f64 {
    let _ = z;
    0.0
}

pub fn get_front_range() -> f64 {
    0.0
}

pub fn get_bottom_range(z: f64) -> f64 {
    let _ = z;
    0.0
}

pub fn get_front_color() -> &'static str {
    ""
}

pub fn get_back_color() -> &'static str {
    ""
}

pub fn get_temperature() -> f64 {
    0.0
}

pub fn get_distance() -> f64 {
    0.0
}

pub fn list_append(state: &mut RuntimeState, name: &str, value: f64) {
    let _ = (state, name, value);
}

pub fn list_get(state: &RuntimeState, name: &str, index: i64) -> Option<f64> {
    let _ = (state, name, index);
    None
}

pub fn timer_start(state: &mut RuntimeState, name: &str) {
    let _ = (state, name);
}

pub fn timer_elapsed(state: &RuntimeState, name: &str) -> f64 {
    let _ = (state, name);
    0.0
}

pub fn time_sleep(state: &mut RuntimeState, dur: f64) {
    let _ = (state, dur);
}

pub fn drone_sleep(state: &mut RuntimeState, dur: f64) {
    let _ = (state, dur);
}
