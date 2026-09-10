use crate::aero::{AeroControl, AeroEngine};
use crate::golden::{Collision, LedValue, Point};
use crate::obstacles::ObstacleSet;

pub const DT: f64 = 0.05;
pub const DEFAULT_SPEED: f64 = 0.5;
pub const TAKEOFF_HEIGHT: f64 = 0.8;
pub const FLIP_TRAVEL: f64 = 0.35;
pub const FLIP_CLIMB: f64 = 0.2;
pub const FLIP_POINTS: usize = 6;
pub const FLIP_SETTLE: usize = 2;
pub const MAX_PITCH: f64 = 25.0;
pub const MAX_ROLL: f64 = 25.0;
pub const MAX_YAW_RATE: f64 = 180.0;

#[derive(Debug, Clone)]
pub struct SimState {
    pub drone_id: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub heading: f64,
    pub points: Vec<Point>,
    pub total_duration: f64,
    pub flying: bool,
    pub led_color: LedValue,
    pub collisions: Vec<Collision>,
    pub engine: AeroEngine,
}

impl Default for SimState {
    fn default() -> Self {
        SimState {
            drone_id: String::new(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            heading: 0.0,
            points: Vec::new(),
            total_duration: 0.0,
            flying: false,
            led_color: LedValue::default(),
            collisions: Vec::new(),
            engine: AeroEngine::new(),
        }
    }
}

impl SimState {
    pub fn new() -> SimState {
        SimState::default()
    }

    pub fn push_point(&mut self, heading: f64, pitch: f64, roll: f64, led: Option<LedValue>) {
        let _ = (heading, pitch, roll, led);
    }

    pub fn record_collision(&mut self, collision: Collision) {
        self.collisions.push(collision);
    }
}

pub fn drive_step(state: &mut SimState, tx: f64, ty: f64, tz: f64, th: f64, dt: f64) {
    let _ = (state, tx, ty, tz, th, dt);
}

pub fn compute_control(
    engine: &AeroEngine,
    tx: f64,
    ty: f64,
    tz: f64,
    th: f64,
    dt: f64,
) -> AeroControl {
    let _ = (engine, tx, ty, tz, th, dt);
    AeroControl {
        throttle: 0.0,
        pitch: 0.0,
        roll: 0.0,
        yaw: 0.0,
        target_altitude: None,
    }
}

pub fn takeoff(state: &mut SimState) {
    let _ = state;
}

pub fn land(state: &mut SimState) {
    let _ = state;
}

pub fn hover(state: &mut SimState, dur: f64) {
    let _ = (state, dur);
}

pub fn flip(state: &mut SimState, dir: &str) {
    let _ = (state, dir);
}

pub fn go(state: &mut SimState, obstacles: Option<&ObstacleSet>, dir: &str, power: f64, dur: f64) {
    let _ = (state, obstacles, dir, power, dur);
}
