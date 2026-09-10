use crate::planio::Vec3;

#[derive(Debug, Clone, PartialEq)]
pub struct AeroControl {
    pub throttle: f64,
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub target_altitude: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct AeroTelemetry {
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub speed_mps: f64,
    pub altitude_m: f64,
    pub velocity: Vec3,
    pub thrust: f64,
    pub drag: f64,
    pub energy_used_wh: f64,
    pub battery_percent: f64,
    pub turn_radius_m: f64,
}

impl Default for AeroTelemetry {
    fn default() -> Self {
        AeroTelemetry {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            speed_mps: 0.0,
            altitude_m: 0.0,
            velocity: [0.0; 3],
            thrust: 0.0,
            drag: 0.0,
            energy_used_wh: 0.0,
            battery_percent: 0.0,
            turn_radius_m: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AeroState {
    pub mass: f64,
    pub gravity: f64,
    pub max_thrust: f64,
    pub drag_coefficient: f64,
    pub rotational_drag: f64,
    pub max_velocity: f64,
    pub battery_capacity_wh: f64,
    pub energy_drain: f64,
    pub efficiency: f64,
    pub altitude_kp: f64,
    pub altitude_kd: f64,
    pub position: Vec3,
    pub velocity: Vec3,
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub yaw_rate: f64,
    pub thrust: f64,
    pub energy_used_wh: f64,
    pub target_z: Option<f64>,
}

impl Default for AeroState {
    fn default() -> Self {
        let mass = 0.5;
        let gravity = 9.81;
        AeroState {
            mass,
            gravity,
            max_thrust: mass * gravity * 2.5,
            drag_coefficient: 0.2,
            rotational_drag: 8.0,
            max_velocity: 8.0,
            battery_capacity_wh: 15.0,
            energy_drain: 0.003,
            efficiency: 0.85,
            altitude_kp: 8.0,
            altitude_kd: 3.0,
            position: [0.0; 3],
            velocity: [0.0; 3],
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
            thrust: 0.0,
            energy_used_wh: 0.0,
            target_z: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AeroEngine {
    pub state: AeroState,
}

impl AeroEngine {
    pub fn new() -> AeroEngine {
        AeroEngine {
            state: AeroState::default(),
        }
    }

    pub fn step(&mut self, throttle: f64, control: &AeroControl, dt: f64) {
        let _ = (throttle, control, dt);
    }

    pub fn telemetry(&self) -> AeroTelemetry {
        AeroTelemetry::default()
    }
}

impl Default for AeroEngine {
    fn default() -> Self {
        AeroEngine::new()
    }
}
