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
    drag_mag: f64,
}

impl AeroEngine {
    pub fn new() -> AeroEngine {
        AeroEngine {
            state: AeroState::default(),
            drag_mag: 0.0,
        }
    }

    pub fn step(&mut self, throttle: f64, control: &AeroControl, dt: f64) {
        let t = throttle.max(0.0).min(1.0);
        let pitch_deg = control.pitch;
        let roll_deg = control.roll;
        let yaw_cmd = control.yaw;

        self.state.pitch = pitch_deg;
        self.state.roll = roll_deg;

        self.state.yaw_rate +=
            (yaw_cmd - self.state.yaw_rate) * (self.state.rotational_drag * dt).min(1.0);
        self.state.yaw += self.state.yaw_rate * dt;

        let pitch_rad = pitch_deg * std::f64::consts::PI / 180.0;
        let roll_rad = roll_deg * std::f64::consts::PI / 180.0;
        let yaw_rad = self.state.yaw * std::f64::consts::PI / 180.0;
        let cos_pitch = pitch_rad.cos();
        let cos_roll = roll_rad.cos();
        let tilt_base = cos_pitch * cos_roll;

        let thrust_total;
        match control.target_altitude {
            Some(target_z) if target_z.is_finite() => {
                let target_vel = match self.state.target_z {
                    Some(prev) => (target_z - prev) / dt,
                    None => 0.0,
                };
                self.state.target_z = Some(target_z);
                let err = target_z - self.state.position[2];
                let err_dot = target_vel - self.state.velocity[2];
                let required_up = self.state.mass * self.state.gravity
                    + self.state.altitude_kp * err
                    + self.state.altitude_kd * err_dot;
                let tilt = tilt_base.max(0.25);
                thrust_total = (required_up / tilt).max(0.0).min(self.state.max_thrust);
            }
            _ => {
                thrust_total = t * self.state.max_thrust;
            }
        }

        let fwd_x = yaw_rad.cos();
        let fwd_y = yaw_rad.sin();
        let right_x = -fwd_y;
        let right_y = fwd_x;

        let f_fwd = -thrust_total * pitch_rad.sin();
        let f_right = thrust_total * roll_rad.sin();
        let f_up = thrust_total * cos_pitch * cos_roll;

        let fx = f_fwd * fwd_x + f_right * right_x;
        let fy = f_fwd * fwd_y + f_right * right_y;
        let fz = f_up;

        let drag_x = -self.state.drag_coefficient
            * self.state.velocity[0]
            * self.state.velocity[0].abs();
        let drag_y = -self.state.drag_coefficient
            * self.state.velocity[1]
            * self.state.velocity[1].abs();
        let drag_z = -self.state.drag_coefficient
            * self.state.velocity[2]
            * self.state.velocity[2].abs();

        let ax = (fx + drag_x) / self.state.mass;
        let ay = (fy + drag_y) / self.state.mass;
        let az = (fz - self.state.mass * self.state.gravity + drag_z) / self.state.mass;

        self.state.velocity[0] += ax * dt;
        self.state.velocity[1] += ay * dt;
        self.state.velocity[2] += az * dt;

        let speed = self.state.velocity[0]
            .hypot(self.state.velocity[1])
            .hypot(self.state.velocity[2]);
        if speed > self.state.max_velocity {
            let scale = self.state.max_velocity / speed;
            self.state.velocity[0] *= scale;
            self.state.velocity[1] *= scale;
            self.state.velocity[2] *= scale;
        }

        self.state.position[0] += self.state.velocity[0] * dt;
        self.state.position[1] += self.state.velocity[1] * dt;
        self.state.position[2] += self.state.velocity[2] * dt;

        if self.state.position[2] < 0.0 {
            self.state.position[2] = 0.0;
            if self.state.velocity[2] < 0.0 {
                self.state.velocity[2] = 0.0;
            }
        }

        self.state.thrust = thrust_total;
        self.drag_mag = drag_x.hypot(drag_y).hypot(drag_z);
        self.state.energy_used_wh +=
            self.state.energy_drain * thrust_total * dt / self.state.efficiency;
    }

    pub fn telemetry(&self) -> AeroTelemetry {
        let velocity = self.state.velocity;
        let speed = velocity[0].hypot(velocity[1]).hypot(velocity[2]);
        let horiz_speed = velocity[0].hypot(velocity[1]);
        let bank_rad = self.state.roll * std::f64::consts::PI / 180.0;
        let mut turn_radius_m = f64::INFINITY;
        if horiz_speed > 1e-4 {
            let tan_bank = bank_rad.tan();
            if tan_bank.abs() > 1e-6 {
                turn_radius_m = (horiz_speed * horiz_speed) / (self.state.gravity * tan_bank.abs());
            }
        }
        let battery_percent =
            (100.0 * (1.0 - self.state.energy_used_wh / self.state.battery_capacity_wh)).max(0.0);
        AeroTelemetry {
            pitch: self.state.pitch,
            roll: self.state.roll,
            yaw: self.state.yaw,
            speed_mps: speed,
            altitude_m: self.state.position[2],
            velocity,
            thrust: self.state.thrust,
            drag: self.drag_mag,
            energy_used_wh: self.state.energy_used_wh,
            battery_percent,
            turn_radius_m,
        }
    }
}

impl Default for AeroEngine {
    fn default() -> Self {
        AeroEngine::new()
    }
}
