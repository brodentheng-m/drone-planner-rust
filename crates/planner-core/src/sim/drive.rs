use crate::aero::{AeroControl, AeroEngine};
use crate::golden::{
    Collision, CollisionObstacle, EulerRotation, LedValue, Point, Position, WorldBox,
};
use crate::obstacles::{COLLISION_DRONE_SIZE, ObstacleSet};

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

fn clamp(v: f64, lo: f64, hi: f64) -> f64 {
    v.max(lo).min(hi)
}

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
        if let Some(l) = led {
            self.led_color = l;
        }
        let tel = self.engine.telemetry();
        let point = Point {
            x: self.engine.state.position[0],
            y: self.engine.state.position[1],
            z: self.engine.state.position[2],
            heading,
            pitch,
            roll,
            speed: tel.speed_mps,
            energy_used: tel.energy_used_wh,
            battery_percent: tel.battery_percent,
            turn_radius_m: if tel.turn_radius_m.is_finite() {
                Some(tel.turn_radius_m)
            } else {
                None
            },
            led: self.led_color.clone(),
        };
        self.points.push(point);
    }

    pub fn record_collision(&mut self, collision: Collision) {
        self.collisions.push(collision);
    }
}

pub fn drive_step(state: &mut SimState, tx: f64, ty: f64, tz: f64, th: f64, dt: f64) {
    let control = compute_control(&state.engine, tx, ty, tz, th, dt);
    state.engine.step(control.throttle, &control, dt);
    state.x = state.engine.state.position[0];
    state.y = state.engine.state.position[1];
    state.z = state.engine.state.position[2];
    state.heading = th;
}

pub fn compute_control(
    engine: &AeroEngine,
    tx: f64,
    ty: f64,
    tz: f64,
    th: f64,
    dt: f64,
) -> AeroControl {
    let g = engine.state.gravity;
    let v = engine.state.velocity;
    let position = engine.state.position;

    let horiz_pos_gain = 2.5;
    let horiz_vel_gain = 4.0;

    let err_x = tx - position[0];
    let err_y = ty - position[1];

    let mut des_vx = horiz_pos_gain * err_x;
    let mut des_vy = horiz_pos_gain * err_y;
    let des_speed = (des_vx * des_vx + des_vy * des_vy).sqrt();
    if des_speed > engine.state.max_velocity {
        let scale = engine.state.max_velocity / des_speed;
        des_vx *= scale;
        des_vy *= scale;
    }

    let ax = (des_vx - v[0]) * horiz_vel_gain;
    let ay = (des_vy - v[1]) * horiz_vel_gain;

    let yaw_rad = th * std::f64::consts::PI / 180.0;
    let fwd_x = yaw_rad.cos();
    let fwd_y = yaw_rad.sin();
    let right_x = -yaw_rad.sin();
    let right_y = yaw_rad.cos();

    let a_fwd = ax * fwd_x + ay * fwd_y;
    let a_right = ax * right_x + ay * right_y;

    let pitch_deg = clamp(
        -a_fwd.atan2(g) * 180.0 / std::f64::consts::PI,
        -MAX_PITCH,
        MAX_PITCH,
    );
    let roll_deg = clamp(
        a_right.atan2(g) * 180.0 / std::f64::consts::PI,
        -MAX_ROLL,
        MAX_ROLL,
    );

    let yaw_rate = clamp((th - engine.state.yaw) / dt, -MAX_YAW_RATE, MAX_YAW_RATE);

    AeroControl {
        throttle: 0.0,
        pitch: pitch_deg,
        roll: roll_deg,
        yaw: yaw_rate,
        target_altitude: Some(tz),
    }
}

pub fn takeoff(state: &mut SimState) -> f64 {
    let steps = ((TAKEOFF_HEIGHT / (DEFAULT_SPEED * DT)).ceil() as usize).max(10);
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let tz = TAKEOFF_HEIGHT * t;
        drive_step(state, state.x, state.y, tz, state.heading, DT);
        let pitch = if t < 0.5 {
            -MAX_PITCH * (1.0 - t * 2.0)
        } else {
            0.0
        };
        state.push_point(state.heading, pitch, 0.0, None);
    }
    state.flying = true;
    steps as f64 * DT
}

pub fn land(state: &mut SimState) -> f64 {
    let start_z = state.z;
    let steps = ((start_z / (DEFAULT_SPEED * DT)).ceil() as usize).max(10);
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let tz = start_z * (1.0 - t);
        drive_step(state, state.x, state.y, tz, state.heading, DT);
        let pitch = if t > 0.5 {
            MAX_PITCH * ((t - 0.5) * 2.0)
        } else {
            0.0
        };
        state.push_point(state.heading, pitch, 0.0, None);
    }
    state.flying = false;
    state.z = 0.0;
    state.engine.state.position[2] = 0.0;
    steps as f64 * DT
}

pub fn hover(state: &mut SimState, dur: f64) -> f64 {
    let steps = (dur / DT).ceil() as usize;
    let hx = state.x;
    let hy = state.y;
    let hz = state.z;
    for _ in 0..steps {
        drive_step(state, hx, hy, hz, state.heading, DT);
        state.push_point(state.heading, 0.0, 0.0, None);
    }
    dur
}

pub fn flip(state: &mut SimState, dir: &str) -> f64 {
    let hr = state.heading * std::f64::consts::PI / 180.0;
    let hx = state.x;
    let hy = state.y;
    let hz = state.z;
    let cos_h = hr.cos();
    let sin_h = hr.sin();
    let (dir_x, dir_y) = match dir {
        "forward" => (cos_h, sin_h),
        "back" => (-cos_h, -sin_h),
        "left" => (sin_h, -cos_h),
        _ => (-sin_h, cos_h),
    };
    let t_pop = 1.0 / 6.0;
    let t_whip = 1.0 / 3.0;
    let t_snap = 5.0 / 6.0;
    for i in 1..=FLIP_POINTS {
        let t = i as f64 / FLIP_POINTS as f64;
        let horiz = FLIP_TRAVEL * (std::f64::consts::PI * t).sin();
        let tx = hx + dir_x * horiz;
        let ty = hy + dir_y * horiz;
        let tz = (hz + FLIP_CLIMB * 4.0 * t * (1.0 - t)).max(0.0);
        let theta = if t <= t_pop {
            0.0
        } else if t <= t_whip {
            90.0 * (t - t_pop) / (t_whip - t_pop)
        } else if t <= t_snap {
            90.0 + 180.0 * (t - t_whip) / (t_snap - t_whip)
        } else {
            270.0 + 90.0 * ((t - t_snap) / (1.0 - t_snap)).sqrt()
        };
        let (pitch, roll) = match dir {
            "back" => (theta, 0.0),
            "forward" => (-theta, 0.0),
            "left" => (0.0, theta),
            _ => (0.0, -theta),
        };
        drive_step(state, tx, ty, tz, state.heading, DT);
        state.push_point(state.heading, pitch, roll, None);
    }
    state.x = hx;
    state.y = hy;
    state.z = hz;
    for _ in 0..FLIP_SETTLE {
        drive_step(state, hx, hy, hz, state.heading, DT);
        state.push_point(state.heading, 0.0, 0.0, None);
    }
    (FLIP_POINTS + FLIP_SETTLE) as f64 * DT
}

pub fn go(
    state: &mut SimState,
    obstacles: Option<&ObstacleSet>,
    dir: &str,
    power: f64,
    dur: f64,
) -> f64 {
    let speed = (power / 100.0) * DEFAULT_SPEED * 2.0;
    let steps = ((dur / DT).ceil() as usize).max(5);
    let hr = state.heading * std::f64::consts::PI / 180.0;
    let mut dx = 0.0;
    let mut dy = 0.0;
    let mut dz = 0.0;
    let mut pitch = 0.0;
    let mut roll = 0.0;
    let pf = power / 100.0;
    match dir {
        "forward" => {
            dx = hr.cos();
            dy = hr.sin();
            pitch = -MAX_PITCH * pf;
        }
        "backward" => {
            dx = -hr.cos();
            dy = -hr.sin();
            pitch = MAX_PITCH * pf;
        }
        "left" => {
            dx = hr.sin();
            dy = -hr.cos();
            roll = -MAX_ROLL * pf;
        }
        "right" => {
            dx = -hr.sin();
            dy = hr.cos();
            roll = MAX_ROLL * pf;
        }
        "up" => {
            dz = speed * dur;
        }
        "down" => {
            dz = -speed * dur;
        }
        _ => {}
    }

    let start_x = state.x;
    let start_y = state.y;
    let start_z = state.z;
    let mut collision_occurred = false;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let mut ease = 1.0;
        if t < 0.2 {
            ease = t / 0.2;
        } else if t > 0.8 {
            ease = (1.0 - t) / 0.2;
        }

        let new_x = start_x + dx * speed * dur * t;
        let new_y = start_y + dy * speed * dur * t;
        let new_z = start_z + dz * t;

        if obstacles.is_some() && !collision_occurred {
            let set = obstacles.unwrap();
            if let Some(hit) = set.check_collision(new_x, new_y, new_z, COLLISION_DRONE_SIZE) {
                let obstacle = &hit.obstacle;
                state.collisions.push(Collision {
                    position: Position {
                        x: new_x,
                        y: new_y,
                        z: new_z,
                    },
                    obstacle: CollisionObstacle {
                        id: obstacle.id.clone(),
                        obstacle_type: obstacle.obstacle_type.clone(),
                        position: Position {
                            x: obstacle.position[0],
                            y: obstacle.position[1],
                            z: obstacle.position[2],
                        },
                        rotation: EulerRotation {
                            is_euler: true,
                            x: obstacle.rotation[0],
                            y: obstacle.rotation[1],
                            z: obstacle.rotation[2],
                            order: "XYZ".to_string(),
                        },
                        name: obstacle.name.clone(),
                        scale: Position {
                            x: obstacle.scale[0],
                            y: obstacle.scale[1],
                            z: obstacle.scale[2],
                        },
                        world_box: WorldBox {
                            is_box3: true,
                            min: Position {
                                x: hit.world_box.min[0],
                                y: hit.world_box.min[1],
                                z: hit.world_box.min[2],
                            },
                            max: Position {
                                x: hit.world_box.max[0],
                                y: hit.world_box.max[1],
                                z: hit.world_box.max[2],
                            },
                        },
                    },
                });
                collision_occurred = true;
            }
        }

        drive_step(state, new_x, new_y, new_z, state.heading, DT);
        state.push_point(state.heading, pitch * ease, roll * ease, None);
    }

    if !collision_occurred {
        state.x = start_x + dx * speed * dur;
        state.y = start_y + dy * speed * dur;
        state.z = start_z + dz;
        state.engine.state.position = [state.x, state.y, state.z];
    }
    state.push_point(state.heading, 0.0, 0.0, None);
    dur
}
