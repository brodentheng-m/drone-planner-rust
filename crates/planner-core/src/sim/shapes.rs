use std::f64::consts::PI;

use crate::obstacles::ObstacleSet;
use crate::sensors::{default_sensor, evaluate, ColorTable, SensorKind};
use crate::sim::drive::{DEFAULT_SPEED, DT, MAX_PITCH, MAX_ROLL, SimState, drive_step};
use crate::sim::runtime::{RuntimeState, VarValue};

const AVOID_WALL_DEADBAND_CM: f64 = 20.0;
const AVOID_WALL_KP: f64 = 0.4;
const AVOID_WALL_STABLE_STEPS: usize = 4;
const MIN_CLEARANCE_M: f64 = 0.15;

pub fn circle(state: &mut SimState, speed: f64, dir: &str) -> f64 {
    let speed = speed / 100.0;
    let direction = if dir == "counter-clockwise" {
        -1.0
    } else {
        1.0
    };
    let radius = 0.5;
    let steps = 60usize;
    let cx = state.x;
    let cy = state.y;
    let cz = state.z;
    for i in 0..steps {
        let angle = 2.0 * PI * i as f64 / steps as f64 * direction;
        let tx = cx + radius * angle.cos();
        let ty = cy + radius * angle.sin();
        let th = state.heading;
        drive_step(state, tx, ty, cz, th, DT);
        state.push_point(th, 0.0, -MAX_ROLL * speed * (angle * direction).sin(), None);
    }
    let fx = cx + radius * direction;
    state.x = fx;
    state.y = cy;
    state.z = cz;
    let th = state.heading;
    drive_step(state, fx, cy, cz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    3.0 * speed
}

fn polygon(state: &mut SimState, speed: f64, secs: f64, dir: &str, num_sides: usize) -> f64 {
    let speed = speed / 100.0;
    let direction = if dir == "counter-clockwise" {
        -1.0
    } else {
        1.0
    };
    let side = 0.5;
    let steps_per_side = 15usize;
    let hr = state.heading * PI / 180.0;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    let mut acc_x = 0.0;
    let mut acc_y = 0.0;
    for si in 0..num_sides {
        let angle = hr + (si as f64 * 2.0 * PI / num_sides as f64) * direction;
        let ddx = angle.cos();
        let ddy = angle.sin();
        for i in 0..steps_per_side {
            let t = (i + 1) as f64 / steps_per_side as f64;
            let ease = if t < 0.2 {
                t / 0.2
            } else if t > 0.8 {
                (1.0 - t) / 0.2
            } else {
                1.0
            };
            let tx = sx + acc_x + ddx * side * t;
            let ty = sy + acc_y + ddy * side * t;
            let th = state.heading;
            drive_step(state, tx, ty, sz, th, DT);
            state.push_point(th, -MAX_PITCH * speed * ease, 0.0, None);
        }
        acc_x += ddx * side;
        acc_y += ddy * side;
    }
    let fx = sx + acc_x;
    let fy = sy + acc_y;
    state.x = fx;
    state.y = fy;
    state.z = sz;
    let dur = num_sides as f64 * secs * speed;
    let th = state.heading;
    drive_step(state, fx, fy, sz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn square(state: &mut SimState, speed: f64, secs: f64, dir: &str) -> f64 {
    polygon(state, speed, secs, dir, 4)
}

pub fn triangle(state: &mut SimState, speed: f64, secs: f64, dir: &str) -> f64 {
    polygon(state, speed, secs, dir, 3)
}

pub fn spiral(state: &mut SimState, speed: f64, dir: &str) -> f64 {
    let speed = speed / 100.0;
    let direction = if dir == "counter-clockwise" {
        -1.0
    } else {
        1.0
    };
    let steps = 120usize;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = i as f64 / steps as f64;
        let angle = 4.0 * PI * t * direction;
        let radius = t * 0.5;
        let tx = sx + radius * angle.cos();
        let ty = sy + radius * angle.sin();
        let tz = sz + t * 0.3;
        let th = state.heading;
        drive_step(state, tx, ty, tz, th, DT);
        state.push_point(th, 0.0, -MAX_ROLL * speed * angle.sin(), None);
    }
    let fx = sx + 0.5 * (4.0 * PI * direction).cos();
    let fy = sy + 0.5 * (4.0 * PI * direction).sin();
    let fz = sz + 0.3;
    state.x = fx;
    state.y = fy;
    state.z = fz;
    let th = state.heading;
    drive_step(state, fx, fy, fz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    5.0 * speed
}

pub fn sway(state: &mut SimState, speed: f64, dir: &str) -> f64 {
    let dir = if dir.is_empty() { "forward-back" } else { dir };
    let speed = speed / 100.0;
    let steps = 40usize;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = i as f64 / steps as f64;
        let angle = 2.0 * PI * t;
        let mut pitch = 0.0;
        let mut roll = 0.0;
        if dir == "forward-back" {
            pitch = MAX_PITCH * speed * angle.sin();
        } else if dir == "left-right" {
            roll = MAX_ROLL * speed * angle.sin();
        } else if dir == "pitch-forward" {
            pitch = MAX_PITCH * speed;
        } else if dir == "pitch-backward" {
            pitch = -MAX_PITCH * speed;
        } else if dir == "roll-left" {
            roll = -MAX_ROLL * speed;
        } else if dir == "roll-right" {
            roll = MAX_ROLL * speed;
        }
        let th = state.heading;
        drive_step(state, sx, sy, sz, th, DT);
        state.push_point(th, pitch, roll, None);
    }
    let dur = 2.0 * speed;
    let th = state.heading;
    let cx = state.x;
    let cy = state.y;
    let cz = state.z;
    drive_step(state, cx, cy, cz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

fn retreat(state: &mut SimState, speed: f64, dist: f64) -> f64 {
    let dist = dist / 100.0;
    let speed = speed / 100.0;
    let steps = (dist / (speed * DEFAULT_SPEED * DT)).ceil().max(5.0) as usize;
    let hr = state.heading * PI / 180.0;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let tx = sx - hr.cos() * dist * t;
        let ty = sy - hr.sin() * dist * t;
        let th = state.heading;
        drive_step(state, tx, ty, sz, th, DT);
        state.push_point(th, 0.0, 0.0, None);
    }
    let fx = sx - hr.cos() * dist;
    let fy = sy - hr.sin() * dist;
    state.x = fx;
    state.y = fy;
    state.z = sz;
    let dur = steps as f64 * DT;
    let th = state.heading;
    drive_step(state, fx, fy, sz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn keep_distance(
    state: &mut SimState,
    obstacles: Option<&ObstacleSet>,
    speed: f64,
    dist: f64,
    timeout: f64,
) -> f64 {
    let obstacles = match obstacles {
        Some(obs) => obs,
        None => return retreat(state, speed, dist),
    };

    if timeout <= 0.0 {
        return 0.0;
    }

    let max_steps = ((timeout / DT).ceil() as usize).max(1);
    let front_sensor = default_sensor(SensorKind::FrontRange);
    let surfaces = ColorTable::new();
    let hr = state.heading * PI / 180.0;
    let cos_h = hr.cos();
    let sin_h = hr.sin();
    let nominal_speed = DEFAULT_SPEED * 2.0;

    for _ in 0..max_steps {
        let probe = [state.x, state.z, state.y];
        let reading = evaluate(&front_sensor, probe, state.heading, obstacles, &surfaces);
        let current_dist_cm = if reading.hit {
            reading.distance_m.unwrap_or(1.0) * 100.0
        } else {
            100.0
        };

        let error_cm = current_dist_cm - dist;
        if reading.hit && error_cm.abs() <= 10.0 {
            drive_step(state, state.x, state.y, state.z, state.heading, DT);
            state.push_point(state.heading, 0.0, 0.0, None);
        } else {
            let error_pct = error_cm.clamp(-100.0, 100.0);
            let pitch_speed = error_pct * AVOID_WALL_KP;
            let v = (speed / 100.0) * (pitch_speed / 100.0) * nominal_speed;
            let pitch = -MAX_PITCH * (pitch_speed / 100.0).clamp(-1.0, 1.0);
            let next_x = state.x + cos_h * v * DT;
            let next_y = state.y + sin_h * v * DT;
            drive_step(state, next_x, next_y, state.z, state.heading, DT);
            state.push_point(state.heading, pitch, 0.0, None);
        }
    }
    max_steps as f64 * DT
}

pub fn avoid_wall(
    state: &mut SimState,
    obstacles: Option<&ObstacleSet>,
    speed: f64,
    dist: f64,
    timeout: f64,
) -> f64 {
    let obstacles = match obstacles {
        Some(obs) => obs,
        None => return retreat(state, speed, dist),
    };

    if timeout <= 0.0 {
        let th = state.heading;
        drive_step(state, state.x, state.y, state.z, th, DT);
        state.push_point(th, 0.0, 0.0, None);
        return 0.0;
    }

    let max_steps = ((timeout / DT).ceil() as usize).max(1);
    let target_dist_cm = dist;
    let front_sensor = default_sensor(SensorKind::FrontRange);
    let surfaces = ColorTable::new();
    let hr = state.heading * PI / 180.0;
    let cos_h = hr.cos();
    let sin_h = hr.sin();
    let nominal_speed = DEFAULT_SPEED * 2.0;

    let mut stable_count = 0usize;
    let mut steps_executed = 0usize;

    for _ in 0..max_steps {
        steps_executed += 1;
        let probe = [state.x, state.z, state.y];
        let reading = evaluate(&front_sensor, probe, state.heading, obstacles, &surfaces);

        let (current_dist_cm, hit) = if reading.hit {
            (reading.distance_m.unwrap_or(1.0) * 100.0, true)
        } else {
            (100.0, false)
        };

        let error_cm = if hit {
            current_dist_cm - target_dist_cm
        } else {
            100.0 - target_dist_cm
        };

        if hit && error_cm.abs() <= AVOID_WALL_DEADBAND_CM {
            stable_count += 1;
            let th = state.heading;
            drive_step(state, state.x, state.y, state.z, th, DT);
            state.push_point(th, 0.0, 0.0, None);
            if stable_count >= AVOID_WALL_STABLE_STEPS {
                break;
            }
        } else {
            stable_count = 0;
            let error_pct = error_cm.clamp(-100.0, 100.0);
            let pitch_speed = error_pct * AVOID_WALL_KP;
            let v = (speed / 100.0) * (pitch_speed / 100.0) * nominal_speed;
            let pitch = -MAX_PITCH * (pitch_speed / 100.0).clamp(-1.0, 1.0);

            let mut step_dist = v * DT;
            if v > 0.0 && hit {
                let clearance_m = (current_dist_cm / 100.0) - MIN_CLEARANCE_M;
                if step_dist > clearance_m.max(0.0) {
                    step_dist = clearance_m.max(0.0);
                }
            }

            let next_x = state.x + cos_h * step_dist;
            let next_y = state.y + sin_h * step_dist;
            let th = state.heading;
            drive_step(state, next_x, next_y, state.z, th, DT);
            state.push_point(th, pitch, 0.0, None);
        }
    }

    let th = state.heading;
    drive_step(state, state.x, state.y, state.z, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    (steps_executed + 1) as f64 * DT
}

pub fn detect_wall(
    state: &mut SimState,
    obstacles: Option<&ObstacleSet>,
    var: &str,
    threshold_cm: f64,
    runtime: &mut RuntimeState,
) -> f64 {
    let detected = match obstacles {
        Some(obs) => {
            let sensor = default_sensor(SensorKind::FrontRange);
            let surfaces = ColorTable::new();
            let probe = [state.x, state.z, state.y];
            let reading = evaluate(&sensor, probe, state.heading, obs, &surfaces);
            if reading.hit {
                let d_cm = reading.distance_m.unwrap_or(10.0) * 100.0;
                if d_cm <= threshold_cm {
                    1.0
                } else {
                    0.0
                }
            } else {
                0.0
            }
        }
        None => 0.0,
    };
    runtime.vars.insert(var.to_string(), VarValue::Num(detected));
    0.1
}
