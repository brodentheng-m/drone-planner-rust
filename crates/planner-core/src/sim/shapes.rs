use std::f64::consts::PI;

use crate::sim::drive::{DEFAULT_SPEED, DT, MAX_PITCH, MAX_ROLL, SimState, drive_step};
use crate::sim::runtime::{RuntimeState, VarValue};

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

pub fn keep_distance(state: &mut SimState, speed: f64, dist: f64) -> f64 {
    retreat(state, speed, dist)
}

pub fn avoid_wall(state: &mut SimState, speed: f64, dist: f64) -> f64 {
    retreat(state, speed, dist)
}

pub fn detect_wall(state: &mut SimState, var: &str, runtime: &mut RuntimeState) -> f64 {
    let _ = state;
    runtime.vars.insert(var.to_string(), VarValue::Num(0.0));
    0.1
}
