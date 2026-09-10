use std::f64::consts::PI;

use crate::sim::drive::{DEFAULT_SPEED, DT, MAX_PITCH, MAX_ROLL, SimState, drive_step};

fn move_axis(state: &mut SimState, dist: f64, speed: f64, sign: f64) -> f64 {
    let dist = dist / 100.0;
    let speed = speed / 100.0;
    let steps = (dist / (speed * DEFAULT_SPEED * 2.0 * DT)).ceil().max(5.0) as usize;
    let hr = state.heading * PI / 180.0;
    let pitch = sign * MAX_PITCH * speed;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        let tx = sx + sign * hr.cos() * dist * t;
        let ty = sy + sign * hr.sin() * dist * t;
        let th = state.heading;
        drive_step(state, tx, ty, sz, th, DT);
        state.push_point(th, pitch * ease, 0.0, None);
    }
    let fx = sx + sign * hr.cos() * dist;
    let fy = sy + sign * hr.sin() * dist;
    state.x = fx;
    state.y = fy;
    state.z = sz;
    let dur = steps as f64 * DT;
    let th = state.heading;
    drive_step(state, fx, fy, sz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn move_forward(state: &mut SimState, dist: f64, speed: f64) -> f64 {
    move_axis(state, dist, speed, 1.0)
}

pub fn move_backward(state: &mut SimState, dist: f64, speed: f64) -> f64 {
    move_axis(state, dist, speed, -1.0)
}

pub fn move_left(state: &mut SimState, dist: f64, speed: f64) -> f64 {
    let dist = dist / 100.0;
    let speed = speed / 100.0;
    let steps = (dist / (speed * DEFAULT_SPEED * 2.0 * DT)).ceil().max(5.0) as usize;
    let hr = state.heading * PI / 180.0;
    let roll = -MAX_ROLL * speed;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        let tx = sx + hr.sin() * dist * t;
        let ty = sy - hr.cos() * dist * t;
        let th = state.heading;
        drive_step(state, tx, ty, sz, th, DT);
        state.push_point(th, 0.0, roll * ease, None);
    }
    let fx = sx + hr.sin() * dist;
    let fy = sy - hr.cos() * dist;
    state.x = fx;
    state.y = fy;
    state.z = sz;
    let dur = steps as f64 * DT;
    let th = state.heading;
    drive_step(state, fx, fy, sz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn move_right(state: &mut SimState, dist: f64, speed: f64) -> f64 {
    let dist = dist / 100.0;
    let speed = speed / 100.0;
    let steps = (dist / (speed * DEFAULT_SPEED * 2.0 * DT)).ceil().max(5.0) as usize;
    let hr = state.heading * PI / 180.0;
    let roll = MAX_ROLL * speed;
    let sx = state.x;
    let sy = state.y;
    let sz = state.z;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        let tx = sx - hr.sin() * dist * t;
        let ty = sy + hr.cos() * dist * t;
        let th = state.heading;
        drive_step(state, tx, ty, sz, th, DT);
        state.push_point(th, 0.0, roll * ease, None);
    }
    let fx = sx - hr.sin() * dist;
    let fy = sy + hr.cos() * dist;
    state.x = fx;
    state.y = fy;
    state.z = sz;
    let dur = steps as f64 * DT;
    let th = state.heading;
    drive_step(state, fx, fy, sz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn turn_left(state: &mut SimState, deg: f64) -> f64 {
    let steps = (deg / 180.0 * 10.0).ceil().max(5.0) as usize;
    let deg_factor = (deg / 360.0).min(1.0);
    let roll_dir = -MAX_ROLL * deg_factor;
    let start_heading = state.heading;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        state.heading = start_heading - deg * t;
        let th = state.heading;
        let cx = state.x;
        let cy = state.y;
        let cz = state.z;
        drive_step(state, cx, cy, cz, th, DT);
        state.push_point(th, 0.0, roll_dir * ease, None);
    }
    let dur = deg / 180.0;
    let th = state.heading;
    let cx = state.x;
    let cy = state.y;
    let cz = state.z;
    drive_step(state, cx, cy, cz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn turn_right(state: &mut SimState, deg: f64) -> f64 {
    let steps = (deg / 180.0 * 10.0).ceil().max(5.0) as usize;
    let deg_factor = (deg / 360.0).min(1.0);
    let roll_dir = MAX_ROLL * deg_factor;
    let start_heading = state.heading;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        state.heading = start_heading + deg * t;
        let th = state.heading;
        let cx = state.x;
        let cy = state.y;
        let cz = state.z;
        drive_step(state, cx, cy, cz, th, DT);
        state.push_point(th, 0.0, roll_dir * ease, None);
    }
    let dur = deg / 180.0;
    let th = state.heading;
    let cx = state.x;
    let cy = state.y;
    let cz = state.z;
    drive_step(state, cx, cy, cz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}

pub fn turn_degree(state: &mut SimState, deg: f64, timeout: f64, p_value: f64) -> f64 {
    let _ = p_value;
    let steps = (timeout * 10.0).ceil().max(5.0) as usize;
    let deg_factor = (deg.abs() / 360.0).min(1.0);
    let roll_dir = if deg > 0.0 {
        MAX_ROLL * deg_factor
    } else {
        -MAX_ROLL * deg_factor
    };
    let start_heading = state.heading;
    for i in 0..steps {
        let t = (i + 1) as f64 / steps as f64;
        let ease = if t < 0.2 {
            t / 0.2
        } else if t > 0.8 {
            (1.0 - t) / 0.2
        } else {
            1.0
        };
        state.heading = start_heading + deg * t;
        let th = state.heading;
        let cx = state.x;
        let cy = state.y;
        let cz = state.z;
        drive_step(state, cx, cy, cz, th, DT);
        state.push_point(th, 0.0, roll_dir * ease, None);
    }
    let dur = timeout;
    let th = state.heading;
    let cx = state.x;
    let cy = state.y;
    let cz = state.z;
    drive_step(state, cx, cy, cz, th, DT);
    state.push_point(th, 0.0, 0.0, None);
    dur
}
