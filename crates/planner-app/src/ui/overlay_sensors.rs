use crate::state::AppState;
use crate::ui::viewport::{emit_octa, push_line, rgba, SceneGeom};
use planner_core::obstacles::ObstacleSet;
use planner_core::sensors::{evaluate, is_omnidirectional, ColorTable, SensorKind};

fn gl_pos(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

fn rotate_local(v: [f64; 3], heading_deg: f64) -> [f64; 3] {
    let th = heading_deg.to_radians();
    let (s, c) = th.sin_cos();
    [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn get_cone_rays(dir: [f64; 3], fov_deg: f64, num_rays: usize) -> Vec<[f64; 3]> {
    let mut rays = vec![];
    let half_angle = (fov_deg / 2.0).to_radians();
    let up = if dir[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = normalize(cross(up, dir));
    let v = cross(dir, u);
    for i in 0..num_rays {
        let t = i as f64 / num_rays as f64 * std::f64::consts::PI * 2.0;
        let (s, c) = t.sin_cos();
        let r = [
            dir[0] * half_angle.cos() + half_angle.sin() * (c * u[0] + s * v[0]),
            dir[1] * half_angle.cos() + half_angle.sin() * (c * u[1] + s * v[1]),
            dir[2] * half_angle.cos() + half_angle.sin() * (c * u[2] + s * v[2]),
        ];
        rays.push(normalize(r));
    }
    rays
}

fn sensor_color(kind: SensorKind, hit: bool) -> [f32; 4] {
    match (kind, hit) {
        (SensorKind::FrontRange, true) => rgba(0x58a6ff, 0.9),
        (SensorKind::FrontRange, false) => rgba(0x58a6ff, 0.3),
        (SensorKind::BottomRange, true) => rgba(0x39d2c0, 0.9),
        (SensorKind::BottomRange, false) => rgba(0x39d2c0, 0.3),
        (SensorKind::FrontColor, true) => rgba(0xf778ba, 0.9),
        (SensorKind::FrontColor, false) => rgba(0xf778ba, 0.3),
        (SensorKind::BackColor, true) => rgba(0xbc8cff, 0.9),
        (SensorKind::BackColor, false) => rgba(0xbc8cff, 0.3),
        (SensorKind::Temperature, _) => rgba(0xf0883e, 0.9),
        (SensorKind::Battery, _) => rgba(0x3fb950, 0.9),
    }
}

pub fn build(g: &mut SceneGeom, state: &AppState) {
    let frame = state.current_frame();
    if frame.is_none() {
        return;
    }
    let frame = frame.unwrap();

    let active_id = state
        .active_drone_index()
        .map(|i| state.plan.drones[i].id.clone());
    let point = active_id
        .as_ref()
        .and_then(|id| frame.positions.get(id))
        .or_else(|| frame.positions.values().next());

    if point.is_none() {
        return;
    }
    let point = point.unwrap();

    let obs_set = ObstacleSet {
        obstacles: state.obstacle_store().clone(),
        boundary: state.obstacles.boundary.clone(),
    };
    let surfaces = ColorTable::new();

    let pos = [point.x, point.z, point.y];
    let heading = point.heading;

    for sensor in &state.sensor_set.sensors {
        if !sensor.enabled {
            continue;
        }

        let reading = evaluate(sensor, pos, heading, &obs_set, &surfaces);
        let color = sensor_color(sensor.kind, reading.hit);

        if is_omnidirectional(sensor.kind) {
            let mut gl_p = gl_pos(pos);
            if sensor.kind == SensorKind::Temperature {
                gl_p[1] += 0.1;
            } else {
                gl_p[1] += 0.15;
            }
            emit_octa(g, gl_p, 0.03, color);
        } else {
            let rotated_mount = rotate_local(sensor.mount_position, heading);
            let mount_world = [
                pos[0] + rotated_mount[0],
                pos[1] + rotated_mount[1],
                pos[2] + rotated_mount[2],
            ];

            let mut dir_world = rotate_local(sensor.facing, heading);
            if matches!(sensor.kind, SensorKind::FrontColor | SensorKind::BackColor) {
                dir_world = [0.0, -1.0, 0.0];
            }
            let dir_world = normalize(dir_world);

            let mut draw_dist = match sensor.kind {
                SensorKind::FrontColor | SensorKind::BackColor => 3.0,
                _ => sensor.range_m,
            };

            if reading.hit {
                if let Some(d) = reading.distance_m {
                    draw_dist = d;
                } else if let Some(hp) = reading.hit_point {
                    let dx = hp[0] - mount_world[0];
                    let dy = hp[1] - mount_world[1];
                    let dz = hp[2] - mount_world[2];
                    draw_dist = (dx * dx + dy * dy + dz * dz).sqrt();
                }
            }

            let gl_mount = gl_pos(mount_world);

            if sensor.fov_deg <= 0.0 {
                let end = [
                    mount_world[0] + dir_world[0] * draw_dist,
                    mount_world[1] + dir_world[1] * draw_dist,
                    mount_world[2] + dir_world[2] * draw_dist,
                ];
                push_line(g, gl_mount, gl_pos(end), color);
            } else {
                let rays = get_cone_rays(dir_world, sensor.fov_deg, 8);
                for r in rays {
                    let end = [
                        mount_world[0] + r[0] * draw_dist,
                        mount_world[1] + r[1] * draw_dist,
                        mount_world[2] + r[2] * draw_dist,
                    ];
                    push_line(g, gl_mount, gl_pos(end), color);
                }
                if reading.hit {
                    if let Some(hp) = reading.hit_point {
                        push_line(g, gl_mount, gl_pos(hp), color);
                    }
                }
            }

            if reading.hit {
                if let Some(hp) = reading.hit_point {
                    emit_octa(g, gl_pos(hp), 0.04, color);
                }
            }
        }
    }
}
