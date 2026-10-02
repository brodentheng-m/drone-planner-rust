use crate::state::AppState;
use crate::ui::viewport::{SceneGeom, emit_octa, push_line, rgba};

pub fn build(g: &mut SceneGeom, state: &AppState) {
    let start_gl = [
        state.route_start_pos[0] as f32,
        state.route_start_pos[1] as f32,
        state.route_start_pos[2] as f32,
    ];

    let mut start_color = rgba(0xbc8cff, 0.9);
    if let Some(res) = &state.route_result {
        if !res.feasible {
            start_color = rgba(0x8b949e, 0.6);
        }
    }

    emit_octa(g, start_gl, 0.06, start_color);

    let hdg_rad = (state.route_start_heading as f32).to_radians();
    let hdx = hdg_rad.cos() * 0.4;
    let hdz = hdg_rad.sin() * 0.4;
    let hdg_end = [
        start_gl[0] + hdx,
        start_gl[1],
        start_gl[2] + hdz,
    ];
    push_line(g, start_gl, hdg_end, start_color);

    let wp_len = state.route_waypoints.len();
    for (i, wp) in state.route_waypoints.iter().enumerate() {
        let mut color = if i == 0 {
            rgba(0x39d2c0, 0.9)
        } else if i == wp_len - 1 {
            rgba(0x58a6ff, 0.9)
        } else {
            rgba(0xd29922, 0.9)
        };
        if let Some(res) = &state.route_result {
            if !res.feasible {
                color = rgba(0x8b949e, 0.6);
            }
        }
        emit_octa(g, [wp[0] as f32, wp[2] as f32, wp[1] as f32], 0.08, color);
    }

    if let Some(res) = &state.route_result {
        if res.feasible {
            let path_color = rgba(0xbc8cff, 0.9);
            for pair in res.path.windows(2) {
                let a = &pair[0];
                let b = &pair[1];
                push_line(
                    g,
                    [a[0] as f32, a[2] as f32, a[1] as f32],
                    [b[0] as f32, b[2] as f32, b[1] as f32],
                    path_color,
                );
            }
        }
    }
}
