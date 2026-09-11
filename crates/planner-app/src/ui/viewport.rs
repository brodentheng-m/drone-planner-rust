use crate::state::{AppState, FrameState};
use eframe::egui_glow;
use eframe::glow::{self, HasContext};
use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Ui};
use planner_core::golden::LedValue;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

const MIN_PITCH: f32 = std::f32::consts::FRAC_PI_2 - std::f32::consts::PI / 2.1;
const MAX_PITCH: f32 = 1.55;
const MIN_DISTANCE: f32 = 0.5;
const MAX_DISTANCE: f32 = 60.0;
const FT: f32 = 0.3048;
const GRID_SIZE: f32 = 20.0;
const FOV_Y: f32 = 50.0;
const BG: [f32; 4] = [0.05098, 0.06667, 0.0902, 1.0];
const FOG_NEAR: f32 = 20.0;
const FOG_FAR: f32 = 50.0;

const VERT_SRC: &str = r#"#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec4 a_col;
uniform mat4 u_proj;
uniform mat4 u_view;
out vec4 v_col;
out float v_dist;
void main() {
    vec4 view_pos = u_view * vec4(a_pos, 1.0);
    v_dist = length(view_pos.xyz);
    v_col = a_col;
    gl_Position = u_proj * view_pos;
}
"#;

const FRAG_SRC: &str = r#"#version 330 core
in vec4 v_col;
in float v_dist;
uniform vec3 u_fog_color;
uniform float u_fog_near;
uniform float u_fog_far;
out vec4 f_color;
void main() {
    float f = clamp((v_dist - u_fog_near) / (u_fog_far - u_fog_near), 0.0, 1.0);
    f_color = vec4(mix(v_col.rgb, u_fog_color, f), v_col.a);
}
"#;

pub struct ViewportPanel {
    pub mode: u8,
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: [f32; 3],
    prev_mode: u8,
    follow_offset: Option<[f32; 3]>,
    prev_scrub: f64,
    seen_generation: u64,
    trail_signature: u64,
    trails: BTreeMap<String, Vec<[f32; 3]>>,
    spin: f32,
    waypoint_hits: Vec<(Pos2, usize)>,
    gl: Arc<Mutex<GlState>>,
}

#[derive(Default)]
struct GlState {
    program: Option<glow::NativeProgram>,
    vao: Option<glow::NativeVertexArray>,
    vbo: Option<glow::NativeBuffer>,
    u_proj: Option<glow::NativeUniformLocation>,
    u_view: Option<glow::NativeUniformLocation>,
    u_fog_color: Option<glow::NativeUniformLocation>,
    u_fog_near: Option<glow::NativeUniformLocation>,
    u_fog_far: Option<glow::NativeUniformLocation>,
    error: Option<String>,
}

struct FrameDraw {
    proj: [f32; 16],
    view: [f32; 16],
    verts: Vec<f32>,
    line_verts: i32,
    total_verts: i32,
}

fn compile_stage(gl: &glow::Context, kind: u32, src: &str) -> Result<glow::NativeShader, String> {
    unsafe {
        let shader = gl.create_shader(kind)?;
        gl.shader_source(shader, src);
        gl.compile_shader(shader);
        if !gl.get_shader_compile_status(shader) {
            let log = gl.get_shader_info_log(shader);
            gl.delete_shader(shader);
            return Err(log);
        }
        Ok(shader)
    }
}

fn gl_init(gl: &glow::Context, st: &mut GlState) -> Result<(), String> {
    unsafe {
        let vs = compile_stage(gl, glow::VERTEX_SHADER, VERT_SRC)?;
        let fs = compile_stage(gl, glow::FRAGMENT_SHADER, FRAG_SRC)?;
        let program = gl.create_program()?;
        gl.attach_shader(program, vs);
        gl.attach_shader(program, fs);
        gl.link_program(program);
        gl.detach_shader(program, vs);
        gl.detach_shader(program, fs);
        gl.delete_shader(vs);
        gl.delete_shader(fs);
        if !gl.get_program_link_status(program) {
            return Err(gl.get_program_info_log(program));
        }
        let vao = gl.create_vertex_array()?;
        let vbo = gl.create_buffer()?;
        gl.bind_vertex_array(Some(vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 28, 0);
        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(1, 4, glow::FLOAT, false, 28, 12);
        st.u_proj = gl.get_uniform_location(program, "u_proj");
        st.u_view = gl.get_uniform_location(program, "u_view");
        st.u_fog_color = gl.get_uniform_location(program, "u_fog_color");
        st.u_fog_near = gl.get_uniform_location(program, "u_fog_near");
        st.u_fog_far = gl.get_uniform_location(program, "u_fog_far");
        st.program = Some(program);
        st.vao = Some(vao);
        st.vbo = Some(vbo);
        Ok(())
    }
}

struct GlSaved {
    program: Option<glow::NativeProgram>,
    vao: Option<glow::NativeVertexArray>,
    vbo: Option<glow::NativeBuffer>,
    blend: bool,
    depth: bool,
    scissor: bool,
    cull: bool,
    depth_mask: bool,
    depth_func: i32,
    viewport: [i32; 4],
    scissor_box: [i32; 4],
    clear_color: [f32; 4],
    blend_func: [i32; 4],
    blend_eq: [i32; 2],
}

fn set_enabled(gl: &glow::Context, what: u32, on: bool) {
    unsafe {
        if on {
            gl.enable(what);
        } else {
            gl.disable(what);
        }
    }
}

fn gl_save(gl: &glow::Context) -> GlSaved {
    let mut viewport = [0i32; 4];
    let mut scissor_box = [0i32; 4];
    let mut clear_color = [0.0f32; 4];
    unsafe {
        gl.get_parameter_i32_slice(glow::VIEWPORT, &mut viewport);
        gl.get_parameter_i32_slice(glow::SCISSOR_BOX, &mut scissor_box);
        gl.get_parameter_f32_slice(glow::COLOR_CLEAR_VALUE, &mut clear_color);
        GlSaved {
            program: std::num::NonZeroU32::new(gl.get_parameter_i32(glow::CURRENT_PROGRAM) as u32)
                .map(glow::NativeProgram),
            vao: std::num::NonZeroU32::new(gl.get_parameter_i32(glow::VERTEX_ARRAY_BINDING) as u32)
                .map(glow::NativeVertexArray),
            vbo: std::num::NonZeroU32::new(gl.get_parameter_i32(glow::ARRAY_BUFFER_BINDING) as u32)
                .map(glow::NativeBuffer),
            blend: gl.is_enabled(glow::BLEND),
            depth: gl.is_enabled(glow::DEPTH_TEST),
            scissor: gl.is_enabled(glow::SCISSOR_TEST),
            cull: gl.is_enabled(glow::CULL_FACE),
            depth_mask: gl.get_parameter_i32(glow::DEPTH_WRITEMASK) != 0,
            depth_func: gl.get_parameter_i32(glow::DEPTH_FUNC),
            viewport,
            scissor_box,
            clear_color,
            blend_func: [
                gl.get_parameter_i32(glow::BLEND_SRC_RGB),
                gl.get_parameter_i32(glow::BLEND_DST_RGB),
                gl.get_parameter_i32(glow::BLEND_SRC_ALPHA),
                gl.get_parameter_i32(glow::BLEND_DST_ALPHA),
            ],
            blend_eq: [
                gl.get_parameter_i32(glow::BLEND_EQUATION_RGB),
                gl.get_parameter_i32(glow::BLEND_EQUATION_ALPHA),
            ],
        }
    }
}

fn gl_restore(gl: &glow::Context, saved: GlSaved) {
    unsafe {
        gl.use_program(saved.program);
        gl.bind_vertex_array(saved.vao);
        gl.bind_buffer(glow::ARRAY_BUFFER, saved.vbo);
        set_enabled(gl, glow::BLEND, saved.blend);
        set_enabled(gl, glow::DEPTH_TEST, saved.depth);
        set_enabled(gl, glow::SCISSOR_TEST, saved.scissor);
        set_enabled(gl, glow::CULL_FACE, saved.cull);
        gl.depth_mask(saved.depth_mask);
        gl.depth_func(saved.depth_func as u32);
        gl.viewport(
            saved.viewport[0],
            saved.viewport[1],
            saved.viewport[2],
            saved.viewport[3],
        );
        gl.scissor(
            saved.scissor_box[0],
            saved.scissor_box[1],
            saved.scissor_box[2],
            saved.scissor_box[3],
        );
        gl.clear_color(
            saved.clear_color[0],
            saved.clear_color[1],
            saved.clear_color[2],
            saved.clear_color[3],
        );
        gl.blend_func_separate(
            saved.blend_func[0] as u32,
            saved.blend_func[1] as u32,
            saved.blend_func[2] as u32,
            saved.blend_func[3] as u32,
        );
        gl.blend_equation_separate(saved.blend_eq[0] as u32, saved.blend_eq[1] as u32);
    }
}

fn paint_gl(gl: &glow::Context, st: &mut GlState, info: &egui::PaintCallbackInfo, draw: &FrameDraw) {
    let saved = gl_save(gl);
    unsafe {
        let vp = info.viewport_in_pixels();
        gl.viewport(vp.left_px, vp.from_bottom_px, vp.width_px, vp.height_px);
        let clip = info.clip_rect_in_pixels();
        let scissor_left = vp.left_px.max(clip.left_px);
        let scissor_right = (vp.left_px + vp.width_px).min(clip.left_px + clip.width_px);
        let scissor_bottom = vp.from_bottom_px.max(clip.from_bottom_px);
        let scissor_top = (vp.from_bottom_px + vp.height_px).min(clip.from_bottom_px + clip.height_px);
        gl.enable(glow::SCISSOR_TEST);
        gl.scissor(
            scissor_left,
            scissor_bottom,
            (scissor_right - scissor_left).max(0),
            (scissor_top - scissor_bottom).max(0),
        );
        gl.clear_color(BG[0], BG[1], BG[2], BG[3]);
        gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        gl.enable(glow::DEPTH_TEST);
        gl.depth_mask(true);
        gl.depth_func(glow::LEQUAL);
        gl.disable(glow::CULL_FACE);
        gl.enable(glow::BLEND);
        gl.blend_equation_separate(glow::FUNC_ADD, glow::FUNC_ADD);
        gl.blend_func_separate(
            glow::SRC_ALPHA,
            glow::ONE_MINUS_SRC_ALPHA,
            glow::ONE,
            glow::ONE_MINUS_SRC_ALPHA,
        );
        if st.program.is_none() && st.error.is_none() {
            if let Err(err) = gl_init(gl, st) {
                st.error = Some(err);
            }
        }
        if let (Some(program), Some(vao), Some(vbo)) = (st.program, st.vao, st.vbo) {
            gl.use_program(Some(program));
            gl.uniform_matrix_4_f32_slice(st.u_proj.as_ref(), false, &draw.proj);
            gl.uniform_matrix_4_f32_slice(st.u_view.as_ref(), false, &draw.view);
            gl.uniform_3_f32(st.u_fog_color.as_ref(), BG[0], BG[1], BG[2]);
            gl.uniform_1_f32(st.u_fog_near.as_ref(), FOG_NEAR);
            gl.uniform_1_f32(st.u_fog_far.as_ref(), FOG_FAR);
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            let bytes = std::slice::from_raw_parts(
                draw.verts.as_ptr() as *const u8,
                draw.verts.len() * 4,
            );
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STREAM_DRAW);
            gl.draw_arrays(glow::LINES, 0, draw.line_verts);
            gl.draw_arrays(glow::TRIANGLES, draw.line_verts, draw.total_verts - draw.line_verts);
        }
    }
    gl_restore(gl, saved);
}

impl Default for ViewportPanel {
    fn default() -> Self {
        ViewportPanel::new()
    }
}

impl ViewportPanel {
    pub fn new() -> ViewportPanel {
        ViewportPanel {
            mode: 1,
            yaw: 0.0,
            pitch: 0.4019,
            distance: 2.1731,
            target: [0.0, 0.15, 0.0],
            prev_mode: 1,
            follow_offset: None,
            prev_scrub: 0.0,
            seen_generation: 0,
            trail_signature: 0,
            trails: BTreeMap::new(),
            spin: 0.0,
            waypoint_hits: Vec::new(),
            gl: Arc::new(Mutex::new(GlState::default())),
        }
    }

    fn build_waypoints(
        &mut self,
        g: &mut SceneGeom,
        state: &AppState,
        view: &[f32; 16],
        proj: &[f32; 16],
        rect: Rect,
    ) {
        self.waypoint_hits.clear();
        let Some(id) = active_drone_id(state) else {
            return;
        };
        let Some(result) = state.sim_results.get(&id) else {
            return;
        };
        if state.active_drone_route.is_empty() {
            return;
        }
        let c = drone_color(state, &id);
        let dot = [c[0], c[1], c[2], 1.0];
        let ring = [c[0], c[1], c[2], 0.5];
        let len = result.positions.len();
        for (i, (command, pos_idx)) in state.active_drone_route.iter().enumerate() {
            let end = state
                .active_drone_route
                .get(i + 1)
                .map(|(_, p)| *p)
                .unwrap_or(len);
            let span = end.saturating_sub(*pos_idx);
            let mut spots = vec![*pos_idx];
            if span > 30 {
                spots.push(pos_idx + span / 2);
            }
            let selected = state.selection.command_path.first().copied() == Some(*command);
            for idx in spots {
                let Some(p) = result.positions.get(idx) else {
                    continue;
                };
                let gl = [p.x as f32, p.z as f32 + 0.05, p.y as f32];
                if selected {
                    emit_octa(g, gl, 0.06, [1.0, 1.0, 1.0, 1.0]);
                    emit_ring(g, gl, 0.075, 0.09, [1.0, 1.0, 1.0, 0.7]);
                } else {
                    emit_octa(g, gl, 0.045, dot);
                    emit_ring(g, gl, 0.06, 0.07, ring);
                }
                if let Some(screen) = project(gl, view, proj, rect) {
                    self.waypoint_hits.push((screen, *command));
                }
            }
        }
    }

    pub fn reset_camera(&mut self) {
        self.yaw = 0.0;
        self.pitch = 0.4019;
        self.distance = 2.1731;
        self.target = [0.0, 0.15, 0.0];
        self.follow_offset = None;
    }

    fn eye(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [
            self.target[0] + self.distance * sy * cp,
            self.target[1] + self.distance * sp,
            self.target[2] + self.distance * cy * cp,
        ]
    }

    fn handle_input(&mut self, ui: &Ui, response: &egui::Response) {
        if response.hovered() {
            let scroll = ui.input(|input| input.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let factor = 1.1f32.powf(-scroll / 40.0);
                self.distance = (self.distance * factor).clamp(MIN_DISTANCE, MAX_DISTANCE);
            }
        }
        let drag = response.drag_delta();
        let primary = response.dragged_by(egui::PointerButton::Primary);
        let shift = ui.input(|input| input.modifiers.shift);
        let pan = response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
            || (primary && shift);
        if pan && (drag.x != 0.0 || drag.y != 0.0) {
            self.pan_by(drag.x, drag.y);
        } else if primary && (drag.x != 0.0 || drag.y != 0.0) {
            self.yaw -= drag.x * 0.01;
            self.pitch = (self.pitch + drag.y * 0.01).clamp(MIN_PITCH, MAX_PITCH);
        }
    }

    fn pan_by(&mut self, dx: f32, dy: f32) {
        let eye = self.eye();
        let forward = vnorm(vsub(self.target, eye));
        let right = vnorm(vcross(forward, [0.0, 1.0, 0.0]));
        let up = vcross(right, forward);
        let scale = self.distance * 0.0016;
        for axis in 0..3 {
            self.target[axis] += (-right[axis] * dx + up[axis] * dy) * scale;
        }
    }

    fn sync_camera(&mut self, state: &AppState) {
        self.mode = state.camera_mode;
        if self.mode != self.prev_mode {
            self.prev_mode = self.mode;
            self.follow_offset = None;
            if self.mode == 1 {
                self.target = [0.0, 0.15, 0.0];
            }
        }
    }

    fn follow_drone(&mut self, state: &AppState, frame: &Option<FrameState>) {
        if self.mode != 2 {
            return;
        }
        let Some(frame) = frame else { return };
        let point = state
            .plan
            .drones
            .first()
            .and_then(|drone| frame.positions.get(&drone.id))
            .or_else(|| frame.positions.values().next());
        let Some(point) = point else { return };
        let pos = [point.x as f32, point.z as f32, point.y as f32];
        if self.follow_offset.is_none() {
            let offset = vsub(self.eye(), pos);
            let dist = (offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]).sqrt();
            if dist > 1e-4 {
                self.distance = dist.clamp(MIN_DISTANCE, MAX_DISTANCE);
                self.pitch = (offset[1] / dist)
                    .clamp(-1.0, 1.0)
                    .asin()
                    .clamp(MIN_PITCH, MAX_PITCH);
                self.yaw = offset[0].atan2(offset[2]);
            }
            self.follow_offset = Some(offset);
        }
        self.target = pos;
    }

    fn update_trails(&mut self, state: &AppState, frame: &Option<FrameState>) {
        let signature = state_signature(state);
        if signature != self.trail_signature || state.trail_generation != self.seen_generation {
            self.trail_signature = signature;
            self.seen_generation = state.trail_generation;
            self.follow_offset = None;
            self.trails.clear();
        }
        if !state.playback.playing {
            return;
        }
        let Some(frame) = frame else { return };
        for (id, point) in &frame.positions {
            let p = [point.x as f32, point.z as f32, point.y as f32];
            let trail = self.trails.entry(id.clone()).or_default();
            if trail.last().copied() != Some(p) {
                trail.push(p);
            }
            if trail.len() > 2000 {
                let excess = trail.len() - 2000;
                trail.drain(0..excess);
            }
        }
    }

    fn draw_labels(&self, ui: &Ui, rect: Rect, view: &[f32; 16], proj: &[f32; 16]) {
        let compass_color = Color32::from_rgb(0x8b, 0x94, 0x9e);
        let compass = [
            ("N", [0.0f32, 0.05, -3.0]),
            ("S", [0.0f32, 0.05, 3.0]),
            ("E", [3.0f32, 0.05, 0.0]),
            ("W", [-3.0f32, 0.05, 0.0]),
        ];
        for (text, pos) in compass {
            if let Some(p) = project(pos, view, proj, rect) {
                ui.painter().text(
                    p,
                    Align2::CENTER_CENTER,
                    text,
                    FontId::proportional(13.0),
                    compass_color,
                );
            }
        }
        let ft_color = Color32::from_rgb(0x48, 0x4f, 0x58);
        let mut ft = 5i32;
        while ft <= (GRID_SIZE / FT) as i32 {
            let m = ft as f32 * FT;
            if m > GRID_SIZE / 2.0 {
                break;
            }
            let label = format!("{ft}ft");
            for pos in [[m, 0.02, 0.15], [-m, 0.02, 0.15], [0.15, 0.02, m], [0.15, 0.02, -m]] {
                if let Some(p) = project(pos, view, proj, rect) {
                    ui.painter().text(
                        p,
                        Align2::CENTER_CENTER,
                        &label,
                        FontId::proportional(10.0),
                        ft_color,
                    );
                }
            }
            ft += 5;
        }
    }

    fn build_trails(&self, g: &mut SceneGeom) {
        let base = rgb(0x22c55e);
        let color = [base[0], base[1], base[2], 0.8];
        for trail in self.trails.values() {
            if let Some(first) = trail.first() {
                emit_octa(g, *first, 0.06, rgba(0x3fb950, 0.9));
            }
            for pair in trail.windows(2) {
                push_line(g, pair[0], pair[1], color);
            }
            if let Some(head) = trail.last() {
                emit_octa(g, *head, 0.04, [base[0], base[1], base[2], 0.4]);
            }
        }
    }

    fn pick_waypoint(&self, response: &egui::Response, state: &mut AppState) {
        let Some(pointer) = response.interact_pointer_pos() else {
            return;
        };
        let mut best: Option<(f32, usize)> = None;
        for (screen, command) in &self.waypoint_hits {
            let d = screen.distance(pointer);
            if d < 14.0 && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, *command));
            }
        }
        if let Some((_, command)) = best {
            state.selection.command_path = vec![command];
        }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        let rect = ui.available_rect_before_wrap();
        if rect.width() < 320.0 {
            ui.painter()
                .rect_filled(rect, 0.0, Color32::from_rgb(0x0d, 0x11, 0x17));
            ui.painter().text(
                rect.center(),
                Align2::CENTER_CENTER,
                "Close some panels (toolbar) to show the map",
                FontId::proportional(13.0),
                Color32::from_rgb(0x8b, 0x94, 0x9e),
            );
            return;
        }
        let response = ui.allocate_rect(rect, Sense::click_and_drag());
        self.handle_input(ui, &response);
        if state.tick(0.033) {
            self.spin += 50.0 * 0.033;
        }
        let frame = state.current_frame();
        self.update_trails(state, &frame);
        self.sync_camera(state);
        let follow_now = state.playback.playing
            || (state.playback_scrub - self.prev_scrub).abs() > f64::EPSILON;
        self.prev_scrub = state.playback_scrub;
        if follow_now {
            self.follow_drone(state, &frame);
        }
        if state.camera_reset_pending {
            state.camera_reset_pending = false;
            self.reset_camera();
        }
        let eye = self.eye();
        let view = mat4_look_at(eye, self.target, [0.0, 1.0, 0.0]);
        let aspect = rect.width().max(1.0) / rect.height().max(1.0);
        let proj = mat4_perspective(FOV_Y.to_radians(), aspect, 0.1, 100.0);
        let mut geom = SceneGeom::default();
        build_grid(&mut geom);
        if state.boundary_visible {
            build_boundary(&mut geom, state);
        }
        build_obstacles(&mut geom, state);
        build_route(&mut geom, state);
        self.build_waypoints(&mut geom, state, &view, &proj, rect);
        self.build_trails(&mut geom);
        build_drones(self, &mut geom, state, &frame);
        build_collision_markers(&mut geom, state);
        if response.clicked() && !state.playback.playing {
            self.pick_waypoint(&response, state);
        }
        let line_verts = (geom.lines.len() / 7) as i32;
        let mut verts = geom.lines;
        verts.extend_from_slice(&geom.tris);
        let total_verts = (verts.len() / 7) as i32;
        let draw = Arc::new(FrameDraw {
            proj,
            view,
            verts,
            line_verts,
            total_verts,
        });
        let gl_state = Arc::clone(&self.gl);
        ui.painter()
            .rect_filled(rect, 0.0, Color32::from_rgb(0x0d, 0x11, 0x17));
        ui.painter().add(egui::PaintCallback {
            rect,
            callback: Arc::new(egui_glow::CallbackFn::new(move |info, painter| {
                let gl = Arc::clone(painter.gl());
                if let Ok(mut st) = gl_state.lock() {
                    paint_gl(&gl, &mut st, &info, &draw);
                }
            })),
        });
        if let Ok(st) = self.gl.lock() {
            if let Some(err) = &st.error {
                ui.painter().text(
                    rect.left_top() + egui::vec2(8.0, 8.0),
                    Align2::LEFT_TOP,
                    err,
                    FontId::proportional(12.0),
                    Color32::from_rgb(0xf8, 0x51, 0x49),
                );
            }
        }
        self.draw_labels(ui, rect, &view, &proj);
    }
}

#[derive(Default)]
struct SceneGeom {
    lines: Vec<f32>,
    tris: Vec<f32>,
}

fn sig_bytes(seed: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *seed ^= *byte as u64;
        *seed = seed.wrapping_mul(1099511628211);
    }
}

fn state_signature(state: &AppState) -> u64 {
    let mut hash = 14695981039346656037u64;
    sig_bytes(&mut hash, &(state.plan.drones.len() as u64).to_le_bytes());
    sig_bytes(&mut hash, state.generated_code.as_bytes());
    let total_points: usize = state.sim_results.values().map(|r| r.positions.len()).sum();
    sig_bytes(&mut hash, &(total_points as u64).to_le_bytes());
    sig_bytes(&mut hash, &state.max_duration().to_bits().to_le_bytes());
    let obstacles = state.obstacle_store();
    sig_bytes(&mut hash, &(obstacles.len() as u64).to_le_bytes());
    for obstacle in obstacles {
        sig_bytes(&mut hash, obstacle.obstacle_type.as_bytes());
        sig_bytes(&mut hash, obstacle.name.as_bytes());
        for value in obstacle
            .position
            .iter()
            .chain(obstacle.rotation.iter())
            .chain(obstacle.scale.iter())
        {
            sig_bytes(&mut hash, &value.to_bits().to_le_bytes());
        }
    }
    let boundary = &state.obstacles.boundary;
    for value in [
        boundary.min_x,
        boundary.max_x,
        boundary.min_z,
        boundary.max_z,
        boundary.max_y,
    ] {
        sig_bytes(&mut hash, &value.to_bits().to_le_bytes());
    }
    hash
}

fn push_line(g: &mut SceneGeom, a: [f32; 3], b: [f32; 3], color: [f32; 4]) {
    for v in [a, b] {
        g.lines
            .extend_from_slice(&[v[0], v[1], v[2], color[0], color[1], color[2], color[3]]);
    }
}

fn push_tri(g: &mut SceneGeom, a: [f32; 3], b: [f32; 3], c: [f32; 3], color: [f32; 4]) {
    for v in [a, b, c] {
        g.tris
            .extend_from_slice(&[v[0], v[1], v[2], color[0], color[1], color[2], color[3]]);
    }
}

fn rgb(hex: u32) -> [f32; 3] {
    [
        ((hex >> 16) & 255) as f32 / 255.0,
        ((hex >> 8) & 255) as f32 / 255.0,
        (hex & 255) as f32 / 255.0,
    ]
}

fn rgba(hex: u32, a: f32) -> [f32; 4] {
    let c = rgb(hex);
    [c[0], c[1], c[2], a]
}

fn build_grid(g: &mut SceneGeom) {
    let half = GRID_SIZE / 2.0;
    let ground = rgba(0x0d1117, 1.0);
    push_tri(
        g,
        [-half, -0.01, -half],
        [half, -0.01, -half],
        [half, -0.01, half],
        ground,
    );
    push_tri(
        g,
        [-half, -0.01, -half],
        [half, -0.01, half],
        [-half, -0.01, half],
        ground,
    );
    let minor_div = (GRID_SIZE / FT).round() as i32;
    for i in 0..=minor_div {
        let t = -half + GRID_SIZE * i as f32 / minor_div as f32;
        let color = if i == minor_div / 2 {
            rgba(0x30363d, 0.6)
        } else {
            rgba(0x21262d, 0.6)
        };
        push_line(g, [t, 0.0, -half], [t, 0.0, half], color);
        push_line(g, [-half, 0.0, t], [half, 0.0, t], color);
    }
    let major_div = (GRID_SIZE / (FT * 5.0)).round() as i32;
    for i in 0..=major_div {
        let t = -half + GRID_SIZE * i as f32 / major_div as f32;
        let color = if i == major_div / 2 {
            rgba(0x484f58, 0.8)
        } else {
            rgba(0x30363d, 0.8)
        };
        push_line(g, [t, 0.001, -half], [t, 0.001, half], color);
        push_line(g, [-half, 0.001, t], [half, 0.001, t], color);
    }
}

fn active_drone_id(state: &AppState) -> Option<String> {
    state
        .plan
        .active_drone_id
        .clone()
        .or_else(|| state.plan.drones.first().map(|drone| drone.id.clone()))
}

fn hue2rgb(p: f32, q: f32, t: f32) -> f32 {
    let mut t = t;
    if t < 0.0 {
        t += 1.0;
    }
    if t > 1.0 {
        t -= 1.0;
    }
    if t < 1.0 / 6.0 {
        p + (q - p) * 6.0 * t
    } else if t < 1.0 / 2.0 {
        q
    } else if t < 2.0 / 3.0 {
        p + (q - p) * 6.0 * (2.0 / 3.0 - t)
    } else {
        p
    }
}

fn hsl_to_rgb(h: f32, s: f32, l: f32) -> [f32; 3] {
    let h = h.rem_euclid(1.0);
    let s = s.clamp(0.0, 1.0);
    let l = l.clamp(0.0, 1.0);
    if s == 0.0 {
        return [l, l, l];
    }
    let p = if l <= 0.5 { l * (1.0 + s) } else { l + s - l * s };
    let q = 2.0 * l - p;
    [
        hue2rgb(q, p, h + 1.0 / 3.0),
        hue2rgb(q, p, h),
        hue2rgb(q, p, h - 1.0 / 3.0),
    ]
}

fn decimal_literal(text: &str) -> Option<f32> {
    if text.is_empty() {
        return None;
    }
    let mut dots = 0;
    let mut digits = 0;
    for byte in text.bytes() {
        if byte.is_ascii_digit() {
            digits += 1;
        } else if byte == b'.' {
            dots += 1;
            if dots > 1 {
                return None;
            }
        } else {
            return None;
        }
    }
    if digits == 0 || text.ends_with('.') {
        return None;
    }
    text.parse::<f32>().ok()
}

fn digit_uint(text: &str) -> Option<u32> {
    if text.is_empty() || !text.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    text.parse::<u32>().ok()
}

fn color_parts(components: &str) -> Option<Vec<&str>> {
    let parts: Vec<&str> = components.split(',').map(|part| part.trim()).collect();
    if parts.len() < 3 || parts.len() > 4 {
        return None;
    }
    if parts.iter().any(|part| part.is_empty()) {
        return None;
    }
    if parts.len() == 4 && decimal_literal(parts[3]).is_none() {
        return None;
    }
    Some(parts)
}

fn rgb_channel(part: &str, percent: bool) -> Option<f32> {
    if percent {
        let value = digit_uint(part.strip_suffix('%')?)?;
        Some(value.min(100) as f32 / 100.0)
    } else {
        let value = digit_uint(part)?;
        Some(value.min(255) as f32 / 255.0)
    }
}

fn percent_channel(part: &str) -> Option<f32> {
    decimal_literal(part.strip_suffix('%')?)
}

fn parse_color_function(name: &str, components: &str) -> Option<[f32; 3]> {
    match name {
        "rgb" | "rgba" => {
            let parts = color_parts(components)?;
            if let (Some(r), Some(g), Some(b)) = (
                rgb_channel(parts[0], false),
                rgb_channel(parts[1], false),
                rgb_channel(parts[2], false),
            ) {
                return Some([r, g, b]);
            }
            let parts = color_parts(components)?;
            if let (Some(r), Some(g), Some(b)) = (
                rgb_channel(parts[0], true),
                rgb_channel(parts[1], true),
                rgb_channel(parts[2], true),
            ) {
                return Some([r, g, b]);
            }
            None
        }
        "hsl" | "hsla" => {
            let parts = color_parts(components)?;
            let h = decimal_literal(parts[0])?;
            let s = percent_channel(parts[1])?;
            let l = percent_channel(parts[2])?;
            Some(hsl_to_rgb(h / 360.0, s / 100.0, l / 100.0))
        }
        _ => None,
    }
}

fn color_function(text: &str) -> Option<(&str, &str)> {
    let open = text.find('(')?;
    let name = &text[..open];
    if name.is_empty() {
        return None;
    }
    if !name
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_')
    {
        return None;
    }
    let rest = &text[open + 1..];
    let close = rest.find(')')?;
    Some((name, &rest[..close]))
}

fn css_color_name(name: &str) -> Option<[f32; 3]> {
    let value = match name {
        "aliceblue" => 0xf0f8ff,
        "antiquewhite" => 0xfaebd7,
        "aqua" => 0x00ffff,
        "aquamarine" => 0x7fffd4,
        "azure" => 0xf0ffff,
        "beige" => 0xf5f5dc,
        "bisque" => 0xffe4c4,
        "black" => 0x000000,
        "blanchedalmond" => 0xffebcd,
        "blue" => 0x0000ff,
        "blueviolet" => 0x8a2be2,
        "brown" => 0xa52a2a,
        "burlywood" => 0xdeb887,
        "cadetblue" => 0x5f9ea0,
        "chartreuse" => 0x7fff00,
        "chocolate" => 0xd2691e,
        "coral" => 0xff7f50,
        "cornflowerblue" => 0x6495ed,
        "cornsilk" => 0xfff8dc,
        "crimson" => 0xdc143c,
        "cyan" => 0x00ffff,
        "darkblue" => 0x00008b,
        "darkcyan" => 0x008b8b,
        "darkgoldenrod" => 0xb8860b,
        "darkgray" => 0xa9a9a9,
        "darkgreen" => 0x006400,
        "darkgrey" => 0xa9a9a9,
        "darkkhaki" => 0xbdb76b,
        "darkmagenta" => 0x8b008b,
        "darkolivegreen" => 0x556b2f,
        "darkorange" => 0xff8c00,
        "darkorchid" => 0x9932cc,
        "darkred" => 0x8b0000,
        "darksalmon" => 0xe9967a,
        "darkseagreen" => 0x8fbc8f,
        "darkslateblue" => 0x483d8b,
        "darkslategray" => 0x2f4f4f,
        "darkslategrey" => 0x2f4f4f,
        "darkturquoise" => 0x00ced1,
        "darkviolet" => 0x9400d3,
        "deeppink" => 0xff1493,
        "deepskyblue" => 0x00bfff,
        "dimgray" => 0x696969,
        "dimgrey" => 0x696969,
        "dodgerblue" => 0x1e90ff,
        "firebrick" => 0xb22222,
        "floralwhite" => 0xfffaf0,
        "forestgreen" => 0x228b22,
        "fuchsia" => 0xff00ff,
        "gainsboro" => 0xdcdcdc,
        "ghostwhite" => 0xf8f8ff,
        "gold" => 0xffd700,
        "goldenrod" => 0xdaa520,
        "gray" => 0x808080,
        "green" => 0x008000,
        "greenyellow" => 0xadff2f,
        "grey" => 0x808080,
        "honeydew" => 0xf0fff0,
        "hotpink" => 0xff69b4,
        "indianred" => 0xcd5c5c,
        "indigo" => 0x4b0082,
        "ivory" => 0xfffff0,
        "khaki" => 0xf0e68c,
        "lavender" => 0xe6e6fa,
        "lavenderblush" => 0xfff0f5,
        "lawngreen" => 0x7cfc00,
        "lemonchiffon" => 0xfffacd,
        "lightblue" => 0xadd8e6,
        "lightcoral" => 0xf08080,
        "lightcyan" => 0xe0ffff,
        "lightgoldenrodyellow" => 0xfafad2,
        "lightgray" => 0xd3d3d3,
        "lightgreen" => 0x90ee90,
        "lightgrey" => 0xd3d3d3,
        "lightpink" => 0xffb6c1,
        "lightsalmon" => 0xffa07a,
        "lightseagreen" => 0x20b2aa,
        "lightskyblue" => 0x87cefa,
        "lightslategray" => 0x778899,
        "lightslategrey" => 0x778899,
        "lightsteelblue" => 0xb0c4de,
        "lightyellow" => 0xffffe0,
        "lime" => 0x00ff00,
        "limegreen" => 0x32cd32,
        "linen" => 0xfaf0e6,
        "magenta" => 0xff00ff,
        "maroon" => 0x800000,
        "mediumaquamarine" => 0x66cdaa,
        "mediumblue" => 0x0000cd,
        "mediumorchid" => 0xba55d3,
        "mediumpurple" => 0x9370db,
        "mediumseagreen" => 0x3cb371,
        "mediumslateblue" => 0x7b68ee,
        "mediumspringgreen" => 0x00fa9a,
        "mediumturquoise" => 0x48d1cc,
        "mediumvioletred" => 0xc71585,
        "midnightblue" => 0x191970,
        "mintcream" => 0xf5fffa,
        "mistyrose" => 0xffe4e1,
        "moccasin" => 0xffe4b5,
        "navajowhite" => 0xffdead,
        "navy" => 0x000080,
        "oldlace" => 0xfdf5e6,
        "olive" => 0x808000,
        "olivedrab" => 0x6b8e23,
        "orange" => 0xffa500,
        "orangered" => 0xff4500,
        "orchid" => 0xda70d6,
        "palegoldenrod" => 0xeee8aa,
        "palegreen" => 0x98fb98,
        "paleturquoise" => 0xafeeee,
        "palevioletred" => 0xdb7093,
        "papayawhip" => 0xffefd5,
        "peachpuff" => 0xffdab9,
        "peru" => 0xcd853f,
        "pink" => 0xffc0cb,
        "plum" => 0xdda0dd,
        "powderblue" => 0xb0e0e6,
        "purple" => 0x800080,
        "rebeccapurple" => 0x663399,
        "red" => 0xff0000,
        "rosybrown" => 0xbc8f8f,
        "royalblue" => 0x4169e1,
        "saddlebrown" => 0x8b4513,
        "salmon" => 0xfa8072,
        "sandybrown" => 0xf4a460,
        "seagreen" => 0x2e8b57,
        "seashell" => 0xfff5ee,
        "sienna" => 0xa0522d,
        "silver" => 0xc0c0c0,
        "skyblue" => 0x87ceeb,
        "slateblue" => 0x6a5acd,
        "slategray" => 0x708090,
        "slategrey" => 0x708090,
        "snow" => 0xfffafa,
        "springgreen" => 0x00ff7f,
        "steelblue" => 0x4682b4,
        "tan" => 0xd2b48c,
        "teal" => 0x008080,
        "thistle" => 0xd8bfd8,
        "tomato" => 0xff6347,
        "turquoise" => 0x40e0d0,
        "violet" => 0xee82ee,
        "wheat" => 0xf5deb3,
        "white" => 0xffffff,
        "whitesmoke" => 0xf5f5f5,
        "yellow" => 0xffff00,
        "yellowgreen" => 0x9acd32,
        _ => return None,
    };
    Some(rgb(value))
}

fn parse_color(text: &str) -> [f32; 3] {
    if let Some((name, components)) = color_function(text) {
        return parse_color_function(name, components).unwrap_or([1.0, 1.0, 1.0]);
    }
    if let Some(body) = text.strip_prefix('#') {
        if !body.is_empty() && body.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            if body.len() == 3 {
                let r = u8::from_str_radix(&body[0..1], 16).unwrap() as f32 / 15.0;
                let g = u8::from_str_radix(&body[1..2], 16).unwrap() as f32 / 15.0;
                let b = u8::from_str_radix(&body[2..3], 16).unwrap() as f32 / 15.0;
                return [r.min(1.0), g.min(1.0), b.min(1.0)];
            }
            if body.len() == 6 {
                let value = u32::from_str_radix(body, 16).unwrap();
                return rgb(value);
            }
        }
        return [1.0, 1.0, 1.0];
    }
    if !text.is_empty() {
        if let Some(value) = css_color_name(&text.to_ascii_lowercase()) {
            return value;
        }
    }
    [1.0, 1.0, 1.0]
}

fn rot_euler_xyz(rot: [f32; 3]) -> [[f32; 3]; 3] {
    let (sx, cx) = rot[0].sin_cos();
    let (sy, cy) = rot[1].sin_cos();
    let (sz, cz) = rot[2].sin_cos();
    [
        [cy * cz, -cy * sz, sy],
        [cx * sz + sx * sy * cz, cx * cz - sx * sy * sz, -sx * cy],
        [sx * sz - cx * sy * cz, sx * cz + cx * sy * sz, cx * cy],
    ]
}

fn mat3_vec(m: &[[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

#[derive(Clone, Copy)]
struct Xform {
    rot: [[f32; 3]; 3],
    scale: [f32; 3],
    pos: [f32; 3],
}

impl Xform {
    fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        let s = [
            v[0] * self.scale[0],
            v[1] * self.scale[1],
            v[2] * self.scale[2],
        ];
        let r = mat3_vec(&self.rot, s);
        [r[0] + self.pos[0], r[1] + self.pos[1], r[2] + self.pos[2]]
    }
}

const BOX_TRIS: [[usize; 3]; 12] = [
    [0, 2, 1],
    [1, 2, 3],
    [4, 5, 7],
    [4, 7, 6],
    [0, 4, 2],
    [2, 4, 6],
    [1, 3, 5],
    [3, 7, 5],
    [0, 1, 4],
    [1, 5, 4],
    [2, 6, 3],
    [3, 6, 7],
];

const BOX_EDGES: [[usize; 2]; 12] = [
    [0, 1],
    [1, 3],
    [3, 2],
    [2, 0],
    [4, 5],
    [5, 7],
    [7, 6],
    [6, 4],
    [0, 4],
    [1, 5],
    [3, 7],
    [2, 6],
];

fn emit_box(g: &mut SceneGeom, xf: &Xform, dims: [f32; 3], color: [f32; 4], edges: bool) {
    let h = [dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0];
    let mut c = [[0.0f32; 3]; 8];
    for (i, corner) in c.iter_mut().enumerate() {
        let v = [
            if i & 1 == 0 { -h[0] } else { h[0] },
            if i & 2 == 0 { -h[1] } else { h[1] },
            if i & 4 == 0 { -h[2] } else { h[2] },
        ];
        *corner = xf.apply(v);
    }
    for t in BOX_TRIS {
        push_tri(g, c[t[0]], c[t[1]], c[t[2]], color);
    }
    if edges {
        let edge = [color[0] * 0.6, color[1] * 0.6, color[2] * 0.6, color[3]];
        for e in BOX_EDGES {
            push_line(g, c[e[0]], c[e[1]], edge);
        }
    }
}

fn build_boundary(g: &mut SceneGeom, state: &AppState) {
    let b = &state.obstacles.boundary;
    let (x0, x1) = (b.min_x as f32, b.max_x as f32);
    let (z0, z1) = (b.min_z as f32, b.max_z as f32);
    let y1 = b.max_y as f32;
    let color = rgba(0x00d4ff, 0.5);
    let c = [
        [x0, 0.0, z0],
        [x1, 0.0, z0],
        [x1, 0.0, z1],
        [x0, 0.0, z1],
        [x0, y1, z0],
        [x1, y1, z0],
        [x1, y1, z1],
        [x0, y1, z1],
    ];
    for (a, b) in [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ] {
        push_line(g, c[a], c[b], color);
    }
}

fn build_route(g: &mut SceneGeom, state: &AppState) {
    let color = rgba(0xf87171, 0.85);
    for result in state.sim_results.values() {
        for pair in result.positions.windows(2) {
            let a = &pair[0];
            let b = &pair[1];
            push_line(
                g,
                [a.x as f32, a.z as f32, a.y as f32],
                [b.x as f32, b.z as f32, b.y as f32],
                color,
            );
        }
    }
}

fn build_collision_markers(g: &mut SceneGeom, state: &AppState) {
    let color = rgba(0xe53e3e, 0.9);
    for pt in state.collision_points() {
        let gl = [pt[0] as f32, pt[2] as f32, pt[1] as f32];
        emit_octa(g, gl, 0.12, color);
    }
}

fn mat3_mul(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
        }
    }
    out
}

fn rot_yxz(yaw: f32, pitch: f32, roll: f32) -> [[f32; 3]; 3] {
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sr, cr) = roll.sin_cos();
    mat3_mul(
        &mat3_mul(
            &[[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]],
            &[[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]],
        ),
        &[[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]],
    )
}

fn led_rgb(led: &LedValue) -> Option<[f32; 3]> {
    match led {
        LedValue::Rgb { r, g, b, .. } => Some([
            *r as f32 / 255.0,
            *g as f32 / 255.0,
            *b as f32 / 255.0,
        ]),
        LedValue::Name(name) => match name.as_str() {
            "red" => Some(rgb(0xff0000)),
            "green" => Some(rgb(0x00ff00)),
            "blue" => Some(rgb(0x0066ff)),
            "yellow" => Some(rgb(0xffff00)),
            "cyan" => Some(rgb(0x00ffff)),
            "magenta" => Some(rgb(0xff00ff)),
            "white" => Some(rgb(0xffffff)),
            "purple" => Some(rgb(0xaa00ff)),
            "orange" => Some(rgb(0xff8800)),
            "pink" => Some(rgb(0xff44aa)),
            _ => None,
        },
    }
}

fn build_drones(panel: &ViewportPanel, g: &mut SceneGeom, state: &AppState, frame: &Option<FrameState>) {
    let Some(frame) = frame else { return };
    let s = 0.1f32 / 0.28;
    let ident = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for (id, point) in &frame.positions {
        let color = drone_color(state, id);
        let yaw = (-(point.heading as f32)).to_radians() - std::f32::consts::FRAC_PI_2;
        let rot = rot_yxz(
            yaw,
            (-(point.pitch as f32)).to_radians(),
            (-(point.roll as f32)).to_radians(),
        );
        let base = Xform {
            rot,
            scale: [s, s, s],
            pos: [point.x as f32, point.z as f32, point.y as f32],
        };
        let at = |offset: [f32; 3], extra: &[[f32; 3]; 3]| -> Xform {
            let moved = mat3_vec(&base.rot, [offset[0] * s, offset[1] * s, offset[2] * s]);
            Xform {
                rot: mat3_mul(&base.rot, extra),
                scale: [s, s, s],
                pos: [
                    base.pos[0] + moved[0],
                    base.pos[1] + moved[1],
                    base.pos[2] + moved[2],
                ],
            }
        };
        emit_box(g, &base, [0.28, 0.035, 0.28], rgba(0x2d5aa0, 1.0), false);
        let led_c = led_rgb(&point.led).unwrap_or([0.0, 0.0, 0.0]);
        let led_color = [led_c[0], led_c[1], led_c[2], 1.0];
        for (lx, lz) in [
            (-0.06f32, -0.145f32),
            (0.06, -0.145),
            (-0.06, 0.145),
            (0.06, 0.145),
        ] {
            emit_box(
                g,
                &at([lx, 0.01, lz], &ident),
                [0.03, 0.008, 0.015],
                led_color,
                false,
            );
        }
        for (lx, lz) in [(-0.145f32, 0.0f32), (0.145, 0.0)] {
            emit_box(
                g,
                &at([lx, 0.01, lz], &ident),
                [0.015, 0.008, 0.03],
                led_color,
                false,
            );
        }
        for (lx, lz) in [
            (-0.08f32, -0.08f32),
            (0.08, -0.08),
            (-0.08, 0.08),
            (0.08, 0.08),
        ] {
            emit_box(
                g,
                &at([lx, -0.038, lz], &ident),
                [0.02, 0.006, 0.02],
                led_color,
                false,
            );
        }
        emit_box(
            g,
            &at([0.0, 0.015, -0.17], &ident),
            [0.06, 0.02, 0.04],
            [color[0], color[1], color[2], 1.0],
            false,
        );
        for (sx, sz) in [(1.0f32, 1.0f32), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
            emit_box(
                g,
                &at([0.18 * sx, 0.0, 0.18 * sz], &ident),
                [0.2, 0.022, 0.032],
                rgba(0x555555, 1.0),
                false,
            );
            for b in 0..2 {
                let blade = rot_yxz(panel.spin + b as f32 * std::f32::consts::FRAC_PI_2, 0.0, 0.0);
                emit_box(
                    g,
                    &at([0.28 * sx, 0.058, 0.28 * sz], &blade),
                    [0.22, 0.002, 0.018],
                    rgba(0x888888, 0.45),
                    false,
                );
            }
        }
    }
}

fn drone_color(state: &AppState, id: &str) -> [f32; 3] {
    state
        .plan
        .drones
        .iter()
        .find(|drone| drone.id == id)
        .map(|drone| parse_color(&drone.color))
        .unwrap_or(rgb(0x00d4ff))
}

fn emit_octa(g: &mut SceneGeom, center: [f32; 3], r: f32, color: [f32; 4]) {
    let v = [
        [center[0] + r, center[1], center[2]],
        [center[0] - r, center[1], center[2]],
        [center[0], center[1] + r, center[2]],
        [center[0], center[1] - r, center[2]],
        [center[0], center[1], center[2] + r],
        [center[0], center[1], center[2] - r],
    ];
    const TRIS: [[usize; 3]; 8] = [
        [2, 4, 0],
        [2, 0, 5],
        [2, 5, 1],
        [2, 1, 4],
        [3, 0, 4],
        [3, 5, 0],
        [3, 1, 5],
        [3, 4, 1],
    ];
    for t in TRIS {
        push_tri(g, v[t[0]], v[t[1]], v[t[2]], color);
    }
}

fn emit_ring(g: &mut SceneGeom, center: [f32; 3], r0: f32, r1: f32, color: [f32; 4]) {
    const SEG: usize = 16;
    for i in 0..SEG {
        let a0 = i as f32 / SEG as f32 * std::f32::consts::TAU;
        let a1 = (i + 1) as f32 / SEG as f32 * std::f32::consts::TAU;
        let p00 = [center[0] + r0 * a0.cos(), center[1], center[2] + r0 * a0.sin()];
        let p01 = [center[0] + r0 * a1.cos(), center[1], center[2] + r0 * a1.sin()];
        let p10 = [center[0] + r1 * a0.cos(), center[1], center[2] + r1 * a0.sin()];
        let p11 = [center[0] + r1 * a1.cos(), center[1], center[2] + r1 * a1.sin()];
        push_tri(g, p00, p10, p11, color);
        push_tri(g, p00, p11, p01, color);
    }
}

fn obstacle_type_color(kind: &str) -> [f32; 3] {
    match kind {
        "tower" => rgb(0x2d3748),
        "hoop" => rgb(0xe53e3e),
        "cone" => rgb(0xfbb6ce),
        "sphere" => rgb(0x22d3ee),
        _ => rgb(0x4a5568),
    }
}

fn finite3(v: &[f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn build_obstacles(g: &mut SceneGeom, state: &AppState) {
    for obstacle in &state.obstacles.obstacles {
        let Some(dims) = planner_core::obstacles::type_dims(&obstacle.obstacle_type) else {
            continue;
        };
        if !finite3(&obstacle.position) || !finite3(&obstacle.rotation) || !finite3(&obstacle.scale)
        {
            continue;
        }
        let base = obstacle
            .color
            .as_deref()
            .map(parse_color)
            .unwrap_or_else(|| obstacle_type_color(&obstacle.obstacle_type));
        let color = [base[0], base[1], base[2], 1.0];
        let xf = Xform {
            rot: rot_euler_xyz([
                obstacle.rotation[0] as f32,
                obstacle.rotation[1] as f32,
                obstacle.rotation[2] as f32,
            ]),
            scale: [
                obstacle.scale[0] as f32,
                obstacle.scale[1] as f32,
                obstacle.scale[2] as f32,
            ],
            pos: [
                obstacle.position[0] as f32,
                obstacle.position[1] as f32,
                obstacle.position[2] as f32,
            ],
        };
        match obstacle.obstacle_type.as_str() {
            "wall" | "tower" | "square" => emit_box(
                g,
                &xf,
                [
                    dims.width as f32,
                    dims.height as f32,
                    dims.depth as f32,
                ],
                color,
                true,
            ),
            "hoop" => emit_torus(
                g,
                &xf,
                dims.outer_radius as f32,
                dims.inner_radius as f32,
                color,
            ),
            "cone" => emit_cone(g, &xf, dims.radius as f32, dims.height as f32, color),
            "sphere" => emit_sphere(g, &xf, dims.radius as f32, color),
            _ => {}
        }
    }
}

fn emit_torus(g: &mut SceneGeom, xf: &Xform, ring_r: f32, tube_r: f32, color: [f32; 4]) {
    const RING: usize = 16;
    const TUBE: usize = 8;
    let pt = |i: usize, j: usize| -> [f32; 3] {
        let u = i as f32 / RING as f32 * std::f32::consts::TAU;
        let v = j as f32 / TUBE as f32 * std::f32::consts::TAU;
        [
            (ring_r + tube_r * v.cos()) * u.cos(),
            (ring_r + tube_r * v.cos()) * u.sin(),
            tube_r * v.sin(),
        ]
    };
    for i in 0..RING {
        for j in 0..TUBE {
            let a = xf.apply(pt(i, j));
            let b = xf.apply(pt(i + 1, j));
            let c = xf.apply(pt(i + 1, j + 1));
            let d = xf.apply(pt(i, j + 1));
            push_tri(g, a, b, c, color);
            push_tri(g, a, c, d, color);
        }
    }
}

fn emit_cone(g: &mut SceneGeom, xf: &Xform, radius: f32, height: f32, color: [f32; 4]) {
    const SEG: usize = 16;
    let apex = xf.apply([0.0, height / 2.0, 0.0]);
    let center = xf.apply([0.0, -height / 2.0, 0.0]);
    for i in 0..SEG {
        let a0 = i as f32 / SEG as f32 * std::f32::consts::TAU;
        let a1 = (i + 1) as f32 / SEG as f32 * std::f32::consts::TAU;
        let p0 = xf.apply([radius * a0.cos(), -height / 2.0, radius * a0.sin()]);
        let p1 = xf.apply([radius * a1.cos(), -height / 2.0, radius * a1.sin()]);
        push_tri(g, apex, p0, p1, color);
        push_tri(g, center, p1, p0, color);
    }
}

fn emit_sphere(g: &mut SceneGeom, xf: &Xform, radius: f32, color: [f32; 4]) {
    const LON: usize = 16;
    const LAT: usize = 8;
    let pt = |i: usize, j: usize| -> [f32; 3] {
        let u = i as f32 / LON as f32 * std::f32::consts::TAU;
        let v = j as f32 / LAT as f32 * std::f32::consts::PI - std::f32::consts::FRAC_PI_2;
        [
            radius * v.cos() * u.cos(),
            radius * v.sin(),
            radius * v.cos() * u.sin(),
        ]
    };
    for i in 0..LON {
        for j in 0..LAT {
            let a = xf.apply(pt(i, j));
            let b = xf.apply(pt(i + 1, j));
            let c = xf.apply(pt(i + 1, j + 1));
            let d = xf.apply(pt(i, j + 1));
            push_tri(g, a, b, c, color);
            push_tri(g, a, c, d, color);
        }
    }
}

fn vsub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn vdot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn mat4_perspective(fov_y: f32, aspect: f32, near: f32, far: f32) -> [f32; 16] {
    let f = 1.0 / (fov_y * 0.5).tan();
    let mut m = [0.0f32; 16];
    m[0] = f / aspect;
    m[5] = f;
    m[10] = (far + near) / (near - far);
    m[11] = -1.0;
    m[14] = 2.0 * far * near / (near - far);
    m
}

fn mat4_look_at(eye: [f32; 3], target: [f32; 3], up: [f32; 3]) -> [f32; 16] {
    let z = vnorm(vsub(eye, target));
    let x = vnorm(vcross(up, z));
    let y = vcross(z, x);
    [
        x[0],
        y[0],
        z[0],
        0.0,
        x[1],
        y[1],
        z[1],
        0.0,
        x[2],
        y[2],
        z[2],
        0.0,
        -vdot(x, eye),
        -vdot(y, eye),
        -vdot(z, eye),
        1.0,
    ]
}

fn mat4_mul_vec4(m: &[f32; 16], v: [f32; 4]) -> [f32; 4] {
    let mut out = [0.0f32; 4];
    for row in 0..4 {
        out[row] = m[row] * v[0] + m[4 + row] * v[1] + m[8 + row] * v[2] + m[12 + row] * v[3];
    }
    out
}

fn project(pos: [f32; 3], view: &[f32; 16], proj: &[f32; 16], rect: Rect) -> Option<Pos2> {
    let v = mat4_mul_vec4(view, [pos[0], pos[1], pos[2], 1.0]);
    let c = mat4_mul_vec4(proj, v);
    if c[3] <= 0.001 {
        return None;
    }
    let ndc_x = c[0] / c[3];
    let ndc_y = c[1] / c[3];
    Some(Pos2::new(
        rect.left() + (ndc_x + 1.0) * 0.5 * rect.width(),
        rect.top() + (1.0 - ndc_y) * 0.5 * rect.height(),
    ))
}

fn vcross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn vnorm(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len > 1e-6 {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn color_u8(color: [f32; 3]) -> [u8; 3] {
        [
            (color[0] * 255.0).round() as u8,
            (color[1] * 255.0).round() as u8,
            (color[2] * 255.0).round() as u8,
        ]
    }

    #[test]
    fn parse_color_matches_style_rules() {
        assert_eq!(color_u8(parse_color("#4a9c68")), [0x4a, 0x9c, 0x68]);
        assert_eq!(color_u8(parse_color("#abc")), [0xaa, 0xbb, 0xcc]);
        assert_eq!(color_u8(parse_color("#A1B2C3")), [0xa1, 0xb2, 0xc3]);
        assert_eq!(color_u8(parse_color("#aabbccdd")), [255, 255, 255]);
        assert_eq!(color_u8(parse_color("4a9c68")), [255, 255, 255]);
        assert_eq!(color_u8(parse_color("0x4a9c68")), [255, 255, 255]);
        assert_eq!(color_u8(parse_color("red")), [255, 0, 0]);
        assert_eq!(color_u8(parse_color("rebeccapurple")), [0x66, 0x33, 0x99]);
        assert_eq!(color_u8(parse_color("aliceblue")), [0xf0, 0xf8, 0xff]);
        assert_eq!(color_u8(parse_color("rgb(74,156,104)")), [74, 156, 104]);
        assert_eq!(color_u8(parse_color("rgba(74,156,104,0.5)")), [74, 156, 104]);
        assert_eq!(color_u8(parse_color("rgb(100%,0%,0%)")), [255, 0, 0]);
        assert_eq!(color_u8(parse_color("hsl(0,100%,50%)")), [255, 0, 0]);
        assert_eq!(color_u8(parse_color("hsl(240,100%,50%)")), [0, 0, 255]);
        assert_eq!(color_u8(parse_color("transparent")), [255, 255, 255]);
        assert_eq!(color_u8(parse_color("not-a-color")), [255, 255, 255]);
        assert_eq!(color_u8(parse_color("")), [255, 255, 255]);
    }
}
