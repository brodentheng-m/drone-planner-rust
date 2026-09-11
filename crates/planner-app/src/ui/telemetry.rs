use crate::state::AppState;
use egui::{Color32, Pos2, RichText, Sense, Stroke, Ui, Vec2};

pub struct TelemetryPanel {
    chart_expanded: bool,
}

impl Default for TelemetryPanel {
    fn default() -> Self {
        TelemetryPanel {
            chart_expanded: false,
        }
    }
}

struct Metrics {
    alt: f64,
    spd: f64,
    bat: f64,
    hdg: f64,
    pitch: f64,
    roll: f64,
    x: f64,
    y: f64,
    z: f64,
}

impl Default for Metrics {
    fn default() -> Self {
        Metrics {
            alt: 0.0,
            spd: 0.0,
            bat: 100.0,
            hdg: 0.0,
            pitch: 0.0,
            roll: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

fn metrics_from(state: &AppState) -> Metrics {
    let first_id = state.plan.drones.first().map(|drone| drone.id.clone());
    if let Some(frame) = state.current_frame() {
        let key = first_id
            .as_ref()
            .filter(|id| frame.positions.contains_key(*id))
            .or_else(|| frame.positions.keys().next());
        if let Some(p) = key.and_then(|id| frame.positions.get(id)) {
            return Metrics {
                alt: p.z,
                spd: p.speed,
                bat: p.battery_percent,
                hdg: p.heading,
                pitch: p.pitch,
                roll: p.roll,
                x: p.x,
                y: p.y,
                z: p.z,
            };
        }
    }
    if let Some(res) = first_id
        .as_ref()
        .and_then(|id| state.sim_results.get(id))
        .or_else(|| state.sim_results.values().next())
    {
        if let Some(p) = res.positions.last() {
            return Metrics {
                alt: p.z,
                spd: p.speed,
                bat: p.battery_percent,
                hdg: p.heading,
                pitch: p.pitch,
                roll: p.roll,
                x: p.x,
                y: p.y,
                z: p.z,
            };
        }
    }
    Metrics::default()
}

impl TelemetryPanel {
    pub fn new() -> TelemetryPanel {
        TelemetryPanel {
            chart_expanded: false,
        }
    }

    pub fn chart_expanded(&self) -> bool {
        self.chart_expanded
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        ui.label(RichText::new("Telemetry").strong());
        let status_color = if state.has_collision {
            Color32::from_rgb(248, 81, 73)
        } else if state.flying {
            Color32::from_rgb(63, 185, 80)
        } else {
            Color32::from_rgb(139, 148, 158)
        };
        ui.horizontal(|ui| {
            let (dot_rect, _) = ui.allocate_exact_size(Vec2::splat(14.0), Sense::hover());
            ui.painter()
                .circle_filled(dot_rect.center(), 5.0, status_color);
            ui.label(
                RichText::new(state.status_text.clone())
                    .monospace()
                    .color(status_color),
            );
            let hits: usize = state
                .sim_results
                .values()
                .map(|result| result.collisions.len())
                .sum();
            let hits_color = if hits > 0 {
                Color32::from_rgb(248, 81, 73)
            } else {
                Color32::from_rgb(139, 148, 158)
            };
            ui.label(
                RichText::new(format!("COLL {hits}"))
                    .monospace()
                    .color(hits_color),
            );
        });
        let m = metrics_from(state);
        let bat = if m.bat == 0.0 { 100.0 } else { m.bat };
        egui::Grid::new("telemetry_metrics").show(ui, |ui| {
            ui.label("ALT");
            ui.label("SPD");
            ui.label("BAT");
            ui.label("HDG");
            ui.end_row();
            ui.monospace(format!("{:.2} m", m.alt));
            ui.monospace(format!("{:.2} m/s", m.spd));
            ui.monospace(format!("{} %", bat.round() as i64));
            ui.monospace(format!("{} deg", m.hdg.round() as i64));
            ui.end_row();
            ui.label("PIT");
            ui.label("ROL");
            ui.end_row();
            ui.monospace(format!("{:+.1} deg", m.pitch));
            ui.monospace(format!("{:+.1} deg", m.roll));
            ui.end_row();
        });
        ui.monospace(format!("X: {:.2} Y: {:.2} Z: {:.2}", m.x, m.y, m.z));
        ui.horizontal(|ui| {
            ui.label("Speed Telemetry");
            let toggle = if self.chart_expanded { "Hide" } else { "Show" };
            if ui.button(toggle).clicked() {
                self.chart_expanded = !self.chart_expanded;
            }
        });
        if self.chart_expanded {
            let (series, min_v, max_v) = state.telemetry_series();
            draw_chart(ui, &series, min_v, max_v);
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Play").clicked() {
                state.play();
            }
            if ui.button("Stop").clicked() {
                state.pause();
            }
            if ui.button("Reset").clicked() {
                state.stop_reset();
            }
        });
        ui.horizontal(|ui| {
            let mut scrub = state.playback_scrub;
            let enabled = state.playback.playing || !state.sim_results.is_empty();
            if ui
                .add_enabled(
                    enabled,
                    egui::Slider::new(&mut scrub, 0.0..=1.0).show_value(false),
                )
                .changed()
            {
                state.set_scrub(scrub);
            }
        });
        ui.horizontal(|ui| {
            let mut speed = state.playback_speed;
            if ui.add(egui::Slider::new(&mut speed, 0.1..=20.0)).changed() {
                state.set_speed(speed);
            }
            ui.monospace(format!("{:.1}x", state.playback_speed));
        });
        ui.horizontal(|ui| {
            let mut cam = state.camera_mode;
            egui::ComboBox::from_id_salt("camera_mode")
                .selected_text(if cam == 2 { "Cam: Drone" } else { "Cam: Map" })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut cam, 1, "Cam: Map");
                    ui.selectable_value(&mut cam, 2, "Cam: Drone");
                });
            if cam != state.camera_mode {
                state.camera_mode = cam;
            }
            if ui.button("Reset Cam").clicked() {
                state.camera_reset_pending = true;
            }
        });
    }
}

fn draw_chart(ui: &mut Ui, series: &[f64], min_v: f64, max_v: f64) {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, 90.0), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, Color32::from_rgba_unmultiplied(13, 20, 31, 153));
    let pad_l = 36.0_f32;
    let pad_t = 16.0_f32;
    let cw = rect.width() - pad_l - 10.0;
    let ch = rect.height() - pad_t - 16.0;
    let left = rect.min.x + pad_l;
    let top = rect.min.y + pad_t;
    let grid_col = Color32::from_rgba_unmultiplied(118, 131, 144, 51);
    for i in 0..=4 {
        let y = top + ch * (i as f32 / 4.0);
        painter.line_segment(
            [Pos2::new(left, y), Pos2::new(left + cw, y)],
            Stroke::new(1.0, grid_col),
        );
    }
    for i in 0..=5 {
        let x = left + cw * (i as f32 / 5.0);
        painter.line_segment(
            [Pos2::new(x, top), Pos2::new(x, top + ch)],
            Stroke::new(1.0, grid_col),
        );
    }
    if series.len() < 2 {
        painter.text(
            Pos2::new(left + cw / 2.0, top + ch / 2.0),
            egui::Align2::CENTER_CENTER,
            "No telemetry data",
            egui::FontId::proportional(10.0),
            Color32::from_rgba_unmultiplied(118, 131, 144, 153),
        );
        return;
    }
    painter.line_segment(
        [Pos2::new(left, top + ch), Pos2::new(left + cw, top + ch)],
        Stroke::new(1.0, Color32::from_rgba_unmultiplied(118, 131, 144, 128)),
    );
    let accent = Color32::from_rgb(88, 166, 255);
    let n = series.len();
    let span = (max_v - min_v).max(1e-9);
    let mut pts: Vec<Pos2> = Vec::with_capacity(n);
    for (i, v) in series.iter().enumerate() {
        let x = left + cw * (i as f32 / (n - 1) as f32);
        let y = top + ch * (1.0 - ((*v - min_v) / span) as f32);
        pts.push(Pos2::new(x, y));
    }
    for pair in pts.windows(2) {
        painter.line_segment([pair[0], pair[1]], Stroke::new(2.0, accent));
    }
    if let Some(last) = pts.last() {
        painter.circle_filled(*last, 3.0, accent);
    }
    let label_col = Color32::from_rgba_unmultiplied(118, 131, 144, 204);
    for i in 0..=4 {
        let v = min_v + (max_v - min_v) * (1.0 - i as f64 / 4.0);
        painter.text(
            Pos2::new(left - 6.0, top + ch * (i as f32 / 4.0)),
            egui::Align2::RIGHT_CENTER,
            format!("{v:.1}"),
            egui::FontId::monospace(9.0),
            label_col,
        );
    }
    for i in 0..=5 {
        let idx = ((n - 1) as f64 * i as f64 / 5.0).round() as usize;
        painter.text(
            Pos2::new(left + cw * (i as f32 / 5.0), top + ch + 4.0),
            egui::Align2::CENTER_TOP,
            format!("{idx}"),
            egui::FontId::monospace(9.0),
            label_col,
        );
    }
    painter.text(
        Pos2::new(left, rect.min.y + 3.0),
        egui::Align2::LEFT_TOP,
        "Speed (m/s)",
        egui::FontId::proportional(9.0),
        Color32::from_rgb(139, 148, 158),
    );
    painter.text(
        Pos2::new(left + cw, rect.min.y + 3.0),
        egui::Align2::RIGHT_TOP,
        "samples",
        egui::FontId::proportional(9.0),
        Color32::from_rgb(139, 148, 158),
    );
}
