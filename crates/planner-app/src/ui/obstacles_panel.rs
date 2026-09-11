use crate::state::AppState;
use egui::{Color32, RichText, Ui};
use planner_core::codegen;
use planner_core::obstacles::{self, Obstacle};
use planner_core::planio::{ObstacleFile, save_obstacles};
use std::time::{Duration, Instant};

const TYPE_NAMES: [&str; 6] = ["wall", "tower", "hoop", "cone", "sphere", "square"];
pub const RIGHT_PANEL_WIDTH: f32 = 272.0;

struct CodeSection {
    edited: String,
    dirty: bool,
    copied_at: Option<Instant>,
    last_generated: String,
}

impl Default for CodeSection {
    fn default() -> Self {
        CodeSection {
            edited: String::new(),
            dirty: false,
            copied_at: None,
            last_generated: String::new(),
        }
    }
}

pub struct ObstaclesPanel {
    expanded: Option<usize>,
    last_rejected: Option<usize>,
    edit_pending: bool,
    code: CodeSection,
    console_open: bool,
}

impl Default for ObstaclesPanel {
    fn default() -> Self {
        ObstaclesPanel::new()
    }
}

impl ObstaclesPanel {
    pub fn new() -> ObstaclesPanel {
        ObstaclesPanel {
            expanded: None,
            last_rejected: None,
            edit_pending: false,
            code: CodeSection::default(),
            console_open: false,
        }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        if !state.show_right {
            return;
        }
        egui::Panel::right("right_panel")
            .exact_size(RIGHT_PANEL_WIDTH)
            .resizable(false)
            .show(ui, |ui| {
                let available = ui.available_height();
                let obstacles_height = (available * 0.42).max(120.0);
                egui::ScrollArea::vertical()
                    .id_salt("obstacle_list_scroll")
                    .max_height(obstacles_height)
                    .show(ui, |ui| {
                        self.obstacle_section(ui, state);
                    });
                ui.separator();
                let code_height = (available * 0.30).max(90.0);
                self.code_section(ui, state, code_height);
                ui.separator();
                self.console_section(ui, state);
            });
    }

    fn obstacle_section(&mut self, ui: &mut Ui, state: &mut AppState) {
        ui.heading("Obstacles");
        ui.horizontal_wrapped(|ui| {
            if ui.button("Toggle Obstacles").clicked() {
                state.toggle_obstacle_visibility();
                let text = if state.obstacles_visible {
                    "Obstacles shown"
                } else {
                    "Obstacles hidden"
                };
                state.log(text);
            }
            if ui.button("Clear All").clicked() {
                state.obstacle_store_mut().clear();
                self.expanded = None;
                state.log("All obstacles cleared");
                state.refresh_sim();
            }
            if ui.button("Reload Base").clicked() {
                *state.obstacle_store_mut() = obstacles::BASE_OBSTACLES.clone();
                self.expanded = None;
                state.log("Base obstacles reloaded");
                state.refresh_sim();
            }
            if ui.button("Import Obstacles").clicked() {
                import_dialog(self, state);
            }
            if ui.button("Toggle Boundary").clicked() {
                state.boundary_visible = !state.boundary_visible;
                let text = if state.boundary_visible {
                    "Boundary shown"
                } else {
                    "Boundary hidden"
                };
                state.log(text);
            }
            if ui.button("Export").clicked() {
                export_dialog(state);
            }
        });
        ui.horizontal(|ui| {
            let boundary_text = if state.boundary_visible {
                "Boundary: shown"
            } else {
                "Boundary: hidden"
            };
            ui.monospace(boundary_text);
            if let Some(rejected) = self.last_rejected {
                let color = if rejected > 0 {
                    Color32::from_rgb(210, 153, 34)
                } else {
                    Color32::from_rgb(139, 148, 158)
                };
                ui.label(
                    RichText::new(format!("Rejected: {rejected}"))
                        .monospace()
                        .color(color),
                );
            }
        });
        ui.separator();
        let mut remove: Option<usize> = None;
        let mut toggle: Option<usize> = None;
        let mut edited = false;
        for index in 0..state.obstacle_store().len() {
            let (obstacle_type, name, pos) = {
                let obstacle = &state.obstacle_store()[index];
                (
                    obstacle.obstacle_type.clone(),
                    obstacle.name.clone(),
                    obstacle.position,
                )
            };
            ui.horizontal(|ui| {
                if ui
                    .selectable_label(
                        self.expanded == Some(index),
                        format!(
                            "{obstacle_type} {name} ({:.1}, {:.1}, {:.1})",
                            pos[0], pos[1], pos[2]
                        ),
                    )
                    .clicked()
                {
                    toggle = Some(index);
                }
                if ui.button("X").clicked() {
                    remove = Some(index);
                }
            });
            if self.expanded == Some(index) {
                let obstacle = &mut state.obstacle_store_mut()[index];
                ui.horizontal(|ui| {
                    ui.label("pos");
                    for axis in 0..3 {
                        edited |= ui
                            .add(egui::DragValue::new(&mut obstacle.position[axis]).speed(0.05))
                            .changed();
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("rot y");
                    edited |= ui
                        .add(egui::Slider::new(
                            &mut obstacle.rotation[1],
                            0.0..=std::f64::consts::TAU,
                        ))
                        .changed();
                });
                ui.horizontal(|ui| {
                    ui.label("scale");
                    for axis in 0..3 {
                        edited |= ui
                            .add(egui::DragValue::new(&mut obstacle.scale[axis]).speed(0.02))
                            .changed();
                    }
                });
            }
        }
        if let Some(index) = toggle {
            self.expanded = if self.expanded == Some(index) {
                None
            } else {
                Some(index)
            };
        }
        if let Some(index) = remove {
            let removed = state.obstacle_store_mut().remove(index);
            state.log(format!("Obstacle removed: {}", removed.name));
            self.expanded = match self.expanded {
                Some(current) if current == index => None,
                Some(current) if current > index => Some(current - 1),
                other => other,
            };
            state.refresh_sim();
        }
        if edited {
            self.edit_pending = true;
        }
        if self.edit_pending && !ui.input(|input| input.pointer.primary_down()) {
            self.edit_pending = false;
            state.refresh_sim();
        }
        ui.separator();
        ui.label("Add obstacle:");
        let mut added: Option<&str> = None;
        ui.horizontal_wrapped(|ui| {
            for type_name in TYPE_NAMES {
                if ui.button(type_name).clicked() {
                    added = Some(type_name);
                }
            }
        });
        if let Some(type_name) = added {
            add_obstacle(state, type_name);
            state.refresh_sim();
        }
    }

    fn code_section(&mut self, ui: &mut Ui, state: &mut AppState, editor_height: f32) {
        let generated = if state.plan.drones.len() > 1 {
            codegen::generate_swarm_code(&state.plan)
        } else {
            codegen::generate_code(&state.plan)
        };
        if generated != self.code.last_generated {
            self.code.last_generated = generated.clone();
            self.code.dirty = false;
        }
        if !self.code.dirty {
            self.code.edited = generated.clone();
        }
        ui.horizontal(|ui| {
            ui.label(RichText::new("Generated Code").strong());
            let flashing = self
                .code
                .copied_at
                .map(|t| t.elapsed() < Duration::from_millis(1500))
                .unwrap_or(false);
            let label = if flashing { "Copied!" } else { "Copy" };
            if ui.button(label).clicked() {
                ui.ctx().copy_text(generated.clone());
                self.code.copied_at = Some(Instant::now());
            }
            if flashing {
                ui.ctx().request_repaint_after(Duration::from_millis(200));
            }
            if ui.button("Apply").clicked() {
                let src = self.code.edited.clone();
                match state.import_code(&src) {
                    Ok(_) => {
                        self.code.dirty = false;
                        self.code.edited.clear();
                    }
                    Err(err) => {
                        state.log_level("error", format!("Error applying code: {err}"));
                    }
                }
            }
        });
        let highlight: i64 = if state.playback.playing {
            state.current_frame().map(|f| f.command_index).unwrap_or(-1)
        } else {
            -1
        };
        egui::ScrollArea::vertical()
            .id_salt("code_preview_scroll")
            .max_height(editor_height)
            .show(ui, |ui| {
                if highlight >= 0 {
                    let idx = highlight as usize;
                    let mut display = self
                        .code
                        .edited
                        .split('\n')
                        .enumerate()
                        .map(|(i, line)| {
                            if i == idx {
                                format!("> {line}")
                            } else {
                                line.to_string()
                            }
                        })
                        .collect::<Vec<String>>()
                        .join("\n");
                    ui.add(
                        egui::TextEdit::multiline(&mut display)
                            .code_editor()
                            .interactive(false)
                            .desired_width(f32::INFINITY),
                    );
                } else {
                    let resp = ui.add(
                        egui::TextEdit::multiline(&mut self.code.edited)
                            .code_editor()
                            .desired_width(f32::INFINITY),
                    );
                    if resp.changed() {
                        self.code.dirty = true;
                    }
                }
            });
    }

    fn console_section(&mut self, ui: &mut Ui, state: &mut AppState) {
        ui.horizontal(|ui| {
            let mark = if self.console_open { "[-]" } else { "[+]" };
            if ui.button(mark).clicked() {
                self.console_open = !self.console_open;
            }
            ui.label(RichText::new("Console").strong());
            if self.console_open && ui.button("Clear").clicked() {
                state.console.clear();
            }
        });
        if self.console_open {
            egui::ScrollArea::vertical()
                .id_salt("console_strip_scroll")
                .max_height(110.0)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for line in &state.console {
                        ui.label(
                            RichText::new(line.as_str())
                                .monospace()
                                .color(console_color(line)),
                        );
                    }
                });
        }
    }
}

fn console_color(line: &str) -> Color32 {
    if line.starts_with("[error]") || line.contains("COLLISION") || line.contains("Error") {
        Color32::from_rgb(248, 81, 73)
    } else if line.starts_with("[warn]") || line.contains("Rejected") || line.contains("Cannot") {
        Color32::from_rgb(210, 153, 34)
    } else if line.starts_with("[success]") || line.contains("success") {
        Color32::from_rgb(63, 185, 80)
    } else {
        Color32::from_rgb(139, 148, 158)
    }
}

fn add_obstacle(state: &mut AppState, type_name: &str) {
    let name = {
        let store = state.obstacle_store_mut();
        let mut number = store.len() + 1;
        let id = loop {
            let candidate = format!("obs_{type_name}_{number}");
            if !store.iter().any(|obstacle| obstacle.id == candidate) {
                break candidate;
            }
            number += 1;
        };
        let name = format!("{type_name} {number}");
        store.push(Obstacle {
            id,
            obstacle_type: type_name.to_string(),
            position: [0.0, 0.5, 0.0],
            rotation: [0.0, 0.0, 0.0],
            scale: [1.0, 1.0, 1.0],
            name: name.clone(),
            color: None,
        });
        name
    };
    state.log(format!("Added obstacle: {name}"));
}

fn import_dialog(panel: &mut ObstaclesPanel, state: &mut AppState) {
    let path = rfd::FileDialog::new()
        .add_filter("Obstacle files", &["json", "geojson", "csv", "obj"])
        .pick_file();
    let Some(path) = path else { return };
    let filename = path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    match std::fs::read_to_string(&path) {
        Ok(text) => {
            if let Ok(rejected) = state.import_obstacle_text(&text, &filename) {
                panel.last_rejected = Some(rejected);
            }
        }
        Err(err) => state.log_level("error", format!("Error importing obstacles: {err}")),
    }
}

fn export_dialog(state: &mut AppState) {
    let path = rfd::FileDialog::new()
        .add_filter("Obstacle file", &["json"])
        .set_file_name("obstacles.json")
        .save_file();
    let Some(path) = path else { return };
    let file = ObstacleFile {
        obstacles: state.obstacle_store().clone(),
        boundary: Some(state.obstacles.boundary),
    };
    let text_path = path.to_string_lossy().into_owned();
    match save_obstacles(&text_path, &file) {
        Ok(()) => state.log("Obstacles exported"),
        Err(err) => state.log_level("error", format!("Error exporting obstacles: {err}")),
    }
}
