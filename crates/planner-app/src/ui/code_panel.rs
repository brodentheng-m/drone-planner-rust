use crate::state::AppState;
use egui::Ui;
use planner_core::codegen;
use std::time::{Duration, Instant};

pub struct CodePanel {
    edited: String,
    dirty: bool,
    copied_at: Option<Instant>,
}

impl Default for CodePanel {
    fn default() -> Self {
        CodePanel {
            edited: String::new(),
            dirty: false,
            copied_at: None,
        }
    }
}

impl CodePanel {
    pub fn new() -> CodePanel {
        CodePanel::default()
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::right("code_panel")
            .default_size(340.0)
            .show(ui, |ui| {
                let generated = if state.plan.drones.len() > 1 {
                    codegen::generate_swarm_code(&state.plan)
                } else {
                    codegen::generate_code(&state.plan)
                };
                if !self.dirty {
                    self.edited = generated.clone();
                }
                ui.horizontal(|ui| {
                    ui.heading("Generated Code");
                    let flashing = self
                        .copied_at
                        .map(|t| t.elapsed() < Duration::from_millis(1500))
                        .unwrap_or(false);
                    let label = if flashing { "Copied!" } else { "Copy" };
                    if ui.button(label).clicked() {
                        ui.ctx().copy_text(generated.clone());
                        self.copied_at = Some(Instant::now());
                    }
                    if flashing {
                        ui.ctx().request_repaint_after(Duration::from_millis(200));
                    }
                    if ui.button("Apply").clicked() {
                        let src = self.edited.clone();
                        match state.import_code(&src) {
                            Ok(_) => {
                                self.dirty = false;
                                self.edited.clear();
                            }
                            Err(err) => {
                                state.console.push(format!("[error] Error applying code: {err}"));
                            }
                        }
                    }
                });
                ui.separator();
                let highlight: i64 = if state.playback.playing {
                    state
                        .current_frame()
                        .map(|f| f.command_index)
                        .unwrap_or(-1)
                } else {
                    -1
                };
                egui::ScrollArea::vertical().show(ui, |ui| {
                    if highlight >= 0 {
                        let idx = highlight as usize;
                        let mut display = self
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
                            egui::TextEdit::multiline(&mut self.edited)
                                .code_editor()
                                .desired_width(f32::INFINITY),
                        );
                        if resp.changed() {
                            self.dirty = true;
                        }
                    }
                });
            });
        egui::Panel::bottom("console_strip")
            .default_size(120.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Console");
                    if ui.button("Clear").clicked() {
                        state.console.clear();
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for line in &state.console {
                            ui.label(
                                egui::RichText::new(line.as_str())
                                    .monospace()
                                    .color(console_color(line)),
                            );
                        }
                    });
            });
    }
}

fn console_color(line: &str) -> egui::Color32 {
    if line.starts_with("[success]") {
        egui::Color32::from_rgb(63, 185, 80)
    } else if line.starts_with("[warn]") {
        egui::Color32::from_rgb(210, 153, 34)
    } else if line.starts_with("[error]") {
        egui::Color32::from_rgb(248, 81, 73)
    } else {
        egui::Color32::from_rgb(139, 148, 158)
    }
}
