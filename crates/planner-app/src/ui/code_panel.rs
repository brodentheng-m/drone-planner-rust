use crate::state::AppState;
use egui::Ui;
use planner_core::codegen;

pub struct CodePanel;

impl Default for CodePanel {
    fn default() -> Self {
        CodePanel
    }
}

impl CodePanel {
    pub fn new() -> CodePanel {
        CodePanel
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::right("code_panel")
            .default_size(340.0)
            .show(ui, |ui| {
                ui.heading("Generated Python");
                if ui.button("Copy").clicked() {
                    ui.ctx().copy_text(codegen::generate_code(&state.plan));
                }
                ui.separator();
                egui::ScrollArea::both().show(ui, |ui| {
                    ui.style_mut().override_text_style = Some(egui::TextStyle::Monospace);
                    ui.code(codegen::generate_code(&state.plan));
                });
            });
        egui::Panel::bottom("console_strip")
            .default_size(120.0)
            .show(ui, |ui| {
                ui.heading("Console");
                egui::ScrollArea::vertical()
                    .max_height(110.0)
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        ui.style_mut().override_text_style = Some(egui::TextStyle::Monospace);
                        for line in &state.console {
                            ui.label(line.clone());
                        }
                    });
            });
    }
}
