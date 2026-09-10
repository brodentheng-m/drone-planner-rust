use crate::state::AppState;
use egui::Ui;

pub struct ViewportPanel;

impl Default for ViewportPanel {
    fn default() -> Self {
        ViewportPanel
    }
}

impl ViewportPanel {
    pub fn new() -> ViewportPanel {
        ViewportPanel
    }

    pub fn show(&mut self, ui: &mut Ui, _state: &mut AppState) {
        egui::CentralPanel::default().show(ui, |ui| {
            ui.centered_and_justified(|ui| {
                ui.label("Drone Planner");
            });
        });
    }
}
