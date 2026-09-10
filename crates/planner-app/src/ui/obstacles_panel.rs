use crate::state::AppState;
use egui::Ui;

pub struct ObstaclesPanel;

impl Default for ObstaclesPanel {
    fn default() -> Self {
        ObstaclesPanel
    }
}

impl ObstaclesPanel {
    pub fn new() -> ObstaclesPanel {
        ObstaclesPanel
    }

    pub fn show(&mut self, ui: &mut Ui, _state: &mut AppState) {
        egui::Panel::right("obstacles_panel")
            .default_size(260.0)
            .show(ui, |ui| {
                ui.heading("Obstacles");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.label("Obstacle list");
                });
            });
    }
}
