use crate::state::AppState;
use egui::Ui;

pub struct PalettePanel;

impl Default for PalettePanel {
    fn default() -> Self {
        PalettePanel
    }
}

impl PalettePanel {
    pub fn new() -> PalettePanel {
        PalettePanel
    }

    pub fn show(&mut self, ui: &mut Ui, _state: &mut AppState) {
        egui::Panel::left("palette_panel")
            .default_size(220.0)
            .show(ui, |ui| {
                ui.heading("Command Palette");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.label("Command groups");
                });
            });
    }
}
