use crate::state::AppState;
use egui::Ui;

pub struct PlanTreePanel;

impl Default for PlanTreePanel {
    fn default() -> Self {
        PlanTreePanel
    }
}

impl PlanTreePanel {
    pub fn new() -> PlanTreePanel {
        PlanTreePanel
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::left("plan_tree_panel")
            .default_size(280.0)
            .show(ui, |ui| {
                ui.heading("Plan Tree");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.label(state.plan.name.clone());
                    for drone in &state.plan.drones {
                        let header = format!("{} ({})", drone.name, drone.commands.len());
                        egui::CollapsingHeader::new(header)
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.label("Commands");
                            });
                    }
                });
            });
    }
}
