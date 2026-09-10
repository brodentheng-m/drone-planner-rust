use crate::state::AppState;
use egui::Ui;

pub mod code_panel;
pub mod obstacles_panel;
pub mod palette;
pub mod plan_tree;
pub mod telemetry;
pub mod viewport;

pub struct UiRoot {
    pub plan_tree: plan_tree::PlanTreePanel,
    pub palette: palette::PalettePanel,
    pub viewport: viewport::ViewportPanel,
    pub telemetry: telemetry::TelemetryPanel,
    pub code_panel: code_panel::CodePanel,
    pub obstacles_panel: obstacles_panel::ObstaclesPanel,
}

impl UiRoot {
    pub fn new() -> UiRoot {
        UiRoot {
            plan_tree: plan_tree::PlanTreePanel::new(),
            palette: palette::PalettePanel::new(),
            viewport: viewport::ViewportPanel::new(),
            telemetry: telemetry::TelemetryPanel::new(),
            code_panel: code_panel::CodePanel::new(),
            obstacles_panel: obstacles_panel::ObstaclesPanel::new(),
        }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::top("top_panel").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Drone Planner");
            });
        });
        self.plan_tree.show(ui, state);
        self.palette.show(ui, state);
        self.telemetry.show(ui, state);
        self.code_panel.show(ui, state);
        self.obstacles_panel.show(ui, state);
        self.viewport.show(ui, state);
    }
}
