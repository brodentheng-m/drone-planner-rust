use crate::state::AppState;
use egui::Ui;

pub struct TelemetryPanel;

impl Default for TelemetryPanel {
    fn default() -> Self {
        TelemetryPanel
    }
}

impl TelemetryPanel {
    pub fn new() -> TelemetryPanel {
        TelemetryPanel
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::right("telemetry_panel")
            .default_size(300.0)
            .show(ui, |ui| {
                ui.heading("Telemetry");
                ui.separator();
                let last = state.sim_result.as_ref().and_then(|r| r.positions.last());
                let altitude = last.map(|p| p.z).unwrap_or(0.0);
                let speed = last.map(|p| p.speed).unwrap_or(0.0);
                let battery = last.map(|p| p.battery_percent).unwrap_or(0.0);
                let heading = last.map(|p| p.heading).unwrap_or(0.0);
                ui.monospace(format!("altitude {altitude:.2}"));
                ui.monospace(format!("speed {speed:.2}"));
                ui.monospace(format!("battery {battery:.0}"));
                ui.monospace(format!("heading {heading:.1}"));
                ui.separator();
                ui.label("Speed over time");
                ui.label("Status");
            });
    }
}
