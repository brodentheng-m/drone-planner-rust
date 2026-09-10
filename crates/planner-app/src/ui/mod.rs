use crate::state::AppState;
use egui::{Color32, CornerRadius, Stroke, Ui, Visuals};
use std::time::Duration;

pub mod code_panel;
pub mod obstacles_panel;
pub mod palette;
pub mod plan_tree;
pub mod telemetry;
pub mod viewport;

const BG: Color32 = Color32::from_rgb(0x0d, 0x11, 0x17);
const PANEL: Color32 = Color32::from_rgb(0x16, 0x1b, 0x22);
const DIM: Color32 = Color32::from_rgb(0x8b, 0x94, 0x9e);
const BORDER: Color32 = Color32::from_rgb(0x21, 0x26, 0x2d);

fn apply_theme(ctx: &egui::Context) {
    let mut visuals = Visuals::dark();
    visuals.panel_fill = PANEL;
    visuals.window_fill = BG;
    visuals.extreme_bg_color = BG;
    visuals.selection.bg_fill = Color32::from_rgba_unmultiplied(0x58, 0xa6, 0xff, 77);
    visuals.weak_text_color = Some(DIM);
    visuals.window_stroke = Stroke::new(1.0, BORDER);
    for w in [
        &mut visuals.widgets.noninteractive,
        &mut visuals.widgets.inactive,
        &mut visuals.widgets.hovered,
        &mut visuals.widgets.active,
        &mut visuals.widgets.open,
    ] {
        w.corner_radius = CornerRadius::from(4u8);
        w.bg_stroke = Stroke::new(1.0, BORDER);
    }
    ctx.set_visuals(visuals);
}

pub struct UiRoot {
    pub plan_tree: plan_tree::PlanTreePanel,
    pub palette: palette::PalettePanel,
    pub viewport: viewport::ViewportPanel,
    pub telemetry: telemetry::TelemetryPanel,
    pub code_panel: code_panel::CodePanel,
    pub obstacles_panel: obstacles_panel::ObstaclesPanel,
    theme_applied: bool,
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
            theme_applied: false,
        }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        if !self.theme_applied {
            apply_theme(ui.ctx());
            self.theme_applied = true;
        }
        if state.playback.playing {
            ui.ctx().request_repaint_after(Duration::from_millis(33));
        }
        let frame = egui::Frame::central_panel(ui.style()).fill(BG);
        egui::CentralPanel::default().frame(frame).show(ui, |ui| {
            self.obstacles_panel.show(ui, state);
            self.telemetry.show(ui, state);
            self.code_panel.show(ui, state);
            self.palette.show(ui, state);
            self.plan_tree.show(ui, state);
            self.viewport.show(ui, state);
        });
    }
}
