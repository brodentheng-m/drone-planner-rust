mod state;
mod ui;

use state::AppState;

fn main() {
    if std::env::var("DP_SMOKE").as_deref() == Ok("1") {
        println!("planner-app smoke ok");
        return;
    }
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        viewport: egui::ViewportBuilder::default()
            .with_title("Drone Planner")
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([1280.0, 800.0]),
        ..Default::default()
    };
    let result = eframe::run_native(
        "Drone Planner",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(DronePlannerApp::new()))
        }),
    );
    if let Err(err) = result {
        eprintln!("eframe run failed: {err}");
    }
}

struct DronePlannerApp {
    state: AppState,
    root: ui::UiRoot,
}

impl DronePlannerApp {
    fn new() -> DronePlannerApp {
        DronePlannerApp {
            state: AppState::new(),
            root: ui::UiRoot::new(),
        }
    }
}

impl eframe::App for DronePlannerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.root.show(ui, &mut self.state);
    }
}
