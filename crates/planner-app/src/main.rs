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
        depth_buffer: 24,
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
    if let Err(_err) = result {
        eprintln!(
            "GL context failed - install mesa llvmpipe opengl32.dll next to the exe (see README)"
        );
        std::process::exit(0);
    }
}

struct DronePlannerApp {
    state: AppState,
    root: ui::UiRoot,
    renaming: bool,
    rename_focus_pending: bool,
    layout_defaults_applied: bool,
}

impl DronePlannerApp {
    fn new() -> DronePlannerApp {
        DronePlannerApp {
            state: AppState::new(),
            root: ui::UiRoot::new(),
            renaming: false,
            rename_focus_pending: false,
            layout_defaults_applied: false,
        }
    }

    fn apply_layout_defaults(&mut self, ui: &egui::Ui) {
        if self.layout_defaults_applied {
            return;
        }
        self.layout_defaults_applied = true;
        if ui.available_width() < 1400.0 {
            self.state.show_right = false;
        }
    }

    fn new_plan(state: &mut AppState) {
        state.plan = planner_core::planio::Plan::default_plan();
        if state.plan.active_drone_id.is_none() {
            state.plan.active_drone_id = state.plan.drones.first().map(|drone| drone.id.clone());
        }
        state.selection = Default::default();
        state.mark_saved();
        state.refresh_sim();
        state.log("New flight plan created");
    }

    fn save_plan_dialog(state: &mut AppState) {
        state.log("Saving flight plan...");
        let path = rfd::FileDialog::new()
            .add_filter("Flight plan", &["flight"])
            .set_file_name(&format!("{}.flight", state.plan.name))
            .save_file();
        let Some(path) = path else { return };
        let text = state.export_plan_json();
        if text.is_empty() {
            state.log_level("error", "Error saving flight plan: empty export");
            return;
        }
        match std::fs::write(&path, text) {
            Ok(()) => {
                state.mark_saved();
                state.log("Flight plan saved");
            }
            Err(err) => state.log_level("error", format!("Error saving flight plan: {err}")),
        }
    }

    fn load_dialog(state: &mut AppState) {
        state.log("Loading flight plan...");
        let path = rfd::FileDialog::new()
            .add_filter("Flight plans and scripts", &["flight", "json", "py"])
            .pick_file();
        let Some(path) = path else { return };
        let ext = path
            .extension()
            .and_then(|ext| ext.to_str())
            .unwrap_or("")
            .to_lowercase();
        if ext == "py" {
            match std::fs::read_to_string(&path) {
                Ok(text) => Self::import_code_text(state, &text),
                Err(err) => state.log_level("error", format!("Error loading file: {err}")),
            }
            return;
        }
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                let _ = state.import_plan_json(&text);
            }
            Err(err) => state.log_level("error", format!("Error loading file: {err}")),
        }
    }

    fn import_obstacles_dialog(state: &mut AppState) {
        state.log("Importing obstacles...");
        let path = rfd::FileDialog::new()
            .add_filter("Obstacle files", &["json", "geojson", "csv", "obj"])
            .pick_file();
        let Some(path) = path else { return };
        let filename = path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_default();
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                let _ = state.import_obstacle_text(&text, &filename);
            }
            Err(err) => state.log_level("error", format!("Error importing obstacles: {err}")),
        }
    }

    fn import_py_dialog(state: &mut AppState) {
        let path = rfd::FileDialog::new()
            .add_filter("Python", &["py"])
            .pick_file();
        let Some(path) = path else { return };
        match std::fs::read_to_string(&path) {
            Ok(text) => Self::import_code_text(state, &text),
            Err(err) => state.log_level("error", format!("Error loading file: {err}")),
        }
    }

    fn import_code_text(state: &mut AppState, text: &str) {
        state.selection = Default::default();
        if let Ok(count) = state.import_code(text) {
            let name = state
                .active_drone_index()
                .and_then(|index| state.plan.drones.get(index))
                .map(|drone| drone.name.clone())
                .unwrap_or_default();
            state.log(format!("Imported {count} commands into {name}"));
        }
    }

    fn export_swarm(state: &mut AppState) {
        let code = planner_core::codegen::generate_swarm_code(&state.plan);
        Self::export_code_dialog(state, &format!("{}.py", state.plan.name), code);
    }

    fn export_single(state: &mut AppState) {
        let Some(index) = state.active_drone_index() else {
            return;
        };
        let Some(drone) = state.plan.drones.get(index) else {
            return;
        };
        let mut plan = state.plan.clone();
        plan.active_drone_id = Some(drone.id.clone());
        let code = planner_core::codegen::generate_code(&plan);
        Self::export_code_dialog(state, &format!("{}.py", drone.name), code);
    }

    fn export_animation(state: &mut AppState) {
        let mut plan = state.plan.clone();
        if let Some(index) = state.active_drone_index() {
            plan.active_drone_id = Some(plan.drones[index].id.clone());
        }
        let code = planner_core::codegen::generate_animation_code(&plan);
        Self::export_code_dialog(state, &format!("{}_sim.py", state.plan.name), code);
    }

    fn export_code_dialog(state: &mut AppState, file_name: &str, code: String) {
        let path = rfd::FileDialog::new()
            .add_filter("Python", &["py"])
            .set_file_name(file_name)
            .save_file();
        let Some(path) = path else { return };
        if let Err(err) = std::fs::write(&path, code) {
            state.log_level("error", format!("Error exporting file: {err}"));
        }
    }
}

impl eframe::App for DronePlannerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.apply_layout_defaults(ui);
        self.state.begin_frame();
        self.state.sync_camera_mode();
        if self.state.playback.playing {
            ui.ctx().request_repaint();
        }
        let plan_label = if self.state.dirty {
            format!("{} *", self.state.plan.name)
        } else {
            self.state.plan.name.clone()
        };
        egui::Panel::top("toolbar")
            .exact_size(36.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.add_space(4.0);
                    if ui.button("New").clicked() {
                        Self::new_plan(&mut self.state);
                    }
                    if ui.button("Save").clicked() {
                        Self::save_plan_dialog(&mut self.state);
                    }
                    if ui.button("Load").clicked() {
                        Self::load_dialog(&mut self.state);
                    }
                    ui.separator();
                    if ui.button("Import .py").clicked() {
                        Self::import_py_dialog(&mut self.state);
                    }
                    if ui.button("Import Obstacles").clicked() {
                        Self::import_obstacles_dialog(&mut self.state);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.add_space(4.0);
                        ui.toggle_value(&mut self.state.show_right, "Right Panel");
                        ui.toggle_value(&mut self.state.show_left, "Left Panel");
                        ui.separator();
                        if ui.button("Export Animation").clicked() {
                            Self::export_animation(&mut self.state);
                        }
                        if ui.button("Export Single").clicked() {
                            Self::export_single(&mut self.state);
                        }
                        if ui.button("Export Swarm").clicked() {
                            Self::export_swarm(&mut self.state);
                        }
                    });
                    ui.with_layout(
                        egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                        |ui| {
                            if self.renaming {
                                let response = ui.add(
                                    egui::TextEdit::singleline(&mut self.state.plan.name)
                                        .desired_width(240.0),
                                );
                                if self.rename_focus_pending {
                                    response.request_focus();
                                    self.rename_focus_pending = false;
                                }
                                if response.changed() {
                                    self.state.mark_dirty();
                                }
                                if response.lost_focus() {
                                    self.renaming = false;
                                }
                            } else if ui.button(plan_label.clone()).clicked() {
                                self.renaming = true;
                                self.rename_focus_pending = true;
                            }
                        },
                    );
                });
            });
        self.root.show(ui, &mut self.state);
    }
}
