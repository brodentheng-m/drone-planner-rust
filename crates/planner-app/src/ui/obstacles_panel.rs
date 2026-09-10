use crate::state::AppState;
use egui::Ui;
use planner_core::obstacles::{self, Obstacle};
use planner_core::planio::{save_obstacles, ObstacleFile};

const TYPE_NAMES: [&str; 6] = ["wall", "tower", "hoop", "cone", "sphere", "square"];

pub struct ObstaclesPanel {
    pub obstacles_visible: bool,
    expanded: Option<usize>,
    add_type: String,
}

impl Default for ObstaclesPanel {
    fn default() -> Self {
        ObstaclesPanel::new()
    }
}

impl ObstaclesPanel {
    pub fn new() -> ObstaclesPanel {
        ObstaclesPanel {
            obstacles_visible: true,
            expanded: None,
            add_type: "wall".to_string(),
        }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::right("obstacles_panel")
            .default_size(260.0)
            .show(ui, |ui| {
                ui.heading("Obstacles");
                ui.horizontal_wrapped(|ui| {
                    if ui.button("Toggle Obstacles").clicked() {
                        self.obstacles_visible = !self.obstacles_visible;
                        let text = if self.obstacles_visible {
                            "Obstacles shown"
                        } else {
                            "Obstacles hidden"
                        };
                        state.log(text);
                    }
                    if ui.button("Clear All").clicked() {
                        state.obstacles.obstacles.clear();
                        self.expanded = None;
                        state.log("All obstacles cleared");
                    }
                    if ui.button("Reload Base").clicked() {
                        state.obstacles.obstacles = obstacles::BASE_OBSTACLES.clone();
                        self.expanded = None;
                        state.log("Base obstacles reloaded");
                    }
                    if ui.button("Import Obstacles").clicked() {
                        import_dialog(state);
                    }
                    if ui.button("Toggle Boundary").clicked() {
                        state.boundary_visible = !state.boundary_visible;
                        let text = if state.boundary_visible {
                            "Boundary shown"
                        } else {
                            "Boundary hidden"
                        };
                        state.log(text);
                    }
                    if ui.button("Export").clicked() {
                        export_dialog(state);
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let mut remove: Option<usize> = None;
                    let mut toggle: Option<usize> = None;
                    for index in 0..state.obstacles.obstacles.len() {
                        let (obstacle_type, name) = {
                            let obstacle = &state.obstacles.obstacles[index];
                            (obstacle.obstacle_type.clone(), obstacle.name.clone())
                        };
                        ui.horizontal(|ui| {
                            if ui
                                .selectable_label(
                                    self.expanded == Some(index),
                                    format!("{obstacle_type} {name}"),
                                )
                                .clicked()
                            {
                                toggle = Some(index);
                            }
                            if ui.button("X").clicked() {
                                remove = Some(index);
                            }
                        });
                        if self.expanded == Some(index) {
                            let obstacle = &mut state.obstacles.obstacles[index];
                            ui.horizontal(|ui| {
                                ui.label("pos");
                                for axis in 0..3 {
                                    ui.add(
                                        egui::DragValue::new(&mut obstacle.position[axis])
                                            .speed(0.05),
                                    );
                                }
                            });
                            ui.horizontal(|ui| {
                                ui.label("rot");
                                for axis in 0..3 {
                                    ui.add(
                                        egui::DragValue::new(&mut obstacle.rotation[axis])
                                            .speed(0.02),
                                    );
                                }
                            });
                            ui.horizontal(|ui| {
                                ui.label("scale");
                                for axis in 0..3 {
                                    ui.add(
                                        egui::DragValue::new(&mut obstacle.scale[axis])
                                            .speed(0.02),
                                    );
                                }
                            });
                            egui::ComboBox::from_id_salt(("obstacle_type", index))
                                .selected_text(obstacle.obstacle_type.clone())
                                .show_ui(ui, |ui| {
                                    for type_name in TYPE_NAMES {
                                        ui.selectable_value(
                                            &mut obstacle.obstacle_type,
                                            type_name.to_string(),
                                            type_name,
                                        );
                                    }
                                });
                        }
                    }
                    if let Some(index) = toggle {
                        self.expanded = if self.expanded == Some(index) {
                            None
                        } else {
                            Some(index)
                        };
                    }
                    if let Some(index) = remove {
                        let removed = state.obstacles.obstacles.remove(index);
                        state.log(format!("Obstacle removed: {}", removed.name));
                        self.expanded = match self.expanded {
                            Some(current) if current == index => None,
                            Some(current) if current > index => Some(current - 1),
                            other => other,
                        };
                    }
                });
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("add_obstacle_type")
                        .selected_text(&self.add_type)
                        .show_ui(ui, |ui| {
                            for type_name in TYPE_NAMES {
                                ui.selectable_value(
                                    &mut self.add_type,
                                    type_name.to_string(),
                                    type_name,
                                );
                            }
                        });
                    if ui.button("Add").clicked() {
                        let number = state.obstacles.obstacles.len() + 1;
                        state.obstacles.obstacles.push(Obstacle {
                            id: format!("obs_{}_{number}", self.add_type),
                            obstacle_type: self.add_type.clone(),
                            position: [0.0, 0.5, 0.0],
                            rotation: [0.0, 0.0, 0.0],
                            scale: [1.0, 1.0, 1.0],
                            name: format!("{} {number}", self.add_type),
                            color: None,
                        });
                    }
                });
            });
    }
}

fn import_dialog(state: &mut AppState) {
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

fn export_dialog(state: &mut AppState) {
    let path = rfd::FileDialog::new()
        .add_filter("Obstacle file", &["json"])
        .set_file_name("obstacles.json")
        .save_file();
    let Some(path) = path else { return };
    let file = ObstacleFile {
        obstacles: state.obstacles.obstacles.clone(),
        boundary: Some(state.obstacles.boundary),
    };
    let text_path = path.to_string_lossy().into_owned();
    match save_obstacles(&text_path, &file) {
        Ok(()) => state.log("Obstacles exported"),
        Err(err) => state.log_level("error", format!("Error exporting obstacles: {err}")),
    }
}
