use crate::state::AppState;
use egui::Ui;
use planner_core::obstacles::ObstacleSet;
use planner_core::sensors::{evaluate, is_omnidirectional, ColorTable, SensorKind, SensorSet};

#[derive(Default, Clone)]
struct PanelState {
    sensor_set: SensorSet,
    selected_id: Option<String>,
}

pub fn draw(ui: &mut Ui, state: &mut AppState) {
    let id = egui::Id::new("sensor_panel_selection");
    let selected_id = ui
        .data_mut(|data| data.get_temp::<Option<String>>(id))
        .unwrap_or_default();
    let mut panel_state = PanelState {
        sensor_set: std::mem::take(&mut state.sensor_set),
        selected_id,
    };

    ui.heading("Sensors");
    ui.separator();
    
    for sensor in &panel_state.sensor_set.sensors {
        let is_selected = panel_state.selected_id.as_deref() == Some(&sensor.id);
        let label = format!("{} ({:?})", sensor.name, sensor.kind);
        if ui
            .add_sized(
                egui::vec2(f32::INFINITY, 0.0),
                egui::Button::selectable(is_selected, label),
            )
            .clicked()
        {
            panel_state.selected_id = Some(sensor.id.clone());
        }
    }

    ui.separator();
    ui.label("Add Sensor:");
    let kinds = [
        SensorKind::FrontRange,
        SensorKind::BottomRange,
        SensorKind::FrontColor,
        SensorKind::BackColor,
        SensorKind::Temperature,
        SensorKind::Battery,
    ];
    for kind in kinds {
        let label = match kind {
            SensorKind::FrontRange => "FrontRange",
            SensorKind::BottomRange => "BottomRange",
            SensorKind::FrontColor => "FrontColor",
            SensorKind::BackColor => "BackColor",
            SensorKind::Temperature => "Temperature",
            SensorKind::Battery => "Battery",
        };
        if ui.add(egui::Button::new(label).min_size(egui::vec2(f32::INFINITY, 0.0))).clicked() {
            let s = panel_state.sensor_set.add_default(kind);
            panel_state.selected_id = Some(s.id.clone());
        }
    }

    ui.separator();
    if let Some(selected_id) = panel_state.selected_id.clone() {
        if let Some(sensor) = panel_state.sensor_set.sensors.iter_mut().find(|s| s.id == selected_id) {
            ui.horizontal(|ui| {
                ui.checkbox(&mut sensor.enabled, "Enabled");
            });
            ui.horizontal(|ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut sensor.name);
            });
            ui.horizontal(|ui| {
                ui.label("Kind");
                ui.label(format!("{:?}", sensor.kind));
            });

            if is_omnidirectional(sensor.kind) {
                ui.label("Position/facing/range do not apply");
            } else {
                ui.horizontal(|ui| {
                    ui.label("Pos");
                    ui.add(egui::DragValue::new(&mut sensor.mount_position[0]).speed(0.01));
                    ui.add(egui::DragValue::new(&mut sensor.mount_position[1]).speed(0.01));
                    ui.add(egui::DragValue::new(&mut sensor.mount_position[2]).speed(0.01));
                });
                ui.horizontal(|ui| {
                    ui.label("Facing");
                    ui.add(egui::DragValue::new(&mut sensor.facing[0]).speed(0.01));
                    ui.add(egui::DragValue::new(&mut sensor.facing[1]).speed(0.01));
                    ui.add(egui::DragValue::new(&mut sensor.facing[2]).speed(0.01));
                });
                ui.horizontal(|ui| {
                    ui.label("Range (m)");
                    ui.add(egui::DragValue::new(&mut sensor.range_m).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("FOV (deg)");
                    ui.add(egui::DragValue::new(&mut sensor.fov_deg).speed(1.0));
                });
            }

            ui.separator();
            if ui.add(egui::Button::new("Remove").min_size(egui::vec2(f32::INFINITY, 0.0))).clicked() {
                panel_state.sensor_set.remove(&selected_id);
                panel_state.selected_id = None;
            }
        } else {
            panel_state.selected_id = None;
        }
    }

    ui.separator();
    ui.heading("Live Readout");
    let frame_point = state.current_frame().and_then(|f| {
        let active_id = state.active_drone_index().map(|i| state.plan.drones[i].id.clone());
        active_id
            .as_ref()
            .and_then(|id| f.positions.get(id).cloned())
            .or_else(|| f.positions.values().next().cloned())
    });

    if let Some(point) = frame_point {
        let obs_set = ObstacleSet {
            obstacles: state.obstacle_store().clone(),
            boundary: state.obstacles.boundary,
        };
        let surfaces = ColorTable::new();
        for sensor in &panel_state.sensor_set.sensors {
            if !sensor.enabled {
                continue;
            }
            let reading = evaluate(
                sensor,
                [point.x, point.z, point.y],
                point.heading,
                &obs_set,
                &surfaces,
            );
            if matches!(
                sensor.kind,
                SensorKind::FrontRange | SensorKind::BottomRange
            ) {
                let status = if reading.hit {
                    format!("Hit: {}", reading.value)
                } else {
                    "Escapement".to_string()
                };
                ui.label(format!("{}: {}", sensor.name, status));
            } else {
                ui.label(format!("{}: {}", sensor.name, reading.value));
            }
        }
    } else {
        ui.label("No frame");
    }

    state.sensor_set = panel_state.sensor_set;
    ui.data_mut(|data| data.insert_temp(id, panel_state.selected_id));
}
