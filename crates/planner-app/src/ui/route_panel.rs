use crate::state::AppState;
use egui::{Color32, RichText, Ui};
use planner_core::obstacles::ObstacleSet;
use planner_core::route::{self, Optimize, RouteFailure, RouteOptions, RouteResult};

#[derive(Clone)]
struct RoutePanelState {
    waypoints: Vec<[f64; 3]>,
    start_pos: [f64; 3],
    start_heading: f64,
    clearance_m: f64,
    cruise_speed: f64,
    takeoff_height_m: f64,
    optimize: Optimize,
    selected_waypoint: Option<usize>,
    result: Option<RouteResult>,
}

impl Default for RoutePanelState {
    fn default() -> Self {
        RoutePanelState {
            waypoints: Vec::new(),
            start_pos: [0.0, 0.8, 0.0],
            start_heading: 0.0,
            clearance_m: 0.05,
            cruise_speed: 50.0,
            takeoff_height_m: 0.8,
            optimize: Optimize::WidestClearance,
            selected_waypoint: None,
            result: None,
        }
    }
}

pub fn draw(ui: &mut Ui, state: &mut AppState) {
    let id = ui.id().with("route_panel_state");
    let mut local_state = ui
        .ctx()
        .data_mut(|d| d.get_temp::<RoutePanelState>(id).unwrap_or_default());
    local_state.waypoints = std::mem::take(&mut state.route_waypoints);
    local_state.start_pos = state.route_start_pos;
    local_state.start_heading = state.route_start_heading;

    ui.heading("Waypoints");
    let mut remove: Option<usize> = None;
    for (i, wp) in local_state.waypoints.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("{}:", i));
            for axis in 0..3 {
                ui.add(egui::DragValue::new(&mut wp[axis]).speed(0.05));
            }
            if ui
                .selectable_label(local_state.selected_waypoint == Some(i), "Candidate")
                .clicked()
            {
                local_state.selected_waypoint = Some(i);
            }
            if ui.button("X").clicked() {
                remove = Some(i);
            }
        });
    }

    if let Some(i) = remove {
        local_state.waypoints.remove(i);
        if local_state.selected_waypoint == Some(i) {
            local_state.selected_waypoint = None;
        } else if let Some(sel) = local_state.selected_waypoint {
            if sel > i {
                local_state.selected_waypoint = Some(sel - 1);
            }
        }
    }

    if ui
        .add_sized([ui.available_width(), 0.0], egui::Button::new("Add waypoint"))
        .clicked()
    {
        let next_wp = if let Some(last) = local_state.waypoints.last() {
            [last[0] + 0.5, last[1], last[2]]
        } else {
            [0.5, 0.8, 0.0]
        };
        local_state.waypoints.push(next_wp);
    }

    ui.separator();

    ui.heading("Start Pose");
    ui.horizontal(|ui| {
        ui.label(format!(
            "pos: {:.2}, {:.2}, {:.2}",
            local_state.start_pos[0], local_state.start_pos[1], local_state.start_pos[2]
        ));
        ui.label(format!("hdg: {:.1}", local_state.start_heading));
    });

    if ui
        .add_sized(
            [ui.available_width(), 0.0],
            egui::Button::new("Set start from selected waypoint"),
        )
        .clicked()
    {
        if let Some(idx) = local_state.selected_waypoint {
            if let Some(wp) = local_state.waypoints.get(idx) {
                local_state.start_pos = *wp;
            }
        }
    }

    ui.horizontal(|ui| {
        ui.label("Heading");
        ui.add(egui::DragValue::new(&mut local_state.start_heading).speed(1.0));
    });

    ui.separator();

    ui.heading("Options");
    if ui
        .add_sized(
            [ui.available_width(), 0.0],
            egui::Button::selectable(local_state.optimize == Optimize::Shortest, "Shortest"),
        )
        .clicked()
    {
        local_state.optimize = Optimize::Shortest;
    }
    if ui
        .add_sized(
            [ui.available_width(), 0.0],
            egui::Button::selectable(
                local_state.optimize == Optimize::WidestClearance,
                "WidestClearance",
            ),
        )
        .clicked()
    {
        local_state.optimize = Optimize::WidestClearance;
    }
    if ui
        .add_sized(
            [ui.available_width(), 0.0],
            egui::Button::selectable(local_state.optimize == Optimize::Balanced, "Balanced"),
        )
        .clicked()
    {
        local_state.optimize = Optimize::Balanced;
    }

    ui.horizontal(|ui| {
        ui.label("Clearance");
        ui.add(egui::DragValue::new(&mut local_state.clearance_m).speed(0.01));
    });
    ui.horizontal(|ui| {
        ui.label("Cruise Speed");
        ui.add(egui::DragValue::new(&mut local_state.cruise_speed).speed(1.0));
    });
    ui.horizontal(|ui| {
        ui.label("Takeoff Height");
        ui.add(egui::DragValue::new(&mut local_state.takeoff_height_m).speed(0.1));
    });

    ui.separator();

    if ui
        .add_sized(
            [ui.available_width(), 0.0],
            egui::Button::new("Route A to B to C"),
        )
        .clicked()
    {
        let opts = RouteOptions {
            clearance_m: local_state.clearance_m,
            optimize: local_state.optimize,
            cruise_speed: local_state.cruise_speed,
            takeoff_height_m: local_state.takeoff_height_m,
        };

        let obstacle_set = ObstacleSet {
            obstacles: state.obstacle_store().clone(),
            boundary: state.obstacles.boundary,
        };

        let res = route::route_through(
            &local_state.waypoints,
            local_state.start_pos,
            local_state.start_heading,
            &obstacle_set,
            &opts,
        );

        if res.feasible {
            let total_dist: f64 = res.legs.iter().map(|l| l.distance_m).sum();
            let mode_str = match local_state.optimize {
                Optimize::Shortest => "Shortest",
                Optimize::WidestClearance => "WidestClearance",
                Optimize::Balanced => "Balanced",
            };
            state.log(format!(
                "Route success: {} waypoints, {:.2} m, mode: {}",
                local_state.waypoints.len(),
                total_dist,
                mode_str
            ));
            if let Some(idx) = state.active_drone_index() {
                state.plan.drones[idx].commands.extend(res.commands.clone());
                state.mark_dirty();
                state.refresh_sim();
            }
        } else {
            let reason = match &res.failure {
                Some(RouteFailure::StartInsideObstacle) => "Start inside obstacle".to_string(),
                Some(RouteFailure::WaypointInsideObstacle(i)) => {
                    format!("Waypoint {} inside obstacle", i)
                }
                Some(RouteFailure::NoPath(i)) => format!("No path to waypoint {}", i),
                Some(RouteFailure::ClearanceUnmet(i)) => {
                    format!("Clearance unmet for waypoint {}", i)
                }
                Some(RouteFailure::TurnBudgetExceeded(i)) => {
                    format!("Turn budget exceeded for waypoint {}", i)
                }
                Some(RouteFailure::EmptyWaypoints) => "Empty waypoints".to_string(),
                None => "Unknown failure".to_string(),
            };
            state.log_level("error", format!("Route failed: {}", reason));
        }
        local_state.result = Some(res);
    }

    ui.separator();

    if let Some(res) = &local_state.result {
        ui.heading("Diagnostics");
        let color = if res.feasible {
            Color32::from_rgb(63, 185, 80)
        } else {
            Color32::from_rgb(248, 81, 73)
        };
        ui.label(
            RichText::new(format!(
                "Feasible: {}",
                if res.feasible { "Yes" } else { "No" }
            ))
            .color(color),
        );
        ui.label(format!("Waypoints Reached: {}", res.waypoints_reached));

        if !res.feasible {
            let reason = match &res.failure {
                Some(RouteFailure::StartInsideObstacle) => "Start inside obstacle".to_string(),
                Some(RouteFailure::WaypointInsideObstacle(i)) => {
                    format!("Waypoint {} inside obstacle", i)
                }
                Some(RouteFailure::NoPath(i)) => format!("No path to waypoint {}", i),
                Some(RouteFailure::ClearanceUnmet(i)) => {
                    format!("Clearance unmet for waypoint {}", i)
                }
                Some(RouteFailure::TurnBudgetExceeded(i)) => {
                    format!("Turn budget exceeded for waypoint {}", i)
                }
                Some(RouteFailure::EmptyWaypoints) => "Empty waypoints".to_string(),
                None => "Unknown failure".to_string(),
            };
            ui.label(RichText::new(reason).color(Color32::from_rgb(248, 81, 73)));
        }

        for leg in &res.legs {
            ui.label(format!("Leg {}: {:.2} m", leg.leg_index, leg.distance_m));
        }
    }

    state.route_waypoints = local_state.waypoints.clone();
    state.route_start_pos = local_state.start_pos;
    state.route_start_heading = local_state.start_heading;
    state.route_result = local_state.result.clone();

    ui.ctx().data_mut(|d| d.insert_temp(id, local_state));
}
