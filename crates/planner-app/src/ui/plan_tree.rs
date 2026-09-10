use crate::state::AppState;
use crate::ui::palette::{category_color, type_category};
use egui::{Color32, RichText, Sense, Ui, Vec2};
use planner_core::commands::{command_defs, interp, Command, CommandType, ParamKind, ParamValue};
use std::collections::BTreeMap;

pub struct PlanTreePanel {
    renaming: Option<(String, String)>,
}

impl Default for PlanTreePanel {
    fn default() -> Self {
        PlanTreePanel { renaming: None }
    }
}

impl PlanTreePanel {
    pub fn new() -> PlanTreePanel {
        PlanTreePanel { renaming: None }
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::left("plan_tree_panel")
            .default_size(280.0)
            .resizable(true)
            .show(ui, |ui| {
                ui.heading("Flight Commands");
                ui.separator();
                self.drone_tabs(ui, state);
                self.drone_actions(ui, state);
                ui.separator();
                self.command_list(ui, state);
            });
    }
}

fn hex_color(color: &str) -> Color32 {
    let hex = color.trim_start_matches('#');
    if hex.len() != 6 {
        return Color32::from_rgb(0x88, 0x88, 0x88);
    }
    let red = u8::from_str_radix(&hex[0..2], 16).unwrap_or(0x88);
    let green = u8::from_str_radix(&hex[2..4], 16).unwrap_or(0x88);
    let blue = u8::from_str_radix(&hex[4..6], 16).unwrap_or(0x88);
    Color32::from_rgb(red, green, blue)
}

fn capitalize(text: &str) -> String {
    let mut chars = text.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
        None => String::new(),
    }
}

fn params_summary(command: &Command) -> String {
    command
        .params
        .iter()
        .map(|(key, value)| format!("{key}={}", interp(value)))
        .collect::<Vec<String>>()
        .join(", ")
}

impl PlanTreePanel {
    fn drone_tabs(&mut self, ui: &mut Ui, state: &mut AppState) {
        let active_id = state
            .plan
            .active_drone_id
            .clone()
            .or_else(|| state.plan.drones.first().map(|drone| drone.id.clone()));
        let drones: Vec<(String, String, String, usize)> = state
            .plan
            .drones
            .iter()
            .map(|drone| {
                (
                    drone.id.clone(),
                    drone.name.clone(),
                    drone.color.clone(),
                    drone.commands.len(),
                )
            })
            .collect();
        ui.horizontal_wrapped(|ui| {
            for (id, name, color, count) in &drones {
                let renaming_this = matches!(&self.renaming, Some((rid, _)) if rid == id);
                if renaming_this {
                    self.rename_editor(ui, state, id);
                    continue;
                }
                let active = active_id.as_ref() == Some(id);
                let mut frame = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(4, 2));
                if active {
                    frame = frame.fill(Color32::from_rgba_unmultiplied(0x58, 0xa6, 0xff, 38));
                }
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        let (rect, _) = ui.allocate_exact_size(Vec2::new(8.0, 8.0), Sense::hover());
                        ui.painter().rect_filled(rect, 2.0, hex_color(color));
                        let tab = ui.add(
                            egui::Label::new(format!("{name} {count}")).sense(Sense::click()),
                        );
                        if tab.clicked() {
                            state.plan.active_drone_id = Some(id.clone());
                            state.selection.command_path.clear();
                        }
                        if tab.double_clicked() {
                            self.renaming = Some((id.clone(), name.clone()));
                        }
                    });
                });
            }
        });
    }

    fn rename_editor(&mut self, ui: &mut Ui, state: &mut AppState, id: &str) {
        let Some((_, buffer)) = &mut self.renaming else {
            return;
        };
        let response = ui.add(egui::TextEdit::singleline(buffer).desired_width(90.0));
        response.request_focus();
        let enter = ui.input(|input| input.key_pressed(egui::Key::Enter));
        let escape = ui.input(|input| input.key_pressed(egui::Key::Escape));
        if !enter && !escape {
            return;
        }
        let name = buffer.trim().to_string();
        if enter && !name.is_empty() {
            if let Some(drone) = state.plan.drones.iter_mut().find(|drone| drone.id == id) {
                drone.name = name;
            }
        }
        self.renaming = None;
    }

    fn drone_actions(&mut self, ui: &mut Ui, state: &mut AppState) {
        ui.horizontal(|ui| {
            if ui.button("+").clicked() {
                state.add_drone();
            }
            if ui.button("Dup").clicked() {
                state.duplicate_active_drone();
            }
            if ui.button("X").clicked() {
                state.remove_active_drone();
            }
            let mut chosen: Option<&'static str> = None;
            egui::ComboBox::from_id_salt("formation_select")
                .selected_text("Formation")
                .show_ui(ui, |ui| {
                    for option in ["line", "grid", "circle", "v", "column", "arc"] {
                        if ui.selectable_label(false, option).clicked() {
                            chosen = Some(option);
                        }
                    }
                });
            if let Some(kind) = chosen {
                state.set_formation(kind);
            }
        });
    }

    fn command_list(&mut self, ui: &mut Ui, state: &mut AppState) {
        let active_index = state
            .plan
            .drones
            .iter()
            .position(|drone| Some(&drone.id) == state.plan.active_drone_id.as_ref())
            .unwrap_or(0);
        let commands = state
            .plan
            .drones
            .get(active_index)
            .map(|drone| drone.commands.clone())
            .unwrap_or_default();
        let selection = state.selection.command_path.clone();
        egui::ScrollArea::vertical().show(ui, |ui| {
            render_commands(ui, state, &commands, &selection, &[], 0);
        });
    }
}

fn render_commands(
    ui: &mut Ui,
    state: &mut AppState,
    commands: &[Command],
    selection: &[usize],
    parent_path: &[usize],
    depth: usize,
) {
    for (index, command) in commands.iter().enumerate() {
        let mut path = parent_path.to_vec();
        path.push(index);
        let is_selected = selection == path.as_slice();
        let label = command_defs()
            .iter()
            .find(|def| def.command_type == command.command_type)
            .map(|def| def.label)
            .unwrap_or(command.command_type.as_str());
        let number = if depth == 0 {
            format!("{}", index + 1)
        } else {
            let parents: Vec<String> = parent_path.iter().map(|item| item.to_string()).collect();
            format!("{}.{}", parents.join("."), index)
        };
        draw_row(ui, state, command, label, &number, &path, is_selected, index, commands.len());
        if is_selected {
            ui.push_id(format!("editor_{path:?}"), |ui| {
                draw_param_editors(ui, state, command.command_type, &command.params);
            });
        }
        if command.command_type.is_block() {
            ui.indent(format!("children_{path:?}"), |ui| {
                render_commands(ui, state, &command.children, selection, &path, depth + 1);
                if ui.button("+ Add").clicked() {
                    state.selection.command_path = path.clone();
                    state.log(format!("Select a command to add inside {label}"));
                }
            });
        }
    }
}

fn draw_row(
    ui: &mut Ui,
    state: &mut AppState,
    command: &Command,
    label: &str,
    number: &str,
    path: &[usize],
    is_selected: bool,
    index: usize,
    count: usize,
) {
    let mut frame = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(4, 2));
    if is_selected {
        frame = frame.fill(Color32::from_rgba_unmultiplied(0x58, 0xa6, 0xff, 38));
    }
    frame.show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            let num = ui.add(
                egui::Label::new(RichText::new(number).monospace().weak()).sense(Sense::click()),
            );
            if num.clicked() {
                state.selection.command_path = path.to_vec();
            }
            if let Some(category) = type_category(command.command_type) {
                let (rect, _) = ui.allocate_exact_size(Vec2::new(8.0, 8.0), Sense::hover());
                ui.painter().rect_filled(rect, 2.0, category_color(category));
            }
            let name = ui.add(egui::Label::new(label).sense(Sense::click()));
            if name.clicked() {
                state.selection.command_path = path.to_vec();
            }
            if !is_selected {
                let summary = params_summary(command);
                if !summary.is_empty() {
                    let sum = ui.add(
                        egui::Label::new(RichText::new(summary).weak().small())
                            .sense(Sense::click()),
                    );
                    if sum.clicked() {
                        state.selection.command_path = path.to_vec();
                    }
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.add(egui::Button::new("X").small()).clicked() {
                    state.selection.command_path = path.to_vec();
                    state.delete_selected();
                }
                if ui
                    .add_enabled(index + 1 < count, egui::Button::new("Dn").small())
                    .clicked()
                {
                    state.selection.command_path = path.to_vec();
                    state.move_command(false);
                }
                if ui.add_enabled(index > 0, egui::Button::new("Up").small()).clicked() {
                    state.selection.command_path = path.to_vec();
                    state.move_command(true);
                }
            });
        });
    });
}

pub(crate) fn draw_param_editors(
    ui: &mut Ui,
    state: &mut AppState,
    command_type: CommandType,
    params: &BTreeMap<String, ParamValue>,
) {
    let Some(def) = command_defs()
        .iter()
        .find(|def| def.command_type == command_type)
    else {
        return;
    };
    for param in def.params {
        ui.horizontal(|ui| {
            ui.label(param.label);
            match param.kind {
                ParamKind::Number => {
                    let mut value = params
                        .get(param.key)
                        .and_then(|value| value.as_f64())
                        .or_else(|| param.default.as_f64())
                        .unwrap_or(0.0);
                    let mut drag =
                        egui::DragValue::new(&mut value).speed(param.step.unwrap_or(1.0));
                    if let (Some(min), Some(max)) = (param.min, param.max) {
                        drag = drag.range(min..=max);
                    }
                    if ui.add(drag).changed() {
                        state.set_param(param.key, ParamValue::Number(value));
                    }
                }
                ParamKind::Text => {
                    let mut value = params
                        .get(param.key)
                        .map(interp)
                        .unwrap_or_else(|| interp(&param.default));
                    let edit = egui::TextEdit::singleline(&mut value).desired_width(140.0);
                    if ui.add(edit).changed() {
                        state.set_param(param.key, ParamValue::Str(value));
                    }
                }
                ParamKind::Select => {
                    let current = params
                        .get(param.key)
                        .map(interp)
                        .unwrap_or_else(|| interp(&param.default));
                    egui::ComboBox::from_id_salt(format!("select_{}", param.key))
                        .selected_text(capitalize(&current))
                        .show_ui(ui, |ui| {
                            for option in param.options {
                                if ui
                                    .selectable_label(current == *option, capitalize(option))
                                    .clicked()
                                {
                                    state.set_param(
                                        param.key,
                                        ParamValue::Str((*option).to_string()),
                                    );
                                }
                            }
                        });
                }
            }
        });
    }
}
