use crate::state::AppState;
use crate::ui::telemetry::TelemetryPanel;
use egui::{Color32, RichText, Sense, Ui, Vec2};
use planner_core::commands::{command_defs, interp, Command, CommandType, ParamKind, ParamValue};
use std::collections::{BTreeMap, BTreeSet};

pub const LEFT_PANEL_WIDTH: f32 = 272.0;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Category {
    Flight,
    Control,
    Output,
    Var,
    Sensor,
    Func,
    Timer,
}

pub fn category_color(category: Category) -> Color32 {
    match category {
        Category::Flight => Color32::from_rgb(0x58, 0xa6, 0xff),
        Category::Output => Color32::from_rgb(0xd2, 0x99, 0x22),
        Category::Control => Color32::from_rgb(0xbc, 0x8c, 0xff),
        Category::Var => Color32::from_rgb(0x39, 0xd2, 0xc0),
        Category::Sensor => Color32::from_rgb(0x3f, 0xb9, 0x50),
        Category::Func => Color32::from_rgb(0xf0, 0x88, 0x3e),
        Category::Timer => Color32::from_rgb(0xf7, 0x78, 0xba),
    }
}

pub fn type_category(command_type: CommandType) -> Option<Category> {
    match command_type {
        CommandType::Takeoff
        | CommandType::Land
        | CommandType::EmergencyStop
        | CommandType::StopMotors
        | CommandType::Hover
        | CommandType::Flip
        | CommandType::Go
        | CommandType::MoveForward
        | CommandType::MoveBackward
        | CommandType::MoveLeft
        | CommandType::MoveRight
        | CommandType::TurnLeft
        | CommandType::TurnRight
        | CommandType::TurnDegree
        | CommandType::Circle
        | CommandType::CircleTurn
        | CommandType::Square
        | CommandType::SquareTurn
        | CommandType::Triangle
        | CommandType::TriangleTurn
        | CommandType::Spiral
        | CommandType::Sway
        | CommandType::KeepDistance
        | CommandType::AvoidWall => Some(Category::Flight),
        CommandType::Led
        | CommandType::LedOff
        | CommandType::Buzzer
        | CommandType::RandomLed
        | CommandType::TimeSleep
        | CommandType::DroneSleep => Some(Category::Output),
        CommandType::IfBlock
        | CommandType::ElifBlock
        | CommandType::ElseBlock
        | CommandType::EndBlock
        | CommandType::WhileBlock
        | CommandType::ForBlock
        | CommandType::BreakCmd => Some(Category::Control),
        CommandType::VarDeclare
        | CommandType::SetVar
        | CommandType::PrintVar
        | CommandType::UserInput => Some(Category::Var),
        CommandType::GetDistance
        | CommandType::GetHeight
        | CommandType::GetBattery
        | CommandType::GetTemperature
        | CommandType::GetFrontRange
        | CommandType::GetBottomRange
        | CommandType::GetFrontColor
        | CommandType::GetBackColor
        | CommandType::DetectWall => Some(Category::Sensor),
        CommandType::FuncDef
        | CommandType::FuncCall
        | CommandType::ReturnVal
        | CommandType::ListDeclare
        | CommandType::ListAppend
        | CommandType::ListGet => Some(Category::Func),
        CommandType::TimerStart | CommandType::TimerElapsed => Some(Category::Timer),
    }
}

type PaletteEntry = (CommandType, &'static str);

const FLIGHT: &[PaletteEntry] = &[
    (CommandType::Takeoff, "Takeoff"),
    (CommandType::Land, "Land"),
    (CommandType::EmergencyStop, "Emergency Stop"),
    (CommandType::StopMotors, "Stop Motors"),
    (CommandType::Hover, "Hover"),
    (CommandType::Flip, "Flip"),
    (CommandType::Go, "Go"),
    (CommandType::MoveForward, "Forward"),
    (CommandType::MoveBackward, "Back"),
    (CommandType::MoveLeft, "Left"),
    (CommandType::MoveRight, "Right"),
    (CommandType::TurnLeft, "Turn L"),
    (CommandType::TurnRight, "Turn R"),
    (CommandType::TurnDegree, "Turn"),
    (CommandType::Circle, "Circle"),
    (CommandType::CircleTurn, "Circle Turn"),
    (CommandType::Square, "Square"),
    (CommandType::SquareTurn, "Square Turn"),
    (CommandType::Triangle, "Triangle"),
    (CommandType::TriangleTurn, "Triangle Turn"),
    (CommandType::Spiral, "Spiral"),
    (CommandType::Sway, "Sway"),
    (CommandType::KeepDistance, "Keep Distance"),
    (CommandType::AvoidWall, "Avoid Wall"),
];

const CONTROL: &[PaletteEntry] = &[
    (CommandType::IfBlock, "If"),
    (CommandType::ElifBlock, "Elif"),
    (CommandType::ElseBlock, "Else"),
    (CommandType::EndBlock, "End"),
    (CommandType::WhileBlock, "While"),
    (CommandType::ForBlock, "For Loop"),
    (CommandType::BreakCmd, "Break"),
];

const OUTPUT: &[PaletteEntry] = &[
    (CommandType::Led, "LED"),
    (CommandType::LedOff, "LED Off"),
    (CommandType::RandomLed, "Random LED"),
    (CommandType::Buzzer, "Buzzer"),
    (CommandType::TimeSleep, "Sleep"),
    (CommandType::DroneSleep, "Drone Sleep"),
];

const VARIABLES: &[PaletteEntry] = &[
    (CommandType::VarDeclare, "Var"),
    (CommandType::SetVar, "Set"),
    (CommandType::PrintVar, "Print"),
    (CommandType::UserInput, "Input"),
];

const SENSORS: &[PaletteEntry] = &[
    (CommandType::GetBattery, "Battery"),
    (CommandType::GetHeight, "Height"),
    (CommandType::GetDistance, "Distance"),
    (CommandType::GetFrontRange, "Front Range"),
    (CommandType::GetBottomRange, "Bottom Range"),
    (CommandType::GetFrontColor, "Front Color"),
    (CommandType::GetBackColor, "Back Color"),
    (CommandType::GetTemperature, "Temp"),
    (CommandType::DetectWall, "Detect Wall"),
];

const FUNCS: &[PaletteEntry] = &[
    (CommandType::FuncDef, "Define"),
    (CommandType::FuncCall, "Call"),
    (CommandType::ReturnVal, "Return"),
    (CommandType::ListDeclare, "New List"),
    (CommandType::ListAppend, "Append"),
    (CommandType::ListGet, "Get Index"),
];

const TIMER: &[PaletteEntry] = &[
    (CommandType::TimerStart, "Start"),
    (CommandType::TimerElapsed, "Elapsed"),
];

const GROUPS: &[(&str, Category, &[PaletteEntry])] = &[
    ("Flight", Category::Flight, FLIGHT),
    ("Control Flow", Category::Control, CONTROL),
    ("Output", Category::Output, OUTPUT),
    ("Variables", Category::Var, VARIABLES),
    ("Sensors", Category::Sensor, SENSORS),
    ("Functions & Lists", Category::Func, FUNCS),
    ("Timer", Category::Timer, TIMER),
];

pub struct PlanTreePanel {
    renaming: Option<(String, String)>,
    palette_filter: String,
    telemetry: TelemetryPanel,
    last_selection: Vec<usize>,
}

impl Default for PlanTreePanel {
    fn default() -> Self {
        PlanTreePanel {
            renaming: None,
            palette_filter: String::new(),
            telemetry: TelemetryPanel::new(),
            last_selection: Vec::new(),
        }
    }
}

impl PlanTreePanel {
    pub fn new() -> PlanTreePanel {
        PlanTreePanel::default()
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        if !state.show_left {
            return;
        }
        egui::Panel::left("left_panel")
            .exact_size(LEFT_PANEL_WIDTH)
            .resizable(false)
            .show(ui, |ui| {
                let telemetry_height = if self.telemetry.chart_expanded() {
                    330.0
                } else {
                    240.0
                };
                let top_height = (ui.available_height() - telemetry_height - 12.0).max(60.0);
                egui::ScrollArea::vertical()
                    .id_salt("plan_tree_commands_scroll")
                    .max_height(top_height)
                    .show(ui, |ui| {
                        ui.heading("Flight Commands");
                        ui.separator();
                        self.drone_tabs(ui, state);
                        self.drone_actions(ui, state);
                        ui.separator();
                        self.command_rows(ui, state);
                        ui.add_space(4.0);
                        ui.separator();
                        self.palette_section(ui, state);
                    });
                ui.separator();
                egui::ScrollArea::vertical()
                    .id_salt("plan_tree_telemetry_scroll")
                    .max_height(telemetry_height)
                    .show(ui, |ui| {
                        self.telemetry.show(ui, state);
                    });
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
    let mut parts: Vec<String> = Vec::new();
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    if let Some(def) = command_defs()
        .iter()
        .find(|def| def.command_type == command.command_type)
    {
        for param in def.params {
            if let Some(value) = command.params.get(param.key) {
                parts.push(format!("{}={}", param.key, interp(value)));
                seen.insert(param.key);
            }
        }
    }
    for (key, value) in &command.params {
        if !seen.contains(key.as_str()) {
            parts.push(format!("{key}={}", interp(value)));
        }
    }
    parts.join(", ")
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
                            state.refresh_sim();
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
                state.mark_dirty();
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

    fn command_rows(&mut self, ui: &mut Ui, state: &mut AppState) {
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
        let scroll = if selection != self.last_selection {
            self.last_selection = selection.clone();
            !selection.is_empty()
        } else {
            false
        };
        render_commands(ui, state, &commands, &selection, &[], 0, scroll);
    }

    fn palette_section(&mut self, ui: &mut Ui, state: &mut AppState) {
        ui.label(RichText::new("Add Command").strong());
        ui.add(
            egui::TextEdit::singleline(&mut self.palette_filter)
                .hint_text("Search commands")
                .desired_width(f32::INFINITY),
        );
        ui.add_space(4.0);
        let needle = self.palette_filter.trim().to_lowercase();
        for (name, category, entries) in GROUPS {
            let visible: Vec<&PaletteEntry> = entries
                .iter()
                .filter(|(command_type, label)| {
                    needle.is_empty()
                        || label.to_lowercase().contains(&needle)
                        || command_type.as_str().contains(&needle)
                        || command_type
                            .as_str()
                            .replace('_', " ")
                            .contains(&needle)
                })
                .collect();
            if visible.is_empty() {
                continue;
            }
            ui.label(RichText::new(*name).strong());
            let tint = category_color(*category);
            for (command_type, label) in &visible {
                let button = egui::Button::new(RichText::new(*label).color(tint))
                    .fill(tint.gamma_multiply(0.18))
                    .min_size(egui::vec2(f32::INFINITY, 22.0));
                if ui.add(button).clicked() {
                    state.add_command(*command_type);
                }
            }
            ui.add_space(6.0);
        }
        self.command_options(ui, state);
    }

    fn command_options(&mut self, ui: &mut Ui, state: &mut AppState) {
        let Some(command) = selected_command(state) else {
            return;
        };
        ui.separator();
        ui.label(RichText::new("Command Options").strong());
        ui.push_id("palette_options", |ui| {
            draw_param_editors(ui, state, command.command_type, &command.params);
        });
    }
}

fn selected_command(state: &AppState) -> Option<Command> {
    let drone = state
        .plan
        .drones
        .iter()
        .find(|drone| Some(&drone.id) == state.plan.active_drone_id.as_ref())
        .or_else(|| state.plan.drones.first())?;
    let path = &state.selection.command_path;
    if path.is_empty() {
        return None;
    }
    let mut commands: &[Command] = &drone.commands;
    for (depth, index) in path.iter().enumerate() {
        let command = commands.get(*index)?;
        if depth == path.len() - 1 {
            return Some(command.clone());
        }
        commands = &command.children;
    }
    None
}

fn render_commands(
    ui: &mut Ui,
    state: &mut AppState,
    commands: &[Command],
    selection: &[usize],
    parent_path: &[usize],
    depth: usize,
    scroll: bool,
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
        draw_row(
            ui, state, command, label, &number, &path, is_selected, index, commands.len(), scroll,
        );
        if is_selected {
            ui.push_id(format!("editor_{path:?}"), |ui| {
                draw_param_editors(ui, state, command.command_type, &command.params);
            });
        }
        if command.command_type.is_block() {
            ui.indent(format!("children_{path:?}"), |ui| {
                render_commands(ui, state, &command.children, selection, &path, depth + 1, scroll);
                if ui.button("+ Add").clicked() {
                    state.selection.command_path = path.clone();
                    state.log(format!("Palette picks now add inside {label}"));
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
    scroll: bool,
) {
    let mut frame = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(4, 2));
    if is_selected {
        frame = frame.fill(Color32::from_rgba_unmultiplied(0x58, 0xa6, 0xff, 38));
    }
    let response = frame.show(ui, |ui| {
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
                if ui.add(egui::Button::new("Dup").small()).clicked() {
                    state.selection.command_path = path.to_vec();
                    duplicate_selected_command(state);
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
    if is_selected && scroll {
        response.response.scroll_to_me(None);
    }
}

fn collect_command_ids(commands: &[Command], out: &mut BTreeSet<String>) {
    for command in commands {
        out.insert(command.id.clone());
        collect_command_ids(&command.children, out);
    }
}

fn reassign_ids(command: &mut Command, used: &BTreeSet<String>, seq: &mut u64) {
    loop {
        let candidate = format!("cp{}", *seq);
        *seq += 1;
        if !used.contains(&candidate) {
            command.id = candidate;
            break;
        }
    }
    for child in &mut command.children {
        reassign_ids(child, used, seq);
    }
}

fn command_at_path_mut<'a>(commands: &'a mut [Command], path: &[usize]) -> Option<&'a mut Command> {
    if path.is_empty() {
        return None;
    }
    let mut current = commands;
    for &idx in &path[..path.len() - 1] {
        current = current.get_mut(idx)?.children.as_mut_slice();
    }
    current.get_mut(path[path.len() - 1])
}

fn duplicate_selected_command(state: &mut AppState) {
    let path = state.selection.command_path.clone();
    if path.is_empty() {
        return;
    }
    let Some(index) = state.active_drone_index() else {
        return;
    };
    let mut used = BTreeSet::new();
    for drone in &state.plan.drones {
        collect_command_ids(&drone.commands, &mut used);
    }
    let last = path[path.len() - 1];
    let parent_path = &path[..path.len() - 1];
    let drone = &mut state.plan.drones[index];
    let list: &mut Vec<Command> = if parent_path.is_empty() {
        &mut drone.commands
    } else {
        match command_at_path_mut(&mut drone.commands, parent_path) {
            Some(parent) => &mut parent.children,
            None => return,
        }
    };
    let Some(source) = list.get(last) else {
        return;
    };
    let mut copy = source.clone();
    let mut seq = 1u64;
    reassign_ids(&mut copy, &used, &mut seq);
    let label = command_defs()
        .iter()
        .find(|def| def.command_type == copy.command_type)
        .map(|def| def.label)
        .unwrap_or(copy.command_type.as_str());
    list.insert(last + 1, copy);
    state.selection.command_path = [parent_path, &[last + 1]].concat();
    state.log(format!("Duplicated: {label}"));
    state.mark_dirty();
    state.refresh_sim();
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
                        state.mark_dirty();
                        state.refresh_sim();
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
                        state.mark_dirty();
                        state.refresh_sim();
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
                                    state.mark_dirty();
                                    state.refresh_sim();
                                }
                            }
                        });
                }
            }
        });
    }
}
