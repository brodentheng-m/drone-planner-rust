use crate::state::AppState;
use crate::ui::plan_tree::draw_param_editors;
use egui::{Color32, RichText, Ui};
use planner_core::commands::{command_defs, Command, CommandType};

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
        | CommandType::Hover
        | CommandType::Flip
        | CommandType::Go
        | CommandType::MoveForward
        | CommandType::MoveBackward
        | CommandType::MoveLeft
        | CommandType::MoveRight
        | CommandType::TurnLeft
        | CommandType::TurnRight
        | CommandType::Circle
        | CommandType::Square
        | CommandType::Triangle => Some(Category::Flight),
        CommandType::Led | CommandType::Buzzer | CommandType::RandomLed | CommandType::TimeSleep => {
            Some(Category::Output)
        }
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
        | CommandType::GetTemperature => Some(Category::Sensor),
        CommandType::FuncDef
        | CommandType::FuncCall
        | CommandType::ReturnVal
        | CommandType::ListDeclare
        | CommandType::ListAppend
        | CommandType::ListGet => Some(Category::Func),
        CommandType::TimerStart | CommandType::TimerElapsed => Some(Category::Timer),
        _ => None,
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

pub struct PalettePanel;

impl Default for PalettePanel {
    fn default() -> Self {
        PalettePanel
    }
}

impl PalettePanel {
    pub fn new() -> PalettePanel {
        PalettePanel
    }

    pub fn show(&mut self, ui: &mut Ui, state: &mut AppState) {
        egui::Panel::left("palette_panel")
            .default_size(220.0)
            .resizable(true)
            .show(ui, |ui| {
                ui.heading("Add Command");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for (name, category, entries) in GROUPS {
                        ui.label(RichText::new(*name).strong());
                        let tint = category_color(*category);
                        egui::Grid::new(format!("palette_grid_{name}"))
                            .num_columns(2)
                            .spacing(egui::vec2(6.0, 4.0))
                            .show(ui, |ui| {
                                for (index, (command_type, label)) in entries.iter().enumerate() {
                                    let button = egui::Button::new(RichText::new(*label).color(tint))
                                        .fill(tint.gamma_multiply(0.18))
                                        .min_size(egui::vec2(96.0, 22.0));
                                    if ui.add(button).clicked() {
                                        state.add_command(*command_type);
                                        let def_label = command_defs()
                                            .iter()
                                            .find(|def| def.command_type == *command_type)
                                            .map(|def| def.label)
                                            .unwrap_or(command_type.as_str());
                                        state.log(format!("Added: {def_label}"));
                                    }
                                    if index % 2 == 1 {
                                        ui.end_row();
                                    }
                                }
                            });
                        ui.add_space(6.0);
                    }
                    self.command_options(ui, state);
                });
            });
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
