use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::LazyLock;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandType {
    Takeoff,
    Land,
    EmergencyStop,
    StopMotors,
    Hover,
    Flip,
    Go,
    MoveForward,
    MoveBackward,
    MoveLeft,
    MoveRight,
    TurnLeft,
    TurnRight,
    TurnDegree,
    Circle,
    CircleTurn,
    Square,
    SquareTurn,
    Triangle,
    TriangleTurn,
    Spiral,
    Sway,
    KeepDistance,
    AvoidWall,
    DetectWall,
    Led,
    LedOff,
    RandomLed,
    Buzzer,
    TimeSleep,
    DroneSleep,
    VarDeclare,
    SetVar,
    PrintVar,
    IfBlock,
    ElifBlock,
    ElseBlock,
    EndBlock,
    WhileBlock,
    ForBlock,
    BreakCmd,
    GetBattery,
    GetHeight,
    GetFrontRange,
    GetBottomRange,
    GetFrontColor,
    GetBackColor,
    GetTemperature,
    GetDistance,
    FuncDef,
    FuncCall,
    ReturnVal,
    ListDeclare,
    ListAppend,
    ListGet,
    UserInput,
    TimerStart,
    TimerElapsed,
}

impl CommandType {
    pub fn as_str(self) -> &'static str {
        match self {
            CommandType::Takeoff => "takeoff",
            CommandType::Land => "land",
            CommandType::EmergencyStop => "emergency_stop",
            CommandType::StopMotors => "stop_motors",
            CommandType::Hover => "hover",
            CommandType::Flip => "flip",
            CommandType::Go => "go",
            CommandType::MoveForward => "move_forward",
            CommandType::MoveBackward => "move_backward",
            CommandType::MoveLeft => "move_left",
            CommandType::MoveRight => "move_right",
            CommandType::TurnLeft => "turn_left",
            CommandType::TurnRight => "turn_right",
            CommandType::TurnDegree => "turn_degree",
            CommandType::Circle => "circle",
            CommandType::CircleTurn => "circle_turn",
            CommandType::Square => "square",
            CommandType::SquareTurn => "square_turn",
            CommandType::Triangle => "triangle",
            CommandType::TriangleTurn => "triangle_turn",
            CommandType::Spiral => "spiral",
            CommandType::Sway => "sway",
            CommandType::KeepDistance => "keep_distance",
            CommandType::AvoidWall => "avoid_wall",
            CommandType::DetectWall => "detect_wall",
            CommandType::Led => "led",
            CommandType::LedOff => "led_off",
            CommandType::RandomLed => "random_led",
            CommandType::Buzzer => "buzzer",
            CommandType::TimeSleep => "time_sleep",
            CommandType::DroneSleep => "drone_sleep",
            CommandType::VarDeclare => "var_declare",
            CommandType::SetVar => "set_var",
            CommandType::PrintVar => "print_var",
            CommandType::IfBlock => "if_block",
            CommandType::ElifBlock => "elif_block",
            CommandType::ElseBlock => "else_block",
            CommandType::EndBlock => "end_block",
            CommandType::WhileBlock => "while_block",
            CommandType::ForBlock => "for_block",
            CommandType::BreakCmd => "break_cmd",
            CommandType::GetBattery => "get_battery",
            CommandType::GetHeight => "get_height",
            CommandType::GetFrontRange => "get_front_range",
            CommandType::GetBottomRange => "get_bottom_range",
            CommandType::GetFrontColor => "get_front_color",
            CommandType::GetBackColor => "get_back_color",
            CommandType::GetTemperature => "get_temperature",
            CommandType::GetDistance => "get_distance",
            CommandType::FuncDef => "func_def",
            CommandType::FuncCall => "func_call",
            CommandType::ReturnVal => "return_val",
            CommandType::ListDeclare => "list_declare",
            CommandType::ListAppend => "list_append",
            CommandType::ListGet => "list_get",
            CommandType::UserInput => "user_input",
            CommandType::TimerStart => "timer_start",
            CommandType::TimerElapsed => "timer_elapsed",
        }
    }

    pub fn is_block(self) -> bool {
        matches!(
            self,
            CommandType::IfBlock
                | CommandType::ElifBlock
                | CommandType::ElseBlock
                | CommandType::WhileBlock
                | CommandType::ForBlock
                | CommandType::FuncDef
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Number(f64),
    Str(String),
    Bool(bool),
}

impl ParamValue {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            ParamValue::Number(n) => Some(*n),
            ParamValue::Str(s) => js_parse_float(s),
            ParamValue::Bool(_) => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            ParamValue::Str(s) => Some(s),
            _ => None,
        }
    }
}

pub fn js_parse_float(text: &str) -> Option<f64> {
    let rest = text.trim_start();
    let bytes = rest.as_bytes();
    let mut index = 0;
    let mut sign = 1.0;
    if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
        if bytes[index] == b'-' {
            sign = -1.0;
        }
        index += 1;
    }
    if rest[index.min(rest.len())..].starts_with("Infinity") {
        return Some(sign * f64::INFINITY);
    }
    let start = index;
    let mut seen_digit = false;
    while index < bytes.len() && bytes[index].is_ascii_digit() {
        index += 1;
        seen_digit = true;
    }
    if index < bytes.len() && bytes[index] == b'.' {
        index += 1;
        while index < bytes.len() && bytes[index].is_ascii_digit() {
            index += 1;
            seen_digit = true;
        }
    }
    if !seen_digit {
        return None;
    }
    let mut end = index;
    if index < bytes.len() && (bytes[index] == b'e' || bytes[index] == b'E') {
        let mut exp_index = index + 1;
        if exp_index < bytes.len() && (bytes[exp_index] == b'+' || bytes[exp_index] == b'-') {
            exp_index += 1;
        }
        let mut exp_seen = false;
        while exp_index < bytes.len() && bytes[exp_index].is_ascii_digit() {
            exp_index += 1;
            exp_seen = true;
        }
        if exp_seen {
            end = exp_index;
        }
    }
    rest[start..end].parse::<f64>().ok().map(|v| sign * v)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Command {
    pub id: String,
    #[serde(rename = "type")]
    pub command_type: CommandType,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub params: BTreeMap<String, ParamValue>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<Command>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamKind {
    Number,
    Text,
    Select,
}

#[derive(Debug, Clone, Copy)]
pub enum CodeTemplate {
    Fixed(&'static str),
    Function(fn(&BTreeMap<String, ParamValue>) -> String),
}

pub struct ParamDef {
    pub key: &'static str,
    pub label: &'static str,
    pub kind: ParamKind,
    pub default: ParamValue,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub step: Option<f64>,
    pub options: &'static [&'static str],
}

pub struct CommandDef {
    pub label: &'static str,
    pub command_type: CommandType,
    pub params: &'static [ParamDef],
    pub is_block: bool,
    pub code: CodeTemplate,
}

pub fn interp(value: &ParamValue) -> String {
    match value {
        ParamValue::Number(n) => js_num(*n),
        ParamValue::Str(s) => s.clone(),
        ParamValue::Bool(b) => b.to_string(),
    }
}

pub fn interp_param(p: &BTreeMap<String, ParamValue>, key: &str) -> String {
    match p.get(key) {
        Some(value) => interp(value),
        None => "undefined".to_string(),
    }
}

fn param_number(p: &BTreeMap<String, ParamValue>, key: &str) -> f64 {
    p.get(key).and_then(|v| v.as_f64()).unwrap_or(f64::NAN)
}

fn js_round(v: f64) -> f64 {
    (v + 0.5).floor()
}

enum ShapeSecs {
    Off,
    FromParam,
    Fixed(&'static str),
}

fn dir_val(p: &BTreeMap<String, ParamValue>) -> i32 {
    if p.get("dir").and_then(|v| v.as_str()) == Some("clockwise") {
        1
    } else {
        -1
    }
}

fn move_code(method: &str, p: &BTreeMap<String, ParamValue>) -> String {
    let speed = js_round(param_number(p, "speed") / 100.0 * 2.0 * 100.0) / 100.0;
    let speed_str = if speed.is_finite() && speed == speed.trunc() {
        format!("{:.1}", speed)
    } else {
        js_num(speed)
    };
    format!(
        "drone.{}({}, speed={})",
        method,
        interp_param(p, "dist"),
        speed_str
    )
}

fn move_forward_code(p: &BTreeMap<String, ParamValue>) -> String {
    move_code("move_forward", p)
}

fn move_backward_code(p: &BTreeMap<String, ParamValue>) -> String {
    move_code("move_backward", p)
}

fn move_left_code(p: &BTreeMap<String, ParamValue>) -> String {
    move_code("move_left", p)
}

fn move_right_code(p: &BTreeMap<String, ParamValue>) -> String {
    move_code("move_right", p)
}

fn shape_code(method: &str, secs: ShapeSecs, p: &BTreeMap<String, ParamValue>) -> String {
    let mut line = format!("drone.{}(speed={}", method, interp_param(p, "speed"));
    match secs {
        ShapeSecs::Off => {}
        ShapeSecs::FromParam => line.push_str(&format!(", seconds={}", interp_param(p, "secs"))),
        ShapeSecs::Fixed(fixed) => line.push_str(&format!(", seconds={}", fixed)),
    }
    line.push_str(&format!(", direction={})", dir_val(p)));
    line
}

fn circle_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("circle", ShapeSecs::Off, p)
}

fn circle_turn_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("circle_turn", ShapeSecs::FromParam, p)
}

fn square_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("square", ShapeSecs::FromParam, p)
}

fn triangle_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("triangle", ShapeSecs::FromParam, p)
}

fn triangle_turn_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("triangle_turn", ShapeSecs::FromParam, p)
}

fn spiral_code(p: &BTreeMap<String, ParamValue>) -> String {
    shape_code("spiral", ShapeSecs::Fixed("3"), p)
}

fn sway_dir_int(dir: &str) -> i32 {
    match dir {
        "forward-back" | "left-right" | "up-down" | "turn-left" | "pitch-forward" | "roll-left" => 1,
        "turn-right" | "pitch-backward" | "roll-right" => -1,
        _ => 1,
    }
}

fn sway_code(p: &BTreeMap<String, ParamValue>) -> String {
    let dir = p.get("dir").and_then(|v| v.as_str()).unwrap_or("");
    format!(
        "drone.sway(speed={}, seconds={}, direction={})",
        interp_param(p, "speed"),
        interp_param(p, "secs"),
        sway_dir_int(dir)
    )
}

fn keep_distance_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.keep_distance(2, {})", interp_param(p, "dist"))
}

fn avoid_wall_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.avoid_wall(2, {})", interp_param(p, "dist"))
}

fn detect_wall_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{} = drone.detect_wall()", interp_param(p, "var"))
}

fn led_rgb(color: &str) -> (i32, i32, i32) {
    match color {
        "red" => (255, 0, 0),
        "green" => (0, 255, 0),
        "blue" => (0, 0, 255),
        "yellow" => (255, 255, 0),
        "cyan" => (0, 255, 255),
        "magenta" => (255, 0, 255),
        "white" => (255, 255, 255),
        "purple" => (128, 0, 255),
        "orange" => (255, 165, 0),
        "pink" => (255, 192, 203),
        _ => (0, 255, 0),
    }
}

fn led_code(p: &BTreeMap<String, ParamValue>) -> String {
    let color = p.get("color").and_then(|v| v.as_str()).unwrap_or("");
    if color == "off" {
        return "drone.drone_LED_off()".to_string();
    }
    let (r, g, b) = led_rgb(color);
    format!("drone.set_drone_LED({}, {}, {}, 100)", r, g, b)
}

const BUZZER_NOTES: &[(f64, &str)] = &[
    (261.0, "C4"),
    (294.0, "D4"),
    (329.0, "E4"),
    (349.0, "F4"),
    (392.0, "G4"),
    (440.0, "A4"),
    (494.0, "B4"),
    (523.0, "C5"),
    (659.0, "E5"),
    (880.0, "A5"),
];

fn buzzer_code(p: &BTreeMap<String, ParamValue>) -> String {
    let freq = param_number(p, "freq");
    let mut nearest = BUZZER_NOTES[0];
    let mut best = (freq - nearest.0).abs();
    for note in BUZZER_NOTES {
        let d = (freq - note.0).abs();
        if d < best {
            best = d;
            nearest = *note;
        }
    }
    format!(
        "drone.drone_buzzer(\"{}\", {})",
        nearest.1,
        interp_param(p, "dur")
    )
}

fn hover_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.hover({})", interp_param(p, "dur"))
}

fn flip_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.flip(\"{}\")", interp_param(p, "dir"))
}

fn go_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "drone.go(\"{}\", {}, {})",
        interp_param(p, "dir"),
        interp_param(p, "power"),
        interp_param(p, "dur")
    )
}

fn turn_left_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.turn_left({})", interp_param(p, "deg"))
}

fn turn_right_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("drone.turn_right({})", interp_param(p, "deg"))
}

fn turn_degree_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "drone.turn_degree({}, timeout={}, p_value={})",
        interp_param(p, "deg"),
        interp_param(p, "timeout"),
        interp_param(p, "p_value")
    )
}

fn sleep_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("time.sleep({})", interp_param(p, "dur"))
}

fn var_declare_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = {}",
        interp_param(p, "name"),
        interp_param(p, "value")
    )
}

fn set_var_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} {} {}",
        interp_param(p, "name"),
        interp_param(p, "op"),
        interp_param(p, "value")
    )
}

fn print_var_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("print({})", interp_param(p, "value"))
}

fn get_battery_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{} = drone.get_battery()", interp_param(p, "var"))
}

fn get_height_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_height(unit=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "unit")
    )
}

fn get_front_range_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_front_range(unit=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "unit")
    )
}

fn get_bottom_range_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_bottom_range(unit=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "unit")
    )
}

fn get_front_color_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_front_color(kind=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "kind")
    )
}

fn get_back_color_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_back_color(kind=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "kind")
    )
}

fn get_temperature_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = drone.get_temperature(unit=\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "unit")
    )
}

fn get_distance_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{} = drone.get_front_range()", interp_param(p, "var"))
}

fn func_call_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{}()", interp_param(p, "name"))
}

fn return_val_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("return {}", interp_param(p, "value"))
}

fn list_declare_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = [{}]",
        interp_param(p, "name"),
        interp_param(p, "values")
    )
}

fn list_append_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{}.append({})", interp_param(p, "name"), interp_param(p, "value"))
}

fn list_get_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = {}[{}]",
        interp_param(p, "var"),
        interp_param(p, "list_name"),
        interp_param(p, "index")
    )
}

fn user_input_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = input(\"{}\")",
        interp_param(p, "var"),
        interp_param(p, "prompt")
    )
}

fn timer_start_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!("{} = time.time()", interp_param(p, "name"))
}

fn timer_elapsed_code(p: &BTreeMap<String, ParamValue>) -> String {
    format!(
        "{} = time.time() - {}",
        interp_param(p, "var"),
        interp_param(p, "name")
    )
}

fn pd_num(key: &'static str, label: &'static str, default: f64, min: Option<f64>, max: Option<f64>, step: Option<f64>) -> ParamDef {
    ParamDef {
        key,
        label,
        kind: ParamKind::Number,
        default: ParamValue::Number(default),
        min,
        max,
        step,
        options: &[],
    }
}

fn pd_text(key: &'static str, label: &'static str, default: &'static str) -> ParamDef {
    ParamDef {
        key,
        label,
        kind: ParamKind::Text,
        default: ParamValue::Str(default.to_string()),
        min: None,
        max: None,
        step: None,
        options: &[],
    }
}

fn pd_select(key: &'static str, label: &'static str, default: &'static str, options: &'static [&'static str]) -> ParamDef {
    ParamDef {
        key,
        label,
        kind: ParamKind::Select,
        default: ParamValue::Str(default.to_string()),
        min: None,
        max: None,
        step: None,
        options,
    }
}

fn leak_params(params: Vec<ParamDef>) -> &'static [ParamDef] {
    Box::leak(params.into_boxed_slice())
}

const DIR_OPTIONS: &[&str] = &["clockwise", "counter-clockwise"];
const FLIP_DIR_OPTIONS: &[&str] = &["forward", "back", "left", "right"];
const GO_DIR_OPTIONS: &[&str] = &["forward", "backward", "left", "right"];
const SWAY_DIR_OPTIONS: &[&str] = &["forward-back", "left-right", "up-down", "turn-left", "turn-right", "pitch-forward", "pitch-backward", "roll-left", "roll-right"];
const LED_COLOR_OPTIONS: &[&str] = &["red", "green", "blue", "yellow", "cyan", "magenta", "white", "purple", "orange", "pink", "off"];
const OP_OPTIONS: &[&str] = &["=", "+=", "-=", "*=", "/="];
const UNIT_OPTIONS: &[&str] = &["cm", "m", "ft", "in"];
const KIND_OPTIONS: &[&str] = &["name", "rgb", "index"];
const TEMP_UNIT_OPTIONS: &[&str] = &["C", "F"];

fn build_command_defs() -> Vec<CommandDef> {
    let move_params = leak_params(vec![
        pd_num("dist", "cm", 50.0, Some(10.0), Some(300.0), None),
        pd_num("speed", "Speed", 50.0, Some(0.0), Some(100.0), None),
    ]);
    let hover_params = leak_params(vec![pd_num("dur", "Secs", 1.0, Some(0.01), Some(10.0), None)]);
    let flip_params = leak_params(vec![pd_select("dir", "Dir", "back", FLIP_DIR_OPTIONS)]);
    let go_params = leak_params(vec![
        pd_select("dir", "Dir", "forward", GO_DIR_OPTIONS),
        pd_num("power", "Power %", 50.0, Some(0.0), Some(100.0), None),
        pd_num("dur", "Secs", 1.0, Some(0.1), Some(10.0), None),
    ]);
    let turn_params = leak_params(vec![pd_num("deg", "Deg", 90.0, Some(1.0), Some(360.0), None)]);
    let turn_degree_params = leak_params(vec![
        pd_num("deg", "Deg", 90.0, Some(-360.0), Some(360.0), None),
        pd_num("timeout", "Timeout", 3.0, Some(0.1), Some(30.0), None),
        pd_num("p_value", "P Value", 10.0, Some(0.0), Some(100.0), None),
    ]);
    let circle_params = leak_params(vec![
        pd_num("speed", "Speed %", 75.0, Some(10.0), Some(100.0), None),
        pd_select("dir", "Direction", "clockwise", DIR_OPTIONS),
    ]);
    let circle_turn_params = leak_params(vec![
        pd_num("speed", "Speed %", 75.0, Some(10.0), Some(100.0), None),
        pd_num("secs", "Secs", 1.0, Some(1.0), Some(10.0), None),
        pd_select("dir", "Direction", "clockwise", DIR_OPTIONS),
    ]);
    let square_params = leak_params(vec![
        pd_num("speed", "Speed %", 60.0, Some(10.0), Some(100.0), None),
        pd_num("secs", "Secs", 1.0, Some(0.1), Some(10.0), None),
        pd_select("dir", "Direction", "clockwise", DIR_OPTIONS),
    ]);
    let triangle_params = leak_params(vec![
        pd_num("speed", "Speed %", 60.0, Some(10.0), Some(100.0), None),
        pd_num("secs", "Secs", 1.0, Some(0.1), Some(10.0), None),
        pd_select("dir", "Direction", "clockwise", DIR_OPTIONS),
    ]);
    let spiral_params = leak_params(vec![
        pd_num("speed", "Speed %", 50.0, Some(10.0), Some(100.0), None),
        pd_select("dir", "Direction", "clockwise", DIR_OPTIONS),
    ]);
    let sway_params = leak_params(vec![
        pd_num("speed", "Speed %", 50.0, Some(10.0), Some(100.0), None),
        pd_num("secs", "Secs", 2.0, Some(1.0), Some(10.0), None),
        pd_select("dir", "Direction", "forward-back", SWAY_DIR_OPTIONS),
    ]);
    let range_params = leak_params(vec![
        pd_num("dist", "cm", 50.0, Some(10.0), Some(300.0), None),
        pd_num("speed", "Speed %", 50.0, Some(10.0), Some(100.0), None),
    ]);
    let detect_wall_params = leak_params(vec![pd_text("var", "Store in", "detected")]);
    let led_params = leak_params(vec![pd_select("color", "Color", "green", LED_COLOR_OPTIONS)]);
    let buzzer_params = leak_params(vec![
        pd_num("freq", "Frequency (Hz)", 440.0, Some(100.0), Some(2000.0), None),
        pd_num("dur", "Duration (s)", 0.5, Some(0.1), Some(5.0), Some(0.1)),
    ]);
    let time_sleep_params = leak_params(vec![pd_num("dur", "Secs", 1.0, Some(0.1), Some(10.0), None)]);
    let drone_sleep_params = leak_params(vec![pd_num("dur", "Secs", 1.0, Some(0.1), Some(10.0), Some(0.1))]);
    let var_declare_params = leak_params(vec![pd_text("name", "Name", "x"), pd_text("value", "Value", "0")]);
    let set_var_params = leak_params(vec![
        pd_text("name", "Name", "x"),
        pd_select("op", "Op", "=", OP_OPTIONS),
        pd_text("value", "Value", "1"),
    ]);
    let print_var_params = leak_params(vec![pd_text("value", "Value", "x")]);
    let if_params = leak_params(vec![pd_text("condition", "Condition", "x > 0")]);
    let elif_params = leak_params(vec![pd_text("condition", "Condition", "x == 0")]);
    let while_params = leak_params(vec![pd_text("condition", "Condition", "True")]);
    let for_params = leak_params(vec![
        pd_text("var", "Var", "i"),
        pd_num("start", "Start", 0.0, None, None, None),
        pd_num("end_val", "End", 5.0, None, None, None),
        pd_num("step", "Step", 1.0, None, None, None),
    ]);
    let battery_params = leak_params(vec![pd_text("var", "Store in", "battery")]);
    let height_params = leak_params(vec![pd_text("var", "Store in", "height"), pd_select("unit", "Unit", "cm", UNIT_OPTIONS)]);
    let front_range_params = leak_params(vec![pd_text("var", "Store in", "dist"), pd_select("unit", "Unit", "cm", UNIT_OPTIONS)]);
    let bottom_range_params = leak_params(vec![pd_text("var", "Store in", "dist"), pd_select("unit", "Unit", "cm", UNIT_OPTIONS)]);
    let front_color_params = leak_params(vec![pd_text("var", "Store in", "color"), pd_select("kind", "Kind", "name", KIND_OPTIONS)]);
    let back_color_params = leak_params(vec![pd_text("var", "Store in", "color"), pd_select("kind", "Kind", "name", KIND_OPTIONS)]);
    let temperature_params = leak_params(vec![pd_text("var", "Store in", "temp"), pd_select("unit", "Unit", "C", TEMP_UNIT_OPTIONS)]);
    let distance_params = leak_params(vec![pd_text("var", "Store in", "dist")]);
    let func_def_params = leak_params(vec![pd_text("name", "Name", "my_func")]);
    let func_call_params = leak_params(vec![pd_text("name", "Name", "my_func")]);
    let return_val_params = leak_params(vec![pd_text("value", "Value", "0")]);
    let list_declare_params = leak_params(vec![pd_text("name", "Name", "my_list"), pd_text("values", "Values", "1, 2, 3")]);
    let list_append_params = leak_params(vec![pd_text("name", "List", "my_list"), pd_text("value", "Value", "0")]);
    let list_get_params = leak_params(vec![
        pd_text("list_name", "List", "my_list"),
        pd_text("index", "Index", "0"),
        pd_text("var", "Store in", "val"),
    ]);
    let user_input_params = leak_params(vec![pd_text("var", "Store in", "user_val"), pd_text("prompt", "Prompt", "Enter value: ")]);
    let timer_start_params = leak_params(vec![pd_text("name", "Name", "t")]);
    let timer_elapsed_params = leak_params(vec![pd_text("name", "Timer", "t"), pd_text("var", "Store in", "elapsed")]);
    vec![
        CommandDef { label: "Takeoff", command_type: CommandType::Takeoff, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.takeoff()") },
        CommandDef { label: "Land", command_type: CommandType::Land, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.land()") },
        CommandDef { label: "Emergency Stop", command_type: CommandType::EmergencyStop, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.emergency_stop()") },
        CommandDef { label: "Stop Motors", command_type: CommandType::StopMotors, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.stop_motors()") },
        CommandDef { label: "Hover", command_type: CommandType::Hover, params: hover_params, is_block: false, code: CodeTemplate::Function(hover_code) },
        CommandDef { label: "Flip", command_type: CommandType::Flip, params: flip_params, is_block: false, code: CodeTemplate::Function(flip_code) },
        CommandDef { label: "Go", command_type: CommandType::Go, params: go_params, is_block: false, code: CodeTemplate::Function(go_code) },
        CommandDef { label: "Forward", command_type: CommandType::MoveForward, params: move_params, is_block: false, code: CodeTemplate::Function(move_forward_code) },
        CommandDef { label: "Back", command_type: CommandType::MoveBackward, params: move_params, is_block: false, code: CodeTemplate::Function(move_backward_code) },
        CommandDef { label: "Left", command_type: CommandType::MoveLeft, params: move_params, is_block: false, code: CodeTemplate::Function(move_left_code) },
        CommandDef { label: "Right", command_type: CommandType::MoveRight, params: move_params, is_block: false, code: CodeTemplate::Function(move_right_code) },
        CommandDef { label: "Turn Left", command_type: CommandType::TurnLeft, params: turn_params, is_block: false, code: CodeTemplate::Function(turn_left_code) },
        CommandDef { label: "Turn Right", command_type: CommandType::TurnRight, params: turn_params, is_block: false, code: CodeTemplate::Function(turn_right_code) },
        CommandDef { label: "Turn", command_type: CommandType::TurnDegree, params: turn_degree_params, is_block: false, code: CodeTemplate::Function(turn_degree_code) },
        CommandDef { label: "Circle", command_type: CommandType::Circle, params: circle_params, is_block: false, code: CodeTemplate::Function(circle_code) },
        CommandDef { label: "Circle Turn", command_type: CommandType::CircleTurn, params: circle_turn_params, is_block: false, code: CodeTemplate::Function(circle_turn_code) },
        CommandDef { label: "Square", command_type: CommandType::Square, params: square_params, is_block: false, code: CodeTemplate::Function(square_code) },
        CommandDef { label: "Square Turn", command_type: CommandType::SquareTurn, params: square_params, is_block: false, code: CodeTemplate::Function(square_code) },
        CommandDef { label: "Triangle", command_type: CommandType::Triangle, params: triangle_params, is_block: false, code: CodeTemplate::Function(triangle_code) },
        CommandDef { label: "Triangle Turn", command_type: CommandType::TriangleTurn, params: triangle_params, is_block: false, code: CodeTemplate::Function(triangle_turn_code) },
        CommandDef { label: "Spiral", command_type: CommandType::Spiral, params: spiral_params, is_block: false, code: CodeTemplate::Function(spiral_code) },
        CommandDef { label: "Sway", command_type: CommandType::Sway, params: sway_params, is_block: false, code: CodeTemplate::Function(sway_code) },
        CommandDef { label: "Keep Distance", command_type: CommandType::KeepDistance, params: range_params, is_block: false, code: CodeTemplate::Function(keep_distance_code) },
        CommandDef { label: "Avoid Wall", command_type: CommandType::AvoidWall, params: range_params, is_block: false, code: CodeTemplate::Function(avoid_wall_code) },
        CommandDef { label: "Detect Wall", command_type: CommandType::DetectWall, params: detect_wall_params, is_block: false, code: CodeTemplate::Function(detect_wall_code) },
        CommandDef { label: "LED", command_type: CommandType::Led, params: led_params, is_block: false, code: CodeTemplate::Function(led_code) },
        CommandDef { label: "LED Off", command_type: CommandType::LedOff, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.drone_LED_off()") },
        CommandDef { label: "Random LED", command_type: CommandType::RandomLed, params: &[], is_block: false, code: CodeTemplate::Fixed("drone.set_drone_LED(0, 255, 128, 100)") },
        CommandDef { label: "Buzzer", command_type: CommandType::Buzzer, params: buzzer_params, is_block: false, code: CodeTemplate::Function(buzzer_code) },
        CommandDef { label: "Sleep", command_type: CommandType::TimeSleep, params: time_sleep_params, is_block: false, code: CodeTemplate::Function(sleep_code) },
        CommandDef { label: "Sleep", command_type: CommandType::DroneSleep, params: drone_sleep_params, is_block: false, code: CodeTemplate::Function(sleep_code) },
        CommandDef { label: "Var", command_type: CommandType::VarDeclare, params: var_declare_params, is_block: false, code: CodeTemplate::Function(var_declare_code) },
        CommandDef { label: "Set", command_type: CommandType::SetVar, params: set_var_params, is_block: false, code: CodeTemplate::Function(set_var_code) },
        CommandDef { label: "Print", command_type: CommandType::PrintVar, params: print_var_params, is_block: false, code: CodeTemplate::Function(print_var_code) },
        CommandDef { label: "If", command_type: CommandType::IfBlock, params: if_params, is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "Elif", command_type: CommandType::ElifBlock, params: elif_params, is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "Else", command_type: CommandType::ElseBlock, params: &[], is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "End", command_type: CommandType::EndBlock, params: &[], is_block: false, code: CodeTemplate::Fixed("") },
        CommandDef { label: "While", command_type: CommandType::WhileBlock, params: while_params, is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "For Loop", command_type: CommandType::ForBlock, params: for_params, is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "Break", command_type: CommandType::BreakCmd, params: &[], is_block: false, code: CodeTemplate::Fixed("break") },
        CommandDef { label: "Battery", command_type: CommandType::GetBattery, params: battery_params, is_block: false, code: CodeTemplate::Function(get_battery_code) },
        CommandDef { label: "Height", command_type: CommandType::GetHeight, params: height_params, is_block: false, code: CodeTemplate::Function(get_height_code) },
        CommandDef { label: "Front Range", command_type: CommandType::GetFrontRange, params: front_range_params, is_block: false, code: CodeTemplate::Function(get_front_range_code) },
        CommandDef { label: "Bottom Range", command_type: CommandType::GetBottomRange, params: bottom_range_params, is_block: false, code: CodeTemplate::Function(get_bottom_range_code) },
        CommandDef { label: "Front Color", command_type: CommandType::GetFrontColor, params: front_color_params, is_block: false, code: CodeTemplate::Function(get_front_color_code) },
        CommandDef { label: "Back Color", command_type: CommandType::GetBackColor, params: back_color_params, is_block: false, code: CodeTemplate::Function(get_back_color_code) },
        CommandDef { label: "Temperature", command_type: CommandType::GetTemperature, params: temperature_params, is_block: false, code: CodeTemplate::Function(get_temperature_code) },
        CommandDef { label: "Distance", command_type: CommandType::GetDistance, params: distance_params, is_block: false, code: CodeTemplate::Function(get_distance_code) },
        CommandDef { label: "Define Func", command_type: CommandType::FuncDef, params: func_def_params, is_block: true, code: CodeTemplate::Fixed("") },
        CommandDef { label: "Call Func", command_type: CommandType::FuncCall, params: func_call_params, is_block: false, code: CodeTemplate::Function(func_call_code) },
        CommandDef { label: "Return", command_type: CommandType::ReturnVal, params: return_val_params, is_block: false, code: CodeTemplate::Function(return_val_code) },
        CommandDef { label: "New List", command_type: CommandType::ListDeclare, params: list_declare_params, is_block: false, code: CodeTemplate::Function(list_declare_code) },
        CommandDef { label: "Append", command_type: CommandType::ListAppend, params: list_append_params, is_block: false, code: CodeTemplate::Function(list_append_code) },
        CommandDef { label: "Get Index", command_type: CommandType::ListGet, params: list_get_params, is_block: false, code: CodeTemplate::Function(list_get_code) },
        CommandDef { label: "Input", command_type: CommandType::UserInput, params: user_input_params, is_block: false, code: CodeTemplate::Function(user_input_code) },
        CommandDef { label: "Start Timer", command_type: CommandType::TimerStart, params: timer_start_params, is_block: false, code: CodeTemplate::Function(timer_start_code) },
        CommandDef { label: "Get Elapsed", command_type: CommandType::TimerElapsed, params: timer_elapsed_params, is_block: false, code: CodeTemplate::Function(timer_elapsed_code) },
    ]
}

pub static COMMAND_DEFS: LazyLock<Vec<CommandDef>> = LazyLock::new(build_command_defs);

pub fn command_defs() -> &'static [CommandDef] {
    &COMMAND_DEFS
}

pub fn js_num(v: f64) -> String {
    if v.is_nan() {
        return "NaN".to_string();
    }
    if v.is_infinite() {
        return if v > 0.0 {
            "Infinity".to_string()
        } else {
            "-Infinity".to_string()
        };
    }
    if v == v.trunc() && v.abs() < 1e21 {
        return format!("{}", v as i64);
    }
    let magnitude = v.abs();
    if magnitude >= 1e21 || (magnitude > 0.0 && magnitude < 1e-6) {
        let rendered = format!("{:e}", v);
        if let Some(pos) = rendered.find('e') {
            let (mantissa, exponent) = rendered.split_at(pos);
            let exponent_digits = &exponent[1..];
            if !exponent_digits.starts_with('-') {
                return format!("{mantissa}e+{exponent_digits}");
            }
        }
        return rendered;
    }
    format!("{v}")
}

impl Command {
    pub fn new(id: &str, command_type: CommandType) -> Command {
        Command {
            id: id.to_string(),
            command_type,
            params: BTreeMap::new(),
            children: Vec::new(),
        }
    }

    pub fn param_f64(&self, key: &str) -> Option<f64> {
        self.params.get(key).and_then(|v| v.as_f64())
    }

    pub fn param_str(&self, key: &str) -> Option<&str> {
        self.params.get(key).and_then(|v| v.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn num(v: f64) -> ParamValue {
        ParamValue::Number(v)
    }

    fn txt(v: &str) -> ParamValue {
        ParamValue::Str(v.to_string())
    }

    fn params(list: &[(&str, ParamValue)]) -> BTreeMap<String, ParamValue> {
        list.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    }

    fn code_for(command_type: CommandType, p: &BTreeMap<String, ParamValue>) -> String {
        let def = COMMAND_DEFS
            .iter()
            .find(|d| d.command_type == command_type)
            .expect("command def must exist");
        match def.code {
            CodeTemplate::Fixed(s) => s.to_string(),
            CodeTemplate::Function(f) => f(p),
        }
    }

    #[test]
    fn command_defs_match_golden_order() {
        let path = format!("{}/../../tests/golden/golden.json", env!("CARGO_MANIFEST_DIR"));
        let raw = std::fs::read_to_string(&path).expect("golden.json must load");
        let golden: serde_json::Value = serde_json::from_str(&raw).expect("golden must parse");
        let types = golden["commandTypes"].as_array().expect("commandTypes array");
        assert_eq!(COMMAND_DEFS.len(), 58);
        assert_eq!(types.len(), 58);
        for (def, name) in COMMAND_DEFS.iter().zip(types.iter()) {
            assert_eq!(def.command_type.as_str(), name.as_str().expect("type name"));
            assert_eq!(def.is_block, def.command_type.is_block());
        }
    }

    #[test]
    fn code_templates_match_golden_strings() {
        let empty = params(&[]);
        assert_eq!(code_for(CommandType::Takeoff, &empty), "drone.takeoff()");
        assert_eq!(code_for(CommandType::Land, &empty), "drone.land()");
        assert_eq!(code_for(CommandType::EmergencyStop, &empty), "drone.emergency_stop()");
        assert_eq!(code_for(CommandType::StopMotors, &empty), "drone.stop_motors()");
        assert_eq!(code_for(CommandType::LedOff, &empty), "drone.drone_LED_off()");
        assert_eq!(code_for(CommandType::RandomLed, &empty), "drone.set_drone_LED(0, 255, 128, 100)");
        assert_eq!(code_for(CommandType::EndBlock, &empty), "");
        assert_eq!(code_for(CommandType::BreakCmd, &empty), "break");
        assert_eq!(code_for(CommandType::IfBlock, &empty), "");
        assert_eq!(code_for(CommandType::ElifBlock, &empty), "");
        assert_eq!(code_for(CommandType::ElseBlock, &empty), "");
        assert_eq!(code_for(CommandType::WhileBlock, &empty), "");
        assert_eq!(code_for(CommandType::ForBlock, &empty), "");
        assert_eq!(code_for(CommandType::FuncDef, &empty), "");
        assert_eq!(
            code_for(CommandType::Hover, &params(&[("dur", num(2.0))])),
            "drone.hover(2)"
        );
        assert_eq!(
            code_for(CommandType::Hover, &params(&[("dur", num(1.0))])),
            "drone.hover(1)"
        );
        assert_eq!(
            code_for(CommandType::Flip, &params(&[("dir", txt("back"))])),
            "drone.flip(\"back\")"
        );
        assert_eq!(
            code_for(
                CommandType::Go,
                &params(&[("dir", txt("forward")), ("power", num(60.0)), ("dur", num(1.0))])
            ),
            "drone.go(\"forward\", 60, 1)"
        );
        assert_eq!(
            code_for(CommandType::MoveForward, &params(&[("dist", num(50.0)), ("speed", num(50.0))])),
            "drone.move_forward(50, speed=1.0)"
        );
        assert_eq!(
            code_for(CommandType::MoveLeft, &params(&[("dist", num(40.0)), ("speed", num(60.0))])),
            "drone.move_left(40, speed=1.2)"
        );
        assert_eq!(
            code_for(CommandType::MoveForward, &params(&[("dist", num(200.0)), ("speed", num(75.0))])),
            "drone.move_forward(200, speed=1.5)"
        );
        assert_eq!(
            code_for(CommandType::MoveForward, &params(&[("dist", num(30.0)), ("speed", num(40.0))])),
            "drone.move_forward(30, speed=0.8)"
        );
        assert_eq!(
            code_for(CommandType::MoveBackward, &params(&[("dist", num(40.0)), ("speed", num(60.0))])),
            "drone.move_backward(40, speed=1.2)"
        );
        assert_eq!(
            code_for(CommandType::TurnLeft, &params(&[("deg", num(45.0))])),
            "drone.turn_left(45)"
        );
        assert_eq!(
            code_for(
                CommandType::TurnDegree,
                &params(&[("deg", num(135.0)), ("timeout", num(3.0)), ("p_value", num(10.0))])
            ),
            "drone.turn_degree(135, timeout=3, p_value=10)"
        );
        assert_eq!(
            code_for(CommandType::Circle, &params(&[("speed", num(75.0)), ("dir", txt("clockwise"))])),
            "drone.circle(speed=75, direction=1)"
        );
        assert_eq!(
            code_for(
                CommandType::CircleTurn,
                &params(&[("speed", num(75.0)), ("secs", num(2.0)), ("dir", txt("counter-clockwise"))])
            ),
            "drone.circle_turn(speed=75, seconds=2, direction=-1)"
        );
        assert_eq!(
            code_for(
                CommandType::Square,
                &params(&[("speed", num(60.0)), ("secs", num(1.0)), ("dir", txt("clockwise"))])
            ),
            "drone.square(speed=60, seconds=1, direction=1)"
        );
        assert_eq!(
            code_for(
                CommandType::SquareTurn,
                &params(&[("speed", num(75.0)), ("secs", num(1.0)), ("dir", txt("counter-clockwise"))])
            ),
            "drone.square(speed=75, seconds=1, direction=-1)"
        );
        assert_eq!(
            code_for(
                CommandType::Triangle,
                &params(&[("speed", num(60.0)), ("secs", num(1.0)), ("dir", txt("clockwise"))])
            ),
            "drone.triangle(speed=60, seconds=1, direction=1)"
        );
        assert_eq!(
            code_for(
                CommandType::TriangleTurn,
                &params(&[("speed", num(60.0)), ("secs", num(2.0)), ("dir", txt("counter-clockwise"))])
            ),
            "drone.triangle_turn(speed=60, seconds=2, direction=-1)"
        );
        assert_eq!(
            code_for(CommandType::Spiral, &params(&[("speed", num(50.0)), ("dir", txt("clockwise"))])),
            "drone.spiral(speed=50, seconds=3, direction=1)"
        );
        assert_eq!(
            code_for(
                CommandType::Sway,
                &params(&[("speed", num(50.0)), ("secs", num(2.0)), ("dir", txt("turn-right"))])
            ),
            "drone.sway(speed=50, seconds=2, direction=-1)"
        );
        assert_eq!(
            code_for(
                CommandType::Sway,
                &params(&[("speed", num(30.0)), ("secs", num(1.0)), ("dir", txt("left-right"))])
            ),
            "drone.sway(speed=30, seconds=1, direction=1)"
        );
        assert_eq!(
            code_for(CommandType::KeepDistance, &params(&[("dist", num(50.0)), ("speed", num(50.0))])),
            "drone.keep_distance(2, 50)"
        );
        assert_eq!(
            code_for(CommandType::AvoidWall, &params(&[("dist", num(70.0)), ("speed", num(50.0))])),
            "drone.avoid_wall(2, 70)"
        );
        assert_eq!(
            code_for(CommandType::DetectWall, &params(&[("var", txt("det"))])),
            "det = drone.detect_wall()"
        );
        assert_eq!(code_for(CommandType::Led, &params(&[("color", txt("green"))])), "drone.set_drone_LED(0, 255, 0, 100)");
        assert_eq!(code_for(CommandType::Led, &params(&[("color", txt("yellow"))])), "drone.set_drone_LED(255, 255, 0, 100)");
        assert_eq!(code_for(CommandType::Led, &params(&[("color", txt("purple"))])), "drone.set_drone_LED(128, 0, 255, 100)");
        assert_eq!(code_for(CommandType::Led, &params(&[("color", txt("off"))])), "drone.drone_LED_off()");
        assert_eq!(
            code_for(CommandType::Buzzer, &params(&[("freq", num(440.0)), ("dur", num(0.5))])),
            "drone.drone_buzzer(\"A4\", 0.5)"
        );
        assert_eq!(
            code_for(CommandType::Buzzer, &params(&[("freq", num(523.0)), ("dur", num(0.2))])),
            "drone.drone_buzzer(\"C5\", 0.2)"
        );
        assert_eq!(code_for(CommandType::TimeSleep, &params(&[("dur", num(0.5))])), "time.sleep(0.5)");
        assert_eq!(code_for(CommandType::DroneSleep, &params(&[("dur", num(1.0))])), "time.sleep(1)");
        assert_eq!(
            code_for(CommandType::VarDeclare, &params(&[("name", txt("x")), ("value", num(3.0))])),
            "x = 3"
        );
        assert_eq!(
            code_for(
                CommandType::SetVar,
                &params(&[("name", txt("x")), ("op", txt("-=")), ("value", num(1.0))])
            ),
            "x -= 1"
        );
        assert_eq!(code_for(CommandType::PrintVar, &params(&[("value", txt("v"))])), "print(v)");
        assert_eq!(
            code_for(CommandType::GetBattery, &params(&[("var", txt("battery"))])),
            "battery = drone.get_battery()"
        );
        assert_eq!(
            code_for(CommandType::GetHeight, &params(&[("var", txt("h")), ("unit", txt("cm"))])),
            "h = drone.get_height(unit=\"cm\")"
        );
        assert_eq!(
            code_for(CommandType::GetFrontRange, &params(&[("var", txt("fr")), ("unit", txt("cm"))])),
            "fr = drone.get_front_range(unit=\"cm\")"
        );
        assert_eq!(
            code_for(CommandType::GetBottomRange, &params(&[("var", txt("br")), ("unit", txt("cm"))])),
            "br = drone.get_bottom_range(unit=\"cm\")"
        );
        assert_eq!(
            code_for(CommandType::GetFrontColor, &params(&[("var", txt("fc")), ("kind", txt("name"))])),
            "fc = drone.get_front_color(kind=\"name\")"
        );
        assert_eq!(
            code_for(CommandType::GetBackColor, &params(&[("var", txt("bc")), ("kind", txt("name"))])),
            "bc = drone.get_back_color(kind=\"name\")"
        );
        assert_eq!(
            code_for(CommandType::GetTemperature, &params(&[("var", txt("t")), ("unit", txt("C"))])),
            "t = drone.get_temperature(unit=\"C\")"
        );
        assert_eq!(
            code_for(CommandType::GetDistance, &params(&[("var", txt("dist"))])),
            "dist = drone.get_front_range()"
        );
        assert_eq!(code_for(CommandType::FuncCall, &params(&[("name", txt("jig"))])), "jig()");
        assert_eq!(
            code_for(CommandType::ReturnVal, &params(&[("value", txt("0"))])),
            "return 0"
        );
        assert_eq!(
            code_for(
                CommandType::ListDeclare,
                &params(&[("name", txt("L")), ("values", txt("1, 2, 3"))])
            ),
            "L = [1, 2, 3]"
        );
        assert_eq!(
            code_for(CommandType::ListAppend, &params(&[("name", txt("L")), ("value", num(4.0))])),
            "L.append(4)"
        );
        assert_eq!(
            code_for(
                CommandType::ListGet,
                &params(&[("list_name", txt("L")), ("index", num(2.0)), ("var", txt("v"))])
            ),
            "v = L[2]"
        );
        assert_eq!(
            code_for(
                CommandType::UserInput,
                &params(&[("var", txt("u")), ("prompt", txt("Enter value: "))])
            ),
            "u = input(\"Enter value: \")"
        );
        assert_eq!(code_for(CommandType::TimerStart, &params(&[("name", txt("t"))])), "t = time.time()");
        assert_eq!(
            code_for(CommandType::TimerElapsed, &params(&[("name", txt("t")), ("var", txt("e"))])),
            "e = time.time() - t"
        );
    }
}
