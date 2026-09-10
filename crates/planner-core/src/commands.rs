use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

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
    rest[start..end].parse::<f64>().ok()
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

pub static COMMAND_DEFS: &[CommandDef] = &[];

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
