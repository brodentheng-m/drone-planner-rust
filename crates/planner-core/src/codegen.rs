use crate::commands::{
    interp, interp_param, CodeTemplate, CommandType, Command, ParamValue, COMMAND_DEFS,
};
use crate::planio::Plan;

fn command_line(command: &Command) -> Option<String> {
    let def = COMMAND_DEFS
        .iter()
        .find(|def| def.command_type == command.command_type)?;
    let code = match def.code {
        CodeTemplate::Fixed(text) => text.to_string(),
        CodeTemplate::Function(func) => func(&command.params),
    };
    if code.is_empty() {
        None
    } else {
        Some(code)
    }
}

fn sub_code(code: &str, drone_var: &str) -> String {
    if drone_var == "drone" {
        code.to_string()
    } else {
        code.replace("drone.", &format!("{drone_var}."))
    }
}

fn header_text(
    params: &std::collections::BTreeMap<String, crate::commands::ParamValue>,
    key: &str,
    fallback: &str,
    drone_var: &str,
) -> String {
    let raw = match params.get(key) {
        Some(value) => interp(value),
        None => fallback.to_string(),
    };
    sub_code(&raw, drone_var)
}

fn raw_param(
    params: &std::collections::BTreeMap<String, crate::commands::ParamValue>,
    key: &str,
    fallback: &str,
) -> String {
    match params.get(key) {
        Some(value) => interp(value),
        None => fallback.to_string(),
    }
}

fn block_header(command: &Command, indent: usize, drone_var: &str) -> Option<String> {
    let pad = "    ".repeat(indent);
    let params = &command.params;
    let condition_fallback = if drone_var == "drone" { "undefined" } else { "" };
    let line = match command.command_type {
        CommandType::IfBlock => format!(
            "{}if {}:",
            pad,
            header_text(params, "condition", condition_fallback, drone_var)
        ),
        CommandType::ElifBlock => format!(
            "{}elif {}:",
            pad,
            header_text(params, "condition", condition_fallback, drone_var)
        ),
        CommandType::ElseBlock => format!("{pad}else:"),
        CommandType::WhileBlock => format!(
            "{}while {}:",
            pad,
            header_text(params, "condition", condition_fallback, drone_var)
        ),
        CommandType::ForBlock => format!(
            "{}for {} in range({}, {}, {}):",
            pad,
            header_text(
                params,
                "var",
                if drone_var == "drone" { "i" } else { "" },
                drone_var
            ),
            raw_param(params, "start", "0"),
            raw_param(params, "end_val", "5"),
            raw_param(params, "step", "1")
        ),
        CommandType::FuncDef => format!(
            "{}def {}():",
            pad,
            header_text(params, "name", condition_fallback, drone_var)
        ),
        _ => return None,
    };
    Some(line)
}

fn generate_block(commands: &[Command], lines: &mut Vec<String>, indent: usize, drone_var: &str) {
    let pad = "    ".repeat(indent);
    for command in commands {
        if let Some(header) = block_header(command, indent, drone_var) {
            lines.push(header);
            if command.children.is_empty() {
                lines.push(format!("{pad}    pass"));
            } else {
                generate_block(&command.children, lines, indent + 1, drone_var);
            }
            continue;
        }
        if let Some(code) = command_line(command) {
            lines.push(format!("{pad}{}", sub_code(&code, drone_var)));
        }
    }
}

fn active_commands(plan: &Plan) -> &[Command] {
    let selected = plan
        .active_drone_id
        .as_ref()
        .and_then(|id| plan.drones.iter().find(|drone| &drone.id == id))
        .or_else(|| plan.drones.first());
    match selected {
        Some(drone) => drone.commands.as_slice(),
        None => &[],
    }
}

type Params = std::collections::BTreeMap<String, crate::commands::ParamValue>;

fn anim_num(params: &Params, key: &str, default: f64) -> String {
    crate::commands::js_num(params.get(key).and_then(|v| v.as_f64()).unwrap_or(default))
}

fn anim_str(params: &Params, key: &str, default: &str) -> String {
    params
        .get(key)
        .and_then(|v| v.as_str())
        .unwrap_or(default)
        .to_string()
}

fn generate_sim_block(commands: &[Command], lines: &mut Vec<String>, indent: usize) {
    let pad = "    ".repeat(indent);
    for command in commands {
        if let Some(header) = block_header(command, indent, "drone") {
            lines.push(header);
            if command.children.is_empty() {
                lines.push(format!("{pad}    pass"));
            } else {
                generate_sim_block(&command.children, lines, indent + 1);
            }
            continue;
        }
        sim_command(command, &pad, lines);
    }
}

fn sim_command(command: &Command, pad: &str, lines: &mut Vec<String>) {
    let p = &command.params;
    match command.command_type {
        CommandType::Takeoff => {
            lines.push(format!("{pad}# Takeoff"));
            lines.push(format!("{pad}steps = max(int(TAKEOFF_HEIGHT / (DEFAULT_SPEED * DT)), 10)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            lines.push(format!("{pad}    z = TAKEOFF_HEIGHT * t"));
            lines.push(format!("{pad}    positions.append((x, y, z))"));
            lines.push(format!("{pad}is_flying = True"));
        }
        CommandType::Land => {
            lines.push(format!("{pad}# Land"));
            lines.push(format!("{pad}start_z = z"));
            lines.push(format!("{pad}steps = max(int(start_z / (DEFAULT_SPEED * DT)), 10)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            lines.push(format!("{pad}    z = start_z * (1 - t)"));
            lines.push(format!("{pad}    positions.append((x, y, z))"));
            lines.push(format!("{pad}is_flying = False"));
            lines.push(format!("{pad}z = 0.0"));
        }
        CommandType::Hover => {
            let dur = interp_param(p, "dur");
            lines.push(format!("{pad}# Hover {dur}s"));
            lines.push(format!("{pad}steps = int({dur} / DT)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    positions.append((x, y, z))"));
        }
        CommandType::Flip => {
            let dir = anim_str(p, "dir", "back");
            lines.push(format!("{pad}# Flip {dir}"));
            lines.push(format!("{pad}for i in range(40):"));
            lines.push(format!("{pad}    t = i / 40"));
            lines.push(format!("{pad}    angle = 2 * math.pi * t"));
            match dir.as_str() {
                "back" => {
                    lines.push(format!("{pad}    fx = x - FLIP_RADIUS * math.sin(angle) * math.cos(math.radians(heading))"));
                    lines.push(format!("{pad}    fy = y - FLIP_RADIUS * math.sin(angle) * math.sin(math.radians(heading))"));
                }
                "forward" => {
                    lines.push(format!("{pad}    fx = x + FLIP_RADIUS * math.sin(angle) * math.cos(math.radians(heading))"));
                    lines.push(format!("{pad}    fy = y + FLIP_RADIUS * math.sin(angle) * math.sin(math.radians(heading))"));
                }
                "left" => {
                    lines.push(format!("{pad}    fx = x - FLIP_RADIUS * math.sin(angle) * math.sin(math.radians(heading))"));
                    lines.push(format!("{pad}    fy = y + FLIP_RADIUS * math.sin(angle) * math.cos(math.radians(heading))"));
                }
                _ => {
                    lines.push(format!("{pad}    fx = x + FLIP_RADIUS * math.sin(angle) * math.sin(math.radians(heading))"));
                    lines.push(format!("{pad}    fy = y - FLIP_RADIUS * math.sin(angle) * math.cos(math.radians(heading))"));
                }
            }
            lines.push(format!("{pad}    fz = z + FLIP_RADIUS * (1 - math.cos(angle))"));
            lines.push(format!("{pad}    positions.append((fx, fy, fz))"));
            match dir.as_str() {
                "back" => {
                    lines.push(format!("{pad}x -= FLIP_RADIUS * 2 * math.cos(math.radians(heading))"));
                    lines.push(format!("{pad}y -= FLIP_RADIUS * 2 * math.sin(math.radians(heading))"));
                }
                "forward" => {
                    lines.push(format!("{pad}x += FLIP_RADIUS * 2 * math.cos(math.radians(heading))"));
                    lines.push(format!("{pad}y += FLIP_RADIUS * 2 * math.sin(math.radians(heading))"));
                }
                "left" => {
                    lines.push(format!("{pad}x -= FLIP_RADIUS * 2 * math.sin(math.radians(heading))"));
                    lines.push(format!("{pad}y += FLIP_RADIUS * 2 * math.cos(math.radians(heading))"));
                }
                _ => {
                    lines.push(format!("{pad}x += FLIP_RADIUS * 2 * math.sin(math.radians(heading))"));
                    lines.push(format!("{pad}y -= FLIP_RADIUS * 2 * math.cos(math.radians(heading))"));
                }
            }
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::Go => {
            let dir = anim_str(p, "dir", "forward");
            let power = anim_num(p, "power", 50.0);
            let dur = anim_num(p, "dur", 1.0);
            lines.push(format!("{pad}# Go {dir} power={power} dur={dur}s"));
            lines.push(format!("{pad}speed = ({power} / 100.0) * DEFAULT_SPEED * 2"));
            lines.push(format!("{pad}steps = max(int({dur} / DT), 5)"));
            lines.push(format!("{pad}rad = math.radians(heading)"));
            match dir.as_str() {
                "forward" => lines.push(format!("{pad}dx, dy = math.cos(rad), math.sin(rad)")),
                "backward" => lines.push(format!("{pad}dx, dy = -math.cos(rad), -math.sin(rad)")),
                "left" => lines.push(format!("{pad}dx, dy = -math.sin(rad), math.cos(rad)")),
                _ => lines.push(format!("{pad}dx, dy = math.sin(rad), -math.cos(rad)")),
            }
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            lines.push(format!("{pad}    px = x + dx * speed * {dur} * t"));
            lines.push(format!("{pad}    py = y + dy * speed * {dur} * t"));
            lines.push(format!("{pad}    positions.append((px, py, z))"));
            lines.push(format!("{pad}x += dx * speed * {dur}"));
            lines.push(format!("{pad}y += dy * speed * {dur}"));
        }
        CommandType::MoveForward | CommandType::MoveBackward => {
            let sign = if command.command_type == CommandType::MoveForward { "" } else { "-" };
            let dist = anim_num(p, "dist", 50.0);
            let speed = anim_num(p, "speed", 50.0);
            let label = if command.command_type == CommandType::MoveForward { "Forward" } else { "Backward" };
            lines.push(format!("{pad}# {label} {dist}cm speed={speed}"));
            lines.push(format!("{pad}dist_m = {dist} / 100.0"));
            lines.push(format!("{pad}rad = math.radians(heading)"));
            lines.push(format!("{pad}steps = max(int(dist_m / (DEFAULT_SPEED * DT)), 5)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            lines.push(format!("{pad}    px = x {sign} math.cos(rad) * dist_m * t"));
            lines.push(format!("{pad}    py = y {sign} math.sin(rad) * dist_m * t"));
            lines.push(format!("{pad}    positions.append((px, py, z))"));
            lines.push(format!("{pad}x {sign}= math.cos(rad) * dist_m"));
            lines.push(format!("{pad}y {sign}= math.sin(rad) * dist_m"));
        }
        CommandType::MoveLeft | CommandType::MoveRight => {
            let dist = anim_num(p, "dist", 50.0);
            let speed = anim_num(p, "speed", 50.0);
            let label = if command.command_type == CommandType::MoveLeft { "Left" } else { "Right" };
            lines.push(format!("{pad}# {label} {dist}cm speed={speed}"));
            lines.push(format!("{pad}dist_m = {dist} / 100.0"));
            lines.push(format!("{pad}rad = math.radians(heading)"));
            lines.push(format!("{pad}steps = max(int(dist_m / (DEFAULT_SPEED * DT)), 5)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            if command.command_type == CommandType::MoveLeft {
                lines.push(format!("{pad}    px = x - math.sin(rad) * dist_m * t"));
                lines.push(format!("{pad}    py = y + math.cos(rad) * dist_m * t"));
            } else {
                lines.push(format!("{pad}    px = x + math.sin(rad) * dist_m * t"));
                lines.push(format!("{pad}    py = y - math.cos(rad) * dist_m * t"));
            }
            lines.push(format!("{pad}    positions.append((px, py, z))"));
            if command.command_type == CommandType::MoveLeft {
                lines.push(format!("{pad}x -= math.sin(rad) * dist_m"));
                lines.push(format!("{pad}y += math.cos(rad) * dist_m"));
            } else {
                lines.push(format!("{pad}x += math.sin(rad) * dist_m"));
                lines.push(format!("{pad}y -= math.cos(rad) * dist_m"));
            }
        }
        CommandType::TurnLeft | CommandType::TurnRight => {
            let sign = if command.command_type == CommandType::TurnLeft { "+" } else { "-" };
            let deg = anim_num(p, "deg", 90.0);
            let label = if command.command_type == CommandType::TurnLeft { "Left" } else { "Right" };
            lines.push(format!("{pad}# Turn {label} {deg}deg"));
            lines.push(format!("{pad}heading {sign}= {deg}"));
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::TurnDegree => {
            let deg = anim_num(p, "deg", 90.0);
            let timeout = anim_num(p, "timeout", 3.0);
            lines.push(format!("{pad}# Turn {deg}deg timeout={timeout}"));
            lines.push(format!("{pad}heading += {deg}"));
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::Circle | CommandType::CircleTurn => {
            let speed = anim_num(p, "speed", 75.0);
            let direction = if p.get("dir").and_then(|v| v.as_str()) == Some("counter-clockwise") { -1 } else { 1 };
            lines.push(format!("{pad}# Circle speed={speed} direction={direction}"));
            lines.push(format!("{pad}for i in range(60):"));
            lines.push(format!("{pad}    angle = 2 * math.pi * i / 60 * {direction}"));
            lines.push(format!("{pad}    fx = x + 0.5 * math.cos(angle)"));
            lines.push(format!("{pad}    fy = y + 0.5 * math.sin(angle)"));
            lines.push(format!("{pad}    positions.append((fx, fy, z))"));
            lines.push(format!("{pad}x += 0.5 * {direction}"));
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::Square | CommandType::SquareTurn | CommandType::Triangle | CommandType::TriangleTurn => {
            let speed = anim_num(p, "speed", 60.0);
            let secs = anim_num(p, "secs", 1.0);
            let direction = if p.get("dir").and_then(|v| v.as_str()) == Some("counter-clockwise") { -1 } else { 1 };
            let is_triangle = matches!(command.command_type, CommandType::Triangle | CommandType::TriangleTurn);
            let num_sides = if is_triangle { 3 } else { 4 };
            let label = if is_triangle { "Triangle" } else { "Square" };
            lines.push(format!("{pad}# {label} speed={speed} secs={secs} direction={direction}"));
            lines.push(format!("{pad}rad = math.radians(heading)"));
            lines.push(format!("{pad}for i in range({num_sides}):"));
            lines.push(format!("{pad}    angle = rad + i * (2 * math.pi / {num_sides}) * {direction}"));
            lines.push(format!("{pad}    for j in range(15):"));
            lines.push(format!("{pad}        t = (j + 1) / 15"));
            lines.push(format!("{pad}        px = x + math.cos(angle) * 0.5 * t"));
            lines.push(format!("{pad}        py = y + math.sin(angle) * 0.5 * t"));
            lines.push(format!("{pad}        positions.append((px, py, z))"));
            lines.push(format!("{pad}    x += math.cos(angle) * 0.5"));
            lines.push(format!("{pad}    y += math.sin(angle) * 0.5"));
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::Spiral => {
            let speed = anim_num(p, "speed", 50.0);
            let direction = if p.get("dir").and_then(|v| v.as_str()) == Some("counter-clockwise") { -1 } else { 1 };
            lines.push(format!("{pad}# Spiral speed={speed} direction={direction}"));
            lines.push(format!("{pad}for i in range(120):"));
            lines.push(format!("{pad}    t = i / 120"));
            lines.push(format!("{pad}    angle = 4 * math.pi * t * {direction}"));
            lines.push(format!("{pad}    radius = t * 0.5"));
            lines.push(format!("{pad}    fx = x + radius * math.cos(angle)"));
            lines.push(format!("{pad}    fy = y + radius * math.sin(angle)"));
            lines.push(format!("{pad}    fz = z + t * 0.3"));
            lines.push(format!("{pad}    positions.append((fx, fy, fz))"));
            lines.push(format!("{pad}x += 0.5 * math.cos(4 * math.pi * {direction})"));
            lines.push(format!("{pad}y += 0.5 * math.sin(4 * math.pi * {direction})"));
            lines.push(format!("{pad}z += 0.3"));
            lines.push(format!("{pad}positions.append((x, y, z))"));
        }
        CommandType::Sway => {
            let speed = anim_num(p, "speed", 50.0);
            let dir = anim_str(p, "dir", "forward-back");
            lines.push(format!("{pad}# Sway speed={speed} dir={dir}"));
            lines.push(format!("{pad}for i in range(40):"));
            lines.push(format!("{pad}    t = i / 40"));
            lines.push(format!("{pad}    angle = 2 * math.pi * t"));
            lines.push(format!("{pad}    positions.append((x, y, z))"));
        }
        CommandType::KeepDistance | CommandType::AvoidWall => {
            let dist = anim_num(p, "dist", 50.0);
            let speed = anim_num(p, "speed", 50.0);
            let label = if command.command_type == CommandType::KeepDistance { "Keep Distance" } else { "Avoid Wall" };
            lines.push(format!("{pad}# {label} {dist}cm speed={speed}"));
            lines.push(format!("{pad}rad = math.radians(heading)"));
            lines.push(format!("{pad}dist_m = {dist} / 100.0"));
            lines.push(format!("{pad}steps = max(int(dist_m / (DEFAULT_SPEED * DT)), 5)"));
            lines.push(format!("{pad}for i in range(steps):"));
            lines.push(format!("{pad}    t = (i + 1) / steps"));
            lines.push(format!("{pad}    px = x - math.cos(rad) * dist_m * t"));
            lines.push(format!("{pad}    py = y - math.sin(rad) * dist_m * t"));
            lines.push(format!("{pad}    positions.append((px, py, z))"));
            lines.push(format!("{pad}x -= math.cos(rad) * dist_m"));
            lines.push(format!("{pad}y -= math.sin(rad) * dist_m"));
        }
        CommandType::DetectWall => lines.push(format!("{pad}{} = 0", interp_param(p, "var"))),
        CommandType::Led => lines.push(format!("{pad}# LED color={}", interp_param(p, "color"))),
        CommandType::LedOff => lines.push(format!("{pad}# LED Off")),
        CommandType::RandomLed => lines.push(format!("{pad}# Random LED")),
        CommandType::Buzzer => lines.push(format!(
            "{pad}# Buzzer freq={}Hz dur={}s",
            interp_param(p, "freq"),
            interp_param(p, "dur")
        )),
        CommandType::VarDeclare => lines.push(format!("{pad}{} = {}", interp_param(p, "name"), interp_param(p, "value"))),
        CommandType::SetVar => lines.push(format!(
            "{pad}{} {} {}",
            interp_param(p, "name"),
            interp_param(p, "op"),
            interp_param(p, "value")
        )),
        CommandType::PrintVar => lines.push(format!("{pad}print({})", interp_param(p, "value"))),
        CommandType::BreakCmd => lines.push(format!("{pad}break")),
        CommandType::GetBattery => lines.push(format!("{pad}{} = 80", interp_param(p, "var"))),
        CommandType::GetHeight => lines.push(format!("{pad}{} = z * 100", interp_param(p, "var"))),
        CommandType::GetFrontRange => lines.push(format!("{pad}{} = 100", interp_param(p, "var"))),
        CommandType::GetBottomRange => lines.push(format!("{pad}{} = z * 100", interp_param(p, "var"))),
        CommandType::GetFrontColor => lines.push(format!("{pad}{} = \"green\"", interp_param(p, "var"))),
        CommandType::GetBackColor => lines.push(format!("{pad}{} = \"blue\"", interp_param(p, "var"))),
        CommandType::GetTemperature => lines.push(format!("{pad}{} = 22.0", interp_param(p, "var"))),
        CommandType::GetDistance => lines.push(format!("{pad}{} = 100", interp_param(p, "var"))),
        CommandType::FuncCall => lines.push(format!("{pad}{}()", interp_param(p, "name"))),
        CommandType::ReturnVal => lines.push(format!("{pad}return {}", interp_param(p, "value"))),
        CommandType::ListDeclare => lines.push(format!("{pad}{} = [{}]", interp_param(p, "name"), interp_param(p, "values"))),
        CommandType::ListAppend => lines.push(format!("{pad}{}.append({})", interp_param(p, "name"), interp_param(p, "value"))),
        CommandType::ListGet => lines.push(format!(
            "{pad}{} = {}[{}]",
            interp_param(p, "var"),
            interp_param(p, "list_name"),
            interp_param(p, "index")
        )),
        CommandType::UserInput => lines.push(format!("{pad}{} = 0", interp_param(p, "var"))),
        CommandType::TimerStart => lines.push(format!("{pad}{} = time.time()", interp_param(p, "name"))),
        CommandType::TimerElapsed => lines.push(format!(
            "{pad}{} = time.time() - {}",
            interp_param(p, "var"),
            interp_param(p, "name")
        )),
        CommandType::TimeSleep => lines.push(format!("{pad}# Sleep {}s", interp_param(p, "dur"))),
        _ => {}
    }
}

pub fn generate_code(plan: &Plan) -> String {
    let mut lines = vec![
        "from codrone_edu.drone import *".to_string(),
        "import time".to_string(),
        String::new(),
        "drone = Drone()".to_string(),
        "drone.pair()".to_string(),
        String::new(),
    ];
    generate_block(active_commands(plan), &mut lines, 0, "drone");
    lines.push(String::new());
    lines.push("drone.close()".to_string());
    lines.join("\n")
}

pub fn generate_swarm_code(plan: &Plan) -> String {
    if plan.drones.len() == 1 {
        return generate_code(plan);
    }
    let count = plan.drones.len();
    let mut lines = vec![
        "from codrone_edu.drone import *".to_string(),
        "import time".to_string(),
        "import threading".to_string(),
        String::new(),
    ];
    for index in 0..count {
        lines.push(format!("drone{} = Drone()", index + 1));
    }
    lines.push(String::new());
    for index in 0..count {
        lines.push(format!("drone{}.pair()", index + 1));
    }
    lines.push(String::new());
    lines.push("time.sleep(2)".to_string());
    lines.push(String::new());
    for (index, drone) in plan.drones.iter().enumerate() {
        lines.push(format!("def fly_drone{}():", index + 1));
        lines.push(format!("    d = drone{}", index + 1));
        generate_block(&drone.commands, &mut lines, 1, "d");
        lines.push(String::new());
    }
    lines.push("threads = []".to_string());
    for index in 0..count {
        lines.push(format!(
            "t{} = threading.Thread(target=fly_drone{})",
            index + 1,
            index + 1
        ));
        lines.push(format!("threads.append(t{})", index + 1));
    }
    lines.push(String::new());
    lines.push("for t in threads:".to_string());
    lines.push("    t.start()".to_string());
    lines.push("for t in threads:".to_string());
    lines.push("    t.join()".to_string());
    lines.push(String::new());
    for index in 0..count {
        lines.push(format!("drone{}.close()", index + 1));
    }
    lines.join("\n")
}

pub fn generate_animation_code(plan: &Plan) -> String {
    let mut lines: Vec<String> = [
        "import matplotlib.pyplot as plt",
        "import matplotlib.animation as animation",
        "from mpl_toolkits.mplot3d import Axes3D",
        "import numpy as np",
        "import math",
        "",
        "TAKEOFF_HEIGHT = 0.8",
        "DEFAULT_SPEED = 0.5",
        "DT = 0.05",
        "FLIP_RADIUS = 0.3",
        "",
        "",
        "def simulate_commands():",
        "    x, y, z = 0.0, 0.0, 0.0",
        "    heading = 0.0",
        "    positions = [(x, y, z)]",
        "",
        "    is_flying = False",
        "",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    generate_sim_block(active_commands(plan), &mut lines, 1);
    let tail: Vec<String> = [
        "",
        "    return positions",
        "",
        "",
        "def animate():",
        "    positions = simulate_commands()",
        "    fig = plt.figure(figsize=(12, 8))",
        "    ax = fig.add_subplot(111, projection=\"3d\")",
        "    xs = [p[0] for p in positions]",
        "    ys = [p[1] for p in positions]",
        "    zs = [p[2] for p in positions]",
        "",
        "    ax.set_xlabel(\"X (m)\")",
        "    ax.set_ylabel(\"Y (m)\")",
        "    ax.set_zlabel(\"Z (m)\")",
        "    ax.set_title(\"Drone Flight Simulation\")",
        "",
        "    trail, = ax.plot([], [], [], \"b-\", linewidth=1.5, alpha=0.6)",
        "    drone_dot = ax.scatter([], [], [], c=\"red\", s=100, marker=\"^\")",
        "",
        "    total = len(positions)",
        "    skip = max(1, total // 100)",
        "    indices = list(range(0, total, skip))",
        "    if indices[-1] != total - 1:",
        "        indices.append(total - 1)",
        "",
        "    def update(frame):",
        "        idx = indices[frame]",
        "        trail.set_data(xs[:idx+1], ys[:idx+1])",
        "        trail.set_3d_properties(zs[:idx+1])",
        "        drone_dot._offsets3d = ([xs[idx]], [ys[idx]], [zs[idx]])",
        "        return trail, drone_dot",
        "",
        "    ani = animation.FuncAnimation(fig, update, frames=len(indices), interval=30, blit=False, repeat=False)",
        "    ax.legend([\"Path\", \"Drone\"])",
        "    plt.tight_layout()",
        "    plt.show()",
        "",
        "",
        "if __name__ == \"__main__\":",
        "    animate()",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    lines.extend(tail);
    lines.join("\n")
}

fn leading_word(line: &str) -> &str {
    match line.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')) {
        Some(end) => &line[..end],
        None => line,
    }
}

fn take_word(text: &str) -> Option<(&str, &str)> {
    let end = match text.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')) {
        Some(end) => end,
        None => text.len(),
    };
    if end == 0 {
        None
    } else {
        Some((&text[..end], &text[end..]))
    }
}

fn skip_ws(text: &str) -> &str {
    text.trim_start()
}

fn after_keyword<'a>(line: &'a str, keyword: &str) -> Option<&'a str> {
    let rest = line.strip_prefix(keyword)?;
    let ws_len = rest.len() - rest.trim_start().len();
    if ws_len == 0 {
        return None;
    }
    Some(&rest[ws_len..])
}

fn take_uint(text: &str) -> Option<(f64, &str)> {
    let end = text
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(text.len());
    if end == 0 {
        return None;
    }
    Some((text[..end].parse::<f64>().ok()?, &text[end..]))
}

fn take_signed_int(text: &str) -> Option<(f64, &str)> {
    let (sign, rest) = match text.as_bytes().first() {
        Some(b'-') => (-1.0f64, &text[1..]),
        _ => (1.0f64, text),
    };
    let (value, rest) = take_uint(rest)?;
    Some((sign * value, rest))
}

fn take_float_class(text: &str) -> Option<(&str, &str)> {
    let end = text
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(text.len());
    if end == 0 {
        return None;
    }
    Some((&text[..end], &text[end..]))
}

fn num_or(text: Option<&str>, default: f64) -> ParamValue {
    ParamValue::Number(text.and_then(crate::commands::js_parse_float).unwrap_or(default))
}

fn int_or(text: Option<&str>, default: f64) -> ParamValue {
    ParamValue::Number(text.and_then(js_parse_int).unwrap_or(default))
}

fn js_parse_int(text: &str) -> Option<f64> {
    let rest = text.trim_start();
    let rest = match rest.as_bytes().first() {
        Some(b'-') | Some(b'+') => &rest[1..],
        _ => rest,
    };
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    if end == 0 {
        return None;
    }
    rest[..end].parse::<f64>().ok()
}

fn cmd_of(command_type: CommandType, params: Vec<(&str, ParamValue)>) -> Command {
    Command {
        id: String::new(),
        command_type,
        params: params
            .into_iter()
            .map(|(key, value)| (key.to_string(), value))
            .collect(),
        children: Vec::new(),
    }
}

fn colon_tail(line: &str, keyword: &str) -> Option<String> {
    let rest = after_keyword(line, keyword)?;
    let body = rest.strip_suffix(':')?;
    if body.is_empty() {
        None
    } else {
        Some(body.to_string())
    }
}

fn parse_for(line: &str) -> Option<Command> {
    let rest = after_keyword(line, "for")?;
    let (var, tail) = take_word(rest)?;
    let tail = skip_ws(tail).strip_prefix("in")?;
    let ws_len = tail.len() - tail.trim_start().len();
    if ws_len == 0 {
        return None;
    }
    let tail = skip_ws(&tail[ws_len..]).strip_prefix("range(")?;
    let args = tail.strip_suffix(')')?;
    let (start, rest) = take_uint(args)?;
    let rest = skip_ws(rest);
    if let Some(rest) = rest.strip_prefix(',') {
        let rest = skip_ws(rest);
        let (end_val, rest) = take_uint(rest)?;
        let rest = skip_ws(rest);
        if let Some(rest) = rest.strip_prefix(',') {
            let rest = skip_ws(rest);
            let (step, rest) = take_signed_int(rest)?;
            if rest.is_empty() {
                return Some(cmd_of(
                    CommandType::ForBlock,
                    vec![
                        ("var", ParamValue::Str(var.to_string())),
                        ("start", ParamValue::Number(start)),
                        ("end_val", ParamValue::Number(end_val)),
                        ("step", ParamValue::Number(step)),
                    ],
                ));
            }
            return None;
        }
        if rest.is_empty() {
            return Some(cmd_of(
                CommandType::ForBlock,
                vec![
                    ("var", ParamValue::Str(var.to_string())),
                    ("start", ParamValue::Number(start)),
                    ("end_val", ParamValue::Number(end_val)),
                    ("step", ParamValue::Number(1.0)),
                ],
            ));
        }
        return None;
    }
    if rest.is_empty() {
        return Some(cmd_of(
            CommandType::ForBlock,
            vec![
                ("var", ParamValue::Str(var.to_string())),
                ("start", ParamValue::Number(0.0)),
                ("end_val", ParamValue::Number(start)),
                ("step", ParamValue::Number(1.0)),
            ],
        ));
    }
    None
}

fn assign_tail(line: &str, word: &str) -> Option<String> {
    let tail = line.strip_prefix(word)?;
    let tail = skip_ws(tail);
    let tail = skip_ws(tail.strip_prefix('=')?);
    if tail.is_empty() {
        None
    } else {
        Some(tail.to_string())
    }
}

fn op_assign_tail(line: &str, word: &str) -> Option<(&'static str, String)> {
    let tail = line.strip_prefix(word)?;
    let tail = skip_ws(tail);
    for op in ["+=", "-=", "*=", "/="] {
        if let Some(rest) = tail.strip_prefix(op) {
            let rest = skip_ws(rest);
            if rest.is_empty() {
                return None;
            }
            return Some((op, rest.to_string()));
        }
    }
    None
}

fn parse_line(line: &str) -> Option<Command> {
    let word = leading_word(line);
    if word == "if" {
        if let Some(condition) = colon_tail(line, "if") {
            return Some(cmd_of(
                CommandType::IfBlock,
                vec![("condition", ParamValue::Str(condition))],
            ));
        }
    }
    if word == "elif" {
        if let Some(condition) = colon_tail(line, "elif") {
            return Some(cmd_of(
                CommandType::ElifBlock,
                vec![("condition", ParamValue::Str(condition))],
            ));
        }
    }
    if word == "else" && line == "else:" {
        return Some(cmd_of(CommandType::ElseBlock, vec![]));
    }
    if word == "while" {
        if let Some(condition) = colon_tail(line, "while") {
            return Some(cmd_of(
                CommandType::WhileBlock,
                vec![("condition", ParamValue::Str(condition))],
            ));
        }
    }
    if word == "for" {
        if let Some(command) = parse_for(line) {
            return Some(command);
        }
    }
    if word == "break" && line == "break" {
        return Some(cmd_of(CommandType::BreakCmd, vec![]));
    }
    if word == "def" {
        if let Some(rest) = after_keyword(line, "def") {
            let (name, tail) = take_word(rest)?;
            if tail == "():" {
                return Some(cmd_of(
                    CommandType::FuncDef,
                    vec![("name", ParamValue::Str(name.to_string()))],
                ));
            }
        }
    }
    if !word.is_empty() && line == format!("{word}()") {
        return Some(cmd_of(
            CommandType::FuncCall,
            vec![("name", ParamValue::Str(word.to_string()))],
        ));
    }
    if word == "return" {
        if let Some(value) = after_keyword(line, "return") {
            return Some(cmd_of(
                CommandType::ReturnVal,
                vec![("value", ParamValue::Str(value.to_string()))],
            ));
        }
    }
    if !word.is_empty() && line.contains('=') {
        if let Some(value) = assign_tail(line, word) {
            return Some(cmd_of(
                CommandType::VarDeclare,
                vec![
                    ("name", ParamValue::Str(word.to_string())),
                    ("value", ParamValue::Str(value)),
                ],
            ));
        }
        if let Some((op, value)) = op_assign_tail(line, word) {
            return Some(cmd_of(
                CommandType::SetVar,
                vec![
                    ("name", ParamValue::Str(word.to_string())),
                    ("op", ParamValue::Str(op.to_string())),
                    ("value", ParamValue::Str(value)),
                ],
            ));
        }
    }
    if word == "print" {
        if let Some(inner) = line.strip_prefix("print(") {
            if let Some(value) = inner.strip_suffix(')') {
                if !value.is_empty() {
                    return Some(cmd_of(
                        CommandType::PrintVar,
                        vec![("value", ParamValue::Str(value.to_string()))],
                    ));
                }
            }
        }
    }
    parse_line_trailing(line, word)
}

fn parse_line_trailing(line: &str, word: &str) -> Option<Command> {
    if !word.is_empty() && line.contains(".append(") {
        if let Some(rest) = line.strip_prefix(word) {
            if let Some(inner) = rest.strip_prefix(".append(") {
                if let Some(value) = inner.strip_suffix(')') {
                    if !value.is_empty() {
                        return Some(cmd_of(
                            CommandType::ListAppend,
                            vec![
                                ("name", ParamValue::Str(word.to_string())),
                                ("value", ParamValue::Str(value.to_string())),
                            ],
                        ));
                    }
                }
            }
        }
    }
    if word == "time" {
        if let Some(inner) = line.strip_prefix("time.sleep(") {
            if let Some(value) = inner.strip_suffix(')') {
                if !value.is_empty() {
                    return Some(cmd_of(
                        CommandType::TimeSleep,
                        vec![("dur", num_or(Some(value), 1.0))],
                    ));
                }
            }
        }
    }
    if line.contains("drone.") {
        for (pattern, command_type) in [
            ("drone.takeoff()", CommandType::Takeoff),
            ("drone.land()", CommandType::Land),
            ("drone.emergency_stop()", CommandType::EmergencyStop),
            ("drone.stop_motors()", CommandType::StopMotors),
        ] {
            if line.contains(pattern) {
                return Some(cmd_of(command_type, vec![]));
            }
        }
        if let Some(index) = line.find("drone.hover(") {
            let rest = &line[index + "drone.hover(".len()..];
            if let Some(pos) = rest.rfind(')') {
                if pos > 0 {
                    return Some(cmd_of(
                        CommandType::Hover,
                        vec![("dur", num_or(Some(&rest[..pos]), 1.0))],
                    ));
                }
            }
        }
    }
    if word == "drone" {
        return parse_drone_call(line);
    }
    None
}

fn drone_method(line: &str) -> Option<(&str, &str)> {
    let rest = line.strip_prefix("drone.")?;
    let (name, tail) = take_word(rest)?;
    let tail = skip_ws(tail);
    if tail.starts_with('(') {
        Some((name, &tail[1..]))
    } else {
        None
    }
}

fn quoted_arg(args: &str) -> Option<(&str, &str)> {
    let quote = args.as_bytes().first()?;
    if *quote != b'"' && *quote != b'\'' {
        return None;
    }
    let (word, tail) = take_word(&args[1..])?;
    let tail = tail.as_bytes();
    if !tail.is_empty() && (tail[0] == b'"' || tail[0] == b'\'') {
        Some((word, &args[args.len() - (tail.len() - 1)..]))
    } else {
        None
    }
}

fn first_paren_args<'a>(line: &'a str, prefix: &str) -> Option<&'a str> {
    let rest = line.strip_prefix(prefix)?;
    let end = rest.find(')')?;
    Some(&rest[..end])
}

fn speed_arg(args: &str, default: f64) -> ParamValue {
    ParamValue::Number(match args.find("speed=") {
        Some(index) => take_uint(&args[index + "speed=".len()..])
            .map(|(value, _)| value)
            .unwrap_or(default),
        None => default,
    })
}

fn parse_drone_call(line: &str) -> Option<Command> {
    let (method, args) = drone_method(line)?;
    match method {
        "flip" => {
            let (dir, rest) = quoted_arg(args)?;
            if !rest.starts_with(')') {
                return None;
            }
            Some(cmd_of(CommandType::Flip, vec![("dir", ParamValue::Str(dir.to_string()))]))
        }
        "go" => {
            let (dir, tail) = quoted_arg(args)?;
            let tail = skip_ws(tail);
            let tail = tail.strip_prefix(',')?;
            let tail = skip_ws(tail);
            let (power, tail) = take_uint(tail)?;
            let tail = skip_ws(tail);
            let dur = match tail.strip_prefix(',') {
                Some(rest) => {
                    let (text, rest) = take_float_class(skip_ws(rest))?;
                    if !skip_ws(rest).starts_with(')') {
                        return None;
                    }
                    num_or(Some(text), 1.0)
                }
                None => {
                    if !tail.starts_with(')') {
                        return None;
                    }
                    ParamValue::Number(1.0)
                }
            };
            Some(cmd_of(
                CommandType::Go,
                vec![
                    ("dir", ParamValue::Str(dir.to_string())),
                    ("power", ParamValue::Number(power)),
                    ("dur", dur),
                ],
            ))
        }
        "move_forward" | "move_backward" | "move_left" | "move_right" => {
            let (dist, tail) = take_uint(args)?;
            let tail = skip_ws(tail);
            let speed = match tail.strip_prefix(',') {
                Some(rest) => {
                    let rest = skip_ws(rest);
                    let rest = rest.strip_prefix("speed=")?;
                    let (speed, rest) = take_uint(rest)?;
                    if !skip_ws(rest).starts_with(')') {
                        return None;
                    }
                    speed
                }
                None => {
                    if !tail.starts_with(')') {
                        return None;
                    }
                    50.0
                }
            };
            let command_type = match method {
                "move_forward" => CommandType::MoveForward,
                "move_backward" => CommandType::MoveBackward,
                "move_left" => CommandType::MoveLeft,
                _ => CommandType::MoveRight,
            };
            Some(cmd_of(
                command_type,
                vec![
                    ("dist", ParamValue::Number(dist)),
                    ("speed", ParamValue::Number(speed)),
                ],
            ))
        }
        "turn_left" | "turn_right" => {
            let pos = args.rfind(')')?;
            if pos == 0 {
                return None;
            }
            let command_type = if method == "turn_left" { CommandType::TurnLeft } else { CommandType::TurnRight };
            Some(cmd_of(command_type, vec![("deg", int_or(Some(&args[..pos]), 90.0))]))
        }
        "turn_degree" => parse_turn_degree(args),
        "circle" | "circle_turn" | "square" | "square_turn" | "triangle" | "triangle_turn"
        | "spiral" => {
            let args_c = first_paren_args(line, &format!("drone.{method}("))?;
            let command_type = match method {
                "circle" => CommandType::Circle,
                "circle_turn" => CommandType::CircleTurn,
                "square" => CommandType::Square,
                "square_turn" => CommandType::SquareTurn,
                "triangle" => CommandType::Triangle,
                "triangle_turn" => CommandType::TriangleTurn,
                _ => CommandType::Spiral,
            };
            let default_speed = if matches!(method, "circle" | "circle_turn") {
                75.0
            } else if method == "spiral" {
                50.0
            } else {
                60.0
            };
            let mut params = vec![
                ("speed", speed_arg(args_c, default_speed)),
                ("dir", ParamValue::Str(dir_arg(args_c).to_string())),
            ];
            if matches!(
                command_type,
                CommandType::Square | CommandType::SquareTurn | CommandType::Triangle | CommandType::TriangleTurn
            ) {
                params.push(("secs", secs_arg(args_c, 1.0)));
            }
            Some(cmd_of(command_type, params))
        }
        "sway" => {
            let args_c = first_paren_args(line, "drone.sway(")?;
            let dir = match args_c.find("direction=") {
                Some(index) => {
                    let rest = &args_c[index + "direction=".len()..];
                    let rest = rest
                        .strip_prefix('"')
                        .or_else(|| rest.strip_prefix('\''))
                        .unwrap_or(rest);
                    let end = rest
                        .find(|c: char| c == '"' || c == '\'')
                        .unwrap_or(rest.len());
                    let value = &rest[..end];
                    if value.is_empty() {
                        "forward-back".to_string()
                    } else {
                        value.to_string()
                    }
                }
                None => "forward-back".to_string(),
            };
            Some(cmd_of(
                CommandType::Sway,
                vec![("speed", speed_arg(args_c, 50.0)), ("dir", ParamValue::Str(dir))],
            ))
        }
        "keep_distance" | "avoid_wall" => {
            let prefix = if method == "keep_distance" { "drone.keep_distance(" } else { "drone.avoid_wall(" };
            let args_c = first_paren_args(line, prefix)?;
            let parts: Vec<&str> = args_c.split(',').map(|a| a.trim()).collect();
            let command_type = if method == "keep_distance" { CommandType::KeepDistance } else { CommandType::AvoidWall };
            Some(cmd_of(
                command_type,
                vec![
                    ("dist", num_or(parts.first().copied(), 50.0)),
                    ("speed", int_or(parts.get(1).copied(), 50.0)),
                ],
            ))
        }
        "set_led" => {
            let (color, rest) = quoted_arg(args)?;
            if !rest.starts_with(')') {
                return None;
            }
            Some(cmd_of(CommandType::Led, vec![("color", ParamValue::Str(color.to_string()))]))
        }
        "random_color" => {
            if line == "drone.random_color()" {
                Some(cmd_of(CommandType::RandomLed, vec![]))
            } else {
                None
            }
        }
        "set_buzzer" => {
            let (freq, rest) = take_uint(args)?;
            let rest = skip_ws(rest).strip_prefix(',')?;
            let (dur_text, rest) = take_float_class(skip_ws(rest))?;
            if !skip_ws(rest).starts_with(')') {
                return None;
            }
            Some(cmd_of(
                CommandType::Buzzer,
                vec![("freq", ParamValue::Number(freq)), ("dur", num_or(Some(dur_text), 0.5))],
            ))
        }
        "set_drone_LED" => {
            let (r, rest) = take_uint(args)?;
            let rest = skip_ws(rest).strip_prefix(',')?;
            let (g, rest) = take_uint(skip_ws(rest))?;
            let rest = skip_ws(rest).strip_prefix(',')?;
            let (b, rest) = take_uint(skip_ws(rest))?;
            let rest = skip_ws(rest);
            let brightness = match rest.strip_prefix(',') {
                Some(rest) => {
                    let (value, rest) = take_uint(skip_ws(rest))?;
                    if !skip_ws(rest).starts_with(')') {
                        return None;
                    }
                    value
                }
                None => {
                    if !rest.starts_with(')') {
                        return None;
                    }
                    100.0
                }
            };
            Some(cmd_of(
                CommandType::Led,
                vec![
                    ("r", ParamValue::Number(r)),
                    ("g", ParamValue::Number(g)),
                    ("b", ParamValue::Number(b)),
                    ("brightness", ParamValue::Number(brightness)),
                ],
            ))
        }
        "drone_LED_off" => {
            if line == "drone.drone_LED_off()" {
                Some(cmd_of(CommandType::LedOff, vec![]))
            } else {
                None
            }
        }
        "drone_buzzer" => {
            let comma = args.find(',')?;
            let note = &args[..comma];
            if note.is_empty() {
                return None;
            }
            let (dur, rest) = take_uint(skip_ws(&args[comma + 1..]))?;
            if !skip_ws(rest).starts_with(')') {
                return None;
            }
            Some(cmd_of(
                CommandType::Buzzer,
                vec![("note", ParamValue::Str(note.to_string())), ("dur", ParamValue::Number(dur))],
            ))
        }
        _ => None,
    }
}

fn dir_arg(args: &str) -> &'static str {
    let Some(index) = args.find("direction=") else {
        return "clockwise";
    };
    let after = &args[index + "direction=".len()..];
    if let Some((value, _)) = take_signed_int(after) {
        if value == -1.0 {
            return "counter-clockwise";
        }
        return "clockwise";
    }
    if after.starts_with("counter-clockwise") {
        return "counter-clockwise";
    }
    if after.starts_with("clockwise") {
        return "clockwise";
    }
    "clockwise"
}

fn secs_arg(args: &str, default: f64) -> ParamValue {
    match args.find("seconds=") {
        Some(index) => num_or(
            take_float_class(&args[index + "seconds=".len()..]).map(|(text, _)| text),
            default,
        ),
        None => ParamValue::Number(default),
    }
}

fn parse_turn_degree(args: &str) -> Option<Command> {
    let seg_end = match args.find(',') {
        Some(pos) => pos,
        None => args.rfind(')')?,
    };
    let deg_text = &args[..seg_end];
    if deg_text.is_empty() {
        return None;
    }
    let mut tail = &args[seg_end..];
    let mut timeout;
    let mut p_value;
    let (next, value) = opt_keyword_arg(tail, "timeout");
    tail = next;
    timeout = value;
    let (next, value) = opt_keyword_arg(tail, "p_value");
    tail = next;
    p_value = value;
    if !tail.starts_with(')') {
        return None;
    }
    Some(cmd_of(
        CommandType::TurnDegree,
        vec![
            ("deg", int_or(Some(deg_text), 90.0)),
            ("timeout", num_or(timeout, 3.0)),
            ("p_value", int_or(p_value, 10.0)),
        ],
    ))
}

fn opt_keyword_arg<'a>(tail: &'a str, keyword: &str) -> (&'a str, Option<&'a str>) {
    let t = skip_ws(tail);
    let Some(after_comma) = t.strip_prefix(',') else {
        return (tail, None);
    };
    let t = skip_ws(after_comma);
    let Some(after_kw) = t.strip_prefix(keyword) else {
        return (tail, None);
    };
    let Some(after_eq) = after_kw.strip_prefix('=') else {
        return (tail, None);
    };
    let (value, rest) = match after_eq.find(',') {
        Some(pos) => (&after_eq[..pos], &after_eq[pos..]),
        None => match after_eq.rfind(')') {
            Some(pos) => (&after_eq[..pos], &after_eq[pos..]),
            None => return (tail, None),
        },
    };
    if value.is_empty() {
        return (tail, None);
    }
    (rest, Some(value))
}

fn parse_level(lines: &[(usize, String)], start: usize, end_indent: usize, counter: &mut u32) -> Vec<Command> {
    let mut cmds = Vec::new();
    let mut index = start;
    while index < lines.len() {
        let (indent, content) = (lines[index].0, lines[index].1.as_str());
        if content.is_empty() {
            index += 1;
            continue;
        }
        if indent < end_indent {
            break;
        }
        if indent > end_indent {
            index += 1;
            continue;
        }
        if content.starts_with('#') || content.starts_with("from ") || content.starts_with("import ") {
            index += 1;
            continue;
        }
        let Some(mut command) = parse_line(content) else {
            index += 1;
            continue;
        };
        *counter += 1;
        command.id = format!("p{}", *counter);
        let block = matches!(
            command.command_type,
            CommandType::IfBlock
                | CommandType::ElifBlock
                | CommandType::ElseBlock
                | CommandType::WhileBlock
                | CommandType::ForBlock
                | CommandType::FuncDef
        );
        if block {
            index += 1;
            command.children = parse_level(lines, index, indent + 4, counter);
            while index < lines.len() && lines[index].0 >= indent + 4 {
                index += 1;
            }
            while index < lines.len() && (lines[index].1.is_empty() || lines[index].1.starts_with('#')) {
                index += 1;
            }
            if index < lines.len() && lines[index].0 == indent {
                let next_content = lines[index].1.as_str();
                if matches!(command.command_type, CommandType::IfBlock | CommandType::ElifBlock)
                    && (next_content.starts_with("elif ") || next_content.starts_with("else:"))
                {
                    continue;
                }
            }
        } else {
            index += 1;
        }
        cmds.push(command);
    }
    cmds
}

#[derive(Debug, Clone, Default)]
pub struct ScriptParser;

impl ScriptParser {
    pub fn new() -> ScriptParser {
        ScriptParser
    }

    pub fn parse(&self, source: &str) -> Vec<Command> {
        let lines: Vec<(usize, String)> = source
            .split('\n')
            .map(|raw| {
                let stripped = raw.replace('\t', "    ");
                let indent = stripped.len() - stripped.trim_start().len();
                (indent, stripped.trim().to_string())
            })
            .collect();
        let mut counter = 0;
        parse_level(&lines, 0, 0, &mut counter)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden::GoldenFile;
    use crate::planio::PlanDrone;

    fn load_golden() -> GoldenFile {
        let path = format!("{}/../../tests/golden/golden.json", env!("CARGO_MANIFEST_DIR"));
        GoldenFile::load(&path).expect("golden.json must load")
    }

    fn single_plan(name: &str, commands: &[Command]) -> Plan {
        Plan {
            name: name.to_string(),
            drones: vec![PlanDrone {
                id: "d1".to_string(),
                name: "d1".to_string(),
                color: "#58a6ff".to_string(),
                commands: commands.to_vec(),
                offset: [0.0, 0.0, 0.0],
            }],
            active_drone_id: None,
        }
    }

    #[test]
    fn takeoff_hover_land_code_matches_golden() {
        let golden = load_golden();
        let plan = golden
            .plans
            .iter()
            .find(|plan| plan.name == "takeoff_hover_land")
            .expect("golden plan missing");
        let built = single_plan(&plan.name, &plan.commands);
        assert_eq!(generate_code(&built), plan.code);
    }

    #[test]
    fn swarm_code_matches_golden() {
        let golden = load_golden();
        let built = Plan {
            name: "swarm".to_string(),
            drones: golden.swarm.drones.clone(),
            active_drone_id: None,
        };
        assert_eq!(
            generate_swarm_code(&built),
            golden.swarm.code.clone().unwrap_or_default()
        );
    }
}
