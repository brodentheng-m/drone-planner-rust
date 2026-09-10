pub mod controlflow;
pub mod drive;
pub mod moves;
pub mod runtime;
pub mod shapes;

use crate::commands::{Command, CommandType, ParamValue, js_num};
use crate::golden::{LedValue, Point, SimResult};
use crate::obstacles::ObstacleSet;
use crate::planio::{Plan, PlanDrone};
use crate::sim::controlflow::{ControlContext, MAX_ITER, MAX_WHILE_LOOPS};
use crate::sim::drive::SimState;
use crate::sim::runtime::{RuntimeState, VarValue};
use std::collections::BTreeMap;

pub type OnCommandStart<'a> = &'a mut dyn FnMut(&Command, usize, &SimState);

fn pd(value: Option<f64>, default: f64) -> f64 {
    value.filter(|v| *v != 0.0).unwrap_or(default)
}

fn ps<'a>(value: Option<&'a str>, default: &'a str) -> &'a str {
    match value {
        Some(text) if !text.is_empty() => text,
        _ => default,
    }
}

fn parse_int_str(text: &str) -> f64 {
    let trimmed = text.trim_start();
    let bytes = trimmed.as_bytes();
    let mut index = 0usize;
    let mut sign = 1.0;
    if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
        if bytes[index] == b'-' {
            sign = -1.0;
        }
        index += 1;
    }
    let head = &trimmed[index.min(trimmed.len())..];
    if head.starts_with("0x") || head.starts_with("0X") {
        let mut digits = String::new();
        let mut cursor = index + 2;
        while cursor < bytes.len() && bytes[cursor].is_ascii_hexdigit() {
            digits.push(bytes[cursor] as char);
            cursor += 1;
        }
        if digits.is_empty() {
            return f64::NAN;
        }
        return sign * i64::from_str_radix(&digits, 16).unwrap_or(0) as f64;
    }
    let start = index;
    while index < bytes.len() && bytes[index].is_ascii_digit() {
        index += 1;
    }
    if index == start {
        return f64::NAN;
    }
    sign * trimmed[start..index].parse::<f64>().unwrap_or(f64::NAN)
}

fn parse_int_param(value: Option<&ParamValue>) -> f64 {
    match value {
        Some(ParamValue::Number(n)) => n.trunc(),
        Some(ParamValue::Str(text)) => parse_int_str(text),
        _ => f64::NAN,
    }
}

fn pi(command: &Command, key: &str, default: f64) -> f64 {
    let parsed = parse_int_param(command.params.get(key));
    if parsed.is_nan() || parsed == 0.0 {
        default
    } else {
        parsed
    }
}

fn param_text(command: &Command, key: &str) -> String {
    match command.params.get(key) {
        Some(ParamValue::Number(n)) => js_num(*n),
        Some(ParamValue::Str(text)) => text.clone(),
        Some(ParamValue::Bool(flag)) => flag.to_string(),
        None => String::new(),
    }
}

fn eval_param(command: &Command, key: &str, vars: &BTreeMap<String, VarValue>) -> f64 {
    match command.params.get(key) {
        Some(ParamValue::Number(n)) => {
            if vars.is_empty() {
                *n
            } else {
                0.0
            }
        }
        Some(ParamValue::Str(text)) => runtime::eval_expr(text, vars),
        Some(ParamValue::Bool(flag)) => {
            if vars.is_empty() && *flag {
                1.0
            } else {
                0.0
            }
        }
        None => 0.0,
    }
}

fn is_truthy(value: f64) -> bool {
    value != 0.0 && !value.is_nan()
}

pub fn simulate_commands(commands: &[Command], obstacles: Option<&ObstacleSet>) -> SimResult {
    let drone = PlanDrone {
        id: "d1".to_string(),
        name: "d1".to_string(),
        color: "#58a6ff".to_string(),
        commands: commands.to_vec(),
        offset: [0.0, 0.0, 0.0],
    };
    simulate_single_drone(&drone, obstacles, None)
}

pub fn simulate_single_drone<'a, 'b>(
    drone: &PlanDrone,
    obstacles: Option<&ObstacleSet>,
    on_start: Option<&'a mut (dyn FnMut(&Command, usize, &SimState) + 'b)>,
) -> SimResult
where
    'b: 'a,
{
    let offset = drone.offset;
    let mut state = SimState::new();
    state.drone_id = drone.id.clone();
    state.points.push(Point {
        x: 2.0 * offset[0],
        y: 2.0 * offset[2],
        z: 2.0 * offset[1],
        heading: 0.0,
        pitch: 0.0,
        roll: 0.0,
        speed: 0.0,
        energy_used: 0.0,
        battery_percent: 100.0,
        turn_radius_m: None,
        led: LedValue::Name("off".to_string()),
    });
    let mut rt = RuntimeState::new();
    let mut ctx = ControlContext::new();
    process_commands(
        &drone.commands,
        &mut state,
        &mut rt,
        &mut ctx,
        obstacles,
        on_start,
    );
    let shift_x = offset[0];
    let shift_y = offset[2];
    let shift_z = offset[1];
    if shift_x != 0.0 || shift_y != 0.0 || shift_z != 0.0 {
        for point in state.points.iter_mut().skip(1) {
            point.x += shift_x;
            point.y += shift_y;
            point.z += shift_z;
        }
    }
    SimResult {
        positions: state.points,
        total_duration: state.total_duration,
        collisions: state.collisions,
    }
}

pub fn simulate_swarm(plan: &Plan, obstacles: Option<&ObstacleSet>) -> BTreeMap<String, SimResult> {
    simulate_swarm_with(plan, obstacles, None)
}

pub fn simulate_swarm_with(
    plan: &Plan,
    obstacles: Option<&ObstacleSet>,
    on_start: Option<OnCommandStart>,
) -> BTreeMap<String, SimResult> {
    let mut results = BTreeMap::new();
    let mut max_duration = 0.0f64;
    let mut on_start = on_start;
    for drone in &plan.drones {
        let result = simulate_single_drone(drone, obstacles, on_start.as_deref_mut());
        if result.total_duration > max_duration {
            max_duration = result.total_duration;
        }
        results.insert(drone.id.clone(), result);
    }
    for result in results.values_mut() {
        result.total_duration = max_duration;
    }
    results
}

fn process_commands<'a, 'b>(
    commands: &[Command],
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
    obstacles: Option<&ObstacleSet>,
    on_start: Option<&'a mut (dyn FnMut(&Command, usize, &SimState) + 'b)>,
) where
    'b: 'a,
{
    let mut iter = 0u64;
    let mut callback = on_start;
    for command in commands {
        if iter > MAX_ITER {
            break;
        }
        iter += 1;
        if let Some(on_command_start) = callback.as_deref_mut() {
            on_command_start(command, (iter - 1) as usize, state);
        }
        let dur = dispatch(command, state, runtime, ctx, obstacles);
        state.total_duration += dur;
    }
}

fn dispatch(
    command: &Command,
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
    obstacles: Option<&ObstacleSet>,
) -> f64 {
    match command.command_type {
        CommandType::Takeoff => drive::takeoff(state),
        CommandType::Land => drive::land(state),
        CommandType::Hover => {
            let dur = pd(command.param_f64("dur"), 1.0);
            drive::hover(state, dur)
        }
        CommandType::Flip => {
            let dir = ps(command.param_str("dir"), "back");
            drive::flip(state, dir)
        }
        CommandType::Go => {
            let dir = ps(command.param_str("dir"), "forward");
            let power = pd(command.param_f64("power"), 50.0);
            let dur = pd(command.param_f64("dur"), 1.0);
            drive::go(state, obstacles, dir, power, dur)
        }
        CommandType::MoveForward => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            moves::move_forward(state, dist, speed)
        }
        CommandType::MoveBackward => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            moves::move_backward(state, dist, speed)
        }
        CommandType::MoveLeft => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            moves::move_left(state, dist, speed)
        }
        CommandType::MoveRight => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            moves::move_right(state, dist, speed)
        }
        CommandType::TurnLeft => {
            let deg = pd(command.param_f64("deg"), 90.0);
            moves::turn_left(state, deg)
        }
        CommandType::TurnRight => {
            let deg = pd(command.param_f64("deg"), 90.0);
            moves::turn_right(state, deg)
        }
        CommandType::TurnDegree => {
            let deg = pd(command.param_f64("deg"), 90.0);
            let timeout = pd(command.param_f64("timeout"), 3.0);
            let p_value = pd(command.param_f64("p_value"), 10.0);
            moves::turn_degree(state, deg, timeout, p_value)
        }
        CommandType::Circle | CommandType::CircleTurn => {
            let speed = pd(command.param_f64("speed"), 75.0);
            let dir = ps(command.param_str("dir"), "clockwise");
            shapes::circle(state, speed, dir)
        }
        CommandType::Square | CommandType::SquareTurn => {
            let speed = pd(command.param_f64("speed"), 60.0);
            let secs = pd(command.param_f64("secs"), 1.0);
            let dir = ps(command.param_str("dir"), "clockwise");
            shapes::square(state, speed, secs, dir)
        }
        CommandType::Triangle | CommandType::TriangleTurn => {
            let speed = pd(command.param_f64("speed"), 60.0);
            let secs = pd(command.param_f64("secs"), 1.0);
            let dir = ps(command.param_str("dir"), "clockwise");
            shapes::triangle(state, speed, secs, dir)
        }
        CommandType::Spiral => {
            let speed = pd(command.param_f64("speed"), 50.0);
            let dir = ps(command.param_str("dir"), "clockwise");
            shapes::spiral(state, speed, dir)
        }
        CommandType::Sway => {
            let speed = pd(command.param_f64("speed"), 50.0);
            let dir = ps(command.param_str("dir"), "forward-back");
            shapes::sway(state, speed, dir)
        }
        CommandType::KeepDistance => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            shapes::keep_distance(state, speed, dist)
        }
        CommandType::AvoidWall => {
            let dist = pd(command.param_f64("dist"), 50.0);
            let speed = pd(command.param_f64("speed"), 50.0);
            shapes::avoid_wall(state, speed, dist)
        }
        CommandType::DetectWall => {
            let var_name = ps(command.param_str("var"), "detected");
            shapes::detect_wall(state, var_name, runtime)
        }
        CommandType::Led => {
            let r = pi(command, "r", 0.0).clamp(0.0, 255.0);
            let g = pi(command, "g", 255.0).clamp(0.0, 255.0);
            let b = pi(command, "b", 0.0).clamp(0.0, 255.0);
            let brightness = pi(command, "brightness", 100.0).clamp(0.0, 255.0);
            state.led_color = LedValue::Rgb {
                r,
                g,
                b,
                brightness,
            };
            let led = state.led_color.clone();
            state.push_point(state.heading, 0.0, 0.0, Some(led));
            0.1
        }
        CommandType::LedOff => {
            state.led_color = LedValue::Rgb {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                brightness: 0.0,
            };
            let led = state.led_color.clone();
            state.push_point(state.heading, 0.0, 0.0, Some(led));
            0.1
        }
        CommandType::Buzzer => pd(command.param_f64("dur"), 500.0) / 1000.0,
        CommandType::VarDeclare => {
            let name = ps(command.param_str("name"), "x");
            let value = eval_param(command, "value", &runtime.vars);
            runtime.vars.insert(name.to_string(), VarValue::Num(value));
            0.0
        }
        CommandType::SetVar => {
            let name = ps(command.param_str("name"), "x");
            let op = ps(command.param_str("op"), "=");
            let value = eval_param(command, "value", &runtime.vars);
            let current = runtime
                .vars
                .get(name)
                .cloned()
                .unwrap_or(VarValue::Num(0.0));
            let next = if op == "=" || op == "+=" || op == "-=" || op == "*=" || op == "/=" {
                runtime::compound_set(&current, op, value)
            } else {
                current
            };
            runtime.vars.insert(name.to_string(), next);
            0.0
        }
        CommandType::PrintVar => 0.0,
        CommandType::IfBlock => {
            if is_truthy(eval_param(command, "condition", &runtime.vars)) {
                process_commands(&command.children, state, runtime, ctx, None, None);
            }
            0.0
        }
        CommandType::ElifBlock => {
            if is_truthy(eval_param(command, "condition", &runtime.vars)) {
                process_commands(&command.children, state, runtime, ctx, None, None);
            }
            0.0
        }
        CommandType::ElseBlock => {
            process_commands(&command.children, state, runtime, ctx, None, None);
            0.0
        }
        CommandType::EndBlock => 0.0,
        CommandType::WhileBlock => {
            let mut loops = 0u64;
            while is_truthy(eval_param(command, "condition", &runtime.vars))
                && loops < MAX_WHILE_LOOPS
            {
                process_commands(&command.children, state, runtime, ctx, None, None);
                loops += 1;
            }
            0.0
        }
        CommandType::ForBlock => {
            let var_name = ps(command.param_str("var"), "i");
            let start = pd(command.param_f64("start"), 0.0) as i64;
            let end_val = pd(command.param_f64("end_val"), 5.0) as i64;
            let step = pd(command.param_f64("step"), 1.0) as i64;
            if step != 0 {
                let mut i = start;
                while (step > 0 && i < end_val) || (step < 0 && i > end_val) {
                    runtime
                        .vars
                        .insert(var_name.to_string(), VarValue::Num(i as f64));
                    process_commands(&command.children, state, runtime, ctx, None, None);
                    i += step;
                }
            }
            0.0
        }
        CommandType::BreakCmd => 0.0,
        CommandType::ReturnVal => 0.0,
        CommandType::EmergencyStop | CommandType::StopMotors => {
            for _ in 0..10 {
                state.push_point(state.heading, 0.0, 0.0, None);
            }
            state.flying = false;
            0.5
        }
        CommandType::GetBattery => {
            let var_name = ps(command.param_str("var"), "battery");
            runtime
                .vars
                .insert(var_name.to_string(), VarValue::Num(runtime::get_battery()));
            0.0
        }
        CommandType::GetHeight => {
            let var_name = ps(command.param_str("var"), "height");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Num(runtime::get_height(state.z)),
            );
            0.0
        }
        CommandType::GetFrontRange => {
            let var_name = ps(command.param_str("var"), "front_range");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Num(runtime::get_front_range()),
            );
            0.0
        }
        CommandType::GetBottomRange => {
            let var_name = ps(command.param_str("var"), "bottom_range");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Num(runtime::get_bottom_range(state.z)),
            );
            0.0
        }
        CommandType::GetFrontColor => {
            let var_name = ps(command.param_str("var"), "front_color");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Str(runtime::get_front_color().to_string()),
            );
            0.0
        }
        CommandType::GetBackColor => {
            let var_name = ps(command.param_str("var"), "back_color");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Str(runtime::get_back_color().to_string()),
            );
            0.0
        }
        CommandType::GetTemperature => {
            let var_name = ps(command.param_str("var"), "temperature");
            runtime.vars.insert(
                var_name.to_string(),
                VarValue::Num(runtime::get_temperature()),
            );
            0.0
        }
        CommandType::FuncDef => {
            let name = ps(command.param_str("name"), "my_func");
            controlflow::define_function(ctx, name, command.children.clone());
            0.0
        }
        CommandType::FuncCall => {
            let name = ps(command.param_str("name"), "my_func");
            if let Some(body) = controlflow::call_function(ctx, name).cloned() {
                process_commands(&body, state, runtime, ctx, None, None);
            }
            0.0
        }
        CommandType::ListDeclare => {
            let name = ps(command.param_str("name"), "my_list");
            let text = param_text(command, "values");
            let items: Vec<f64> = text
                .split(',')
                .map(|item| runtime::eval_expr(item.trim(), &runtime.vars))
                .collect();
            runtime.vars.insert(name.to_string(), VarValue::List(items));
            0.0
        }
        CommandType::ListAppend => {
            let name = ps(command.param_str("name"), "my_list");
            let value = eval_param(command, "value", &runtime.vars);
            runtime::list_append(runtime, name, value);
            0.0
        }
        CommandType::ListGet => {
            let list_name = ps(command.param_str("list_name"), "my_list");
            let index = pd(command.param_f64("index"), 0.0) as i64;
            let var_name = ps(command.param_str("var"), "val");
            let value = runtime::list_get(runtime, list_name, index).unwrap_or(0.0);
            runtime
                .vars
                .insert(var_name.to_string(), VarValue::Num(value));
            0.0
        }
        CommandType::UserInput => {
            let var_name = ps(command.param_str("var"), "user_val");
            runtime
                .vars
                .insert(var_name.to_string(), VarValue::Num(0.0));
            0.0
        }
        CommandType::TimerStart => {
            let name = ps(command.param_str("name"), "t");
            runtime::timer_start(runtime, name);
            0.0
        }
        CommandType::TimerElapsed => {
            let name = ps(command.param_str("name"), "t");
            let var_name = ps(command.param_str("var"), "elapsed");
            let elapsed = runtime::timer_elapsed(runtime, name);
            runtime
                .vars
                .insert(var_name.to_string(), VarValue::Num(elapsed));
            0.0
        }
        CommandType::TimeSleep | CommandType::DroneSleep => pd(command.param_f64("dur"), 1.0),
        CommandType::RandomLed | CommandType::GetDistance => 0.0,
    }
}
