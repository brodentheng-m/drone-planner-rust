use planner_core::codegen::{
    build_dynamic_navigation_plan, generate_autonomous_navigation_script,
    generate_code, generate_dynamic_navigation_code, generate_swarm_code,
    generate_waypoint_avoidance_code,
};
use planner_core::commands::{Command, CommandType, ParamValue};
use planner_core::obstacles::{Boundary, Obstacle, ObstacleSet};
use planner_core::planio::{Plan, PlanDrone};
use planner_core::sensors::{
    default_sensor, evaluate, ColorTable, SensorKind,
};
use planner_core::sim::simulate_commands;
use std::collections::BTreeMap;
use std::io::Write;
use std::process::{Command as ProcessCommand, Stdio};

fn validate_python_ast(code: &str) -> Result<(), String> {
    let mut child = ProcessCommand::new("python3")
        .arg("-c")
        .arg("import ast, sys; ast.parse(sys.stdin.read())")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("failed to spawn python3: {e}"))?;

    if let Some(mut stdin) = child.stdin.take() {
        stdin
            .write_all(code.as_bytes())
            .map_err(|e| format!("failed to write to python3 stdin: {e}"))?;
    }

    let output = child
        .wait_with_output()
        .map_err(|e| format!("failed to wait for python3: {e}"))?;

    if output.status.success() {
        Ok(())
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr);
        Err(format!("AST parse error: {stderr}"))
    }
}

fn assert_no_forbidden_tokens(code: &str) {
    let forbidden = [
        "set_led",
        "random_color",
        "set_buzzer",
        "square_turn",
        "from drone import",
    ];
    for token in forbidden {
        assert!(
            !code.contains(token),
            "generated code must not contain forbidden token: {token}"
        );
    }
}

fn make_cmd(
    id: &str,
    command_type: CommandType,
    params: &[(&str, ParamValue)],
    children: Vec<Command>,
) -> Command {
    let mut map = BTreeMap::new();
    for (k, v) in params {
        map.insert((*k).to_string(), v.clone());
    }
    Command {
        id: id.to_string(),
        command_type,
        params: map,
        children,
    }
}

fn make_drone(id: &str, commands: Vec<Command>) -> PlanDrone {
    PlanDrone {
        id: id.to_string(),
        name: id.to_string(),
        color: "#58a6ff".to_string(),
        commands,
        offset: [0.0, 0.0, 0.0],
    }
}

fn make_plan(drones: Vec<PlanDrone>) -> Plan {
    Plan {
        name: "Test Plan".to_string(),
        drones,
        active_drone_id: Some("d1".to_string()),
    }
}

fn make_obstacle(
    id: &str,
    obstacle_type: &str,
    position: [f64; 3],
    scale: [f64; 3],
    name: &str,
    color: Option<&str>,
) -> Obstacle {
    Obstacle {
        id: id.to_string(),
        obstacle_type: obstacle_type.to_string(),
        position,
        rotation: [0.0, 0.0, 0.0],
        scale,
        name: name.to_string(),
        color: color.map(|c| c.to_string()),
    }
}

fn make_obstacle_set(obstacles: Vec<Obstacle>) -> ObstacleSet {
    ObstacleSet {
        obstacles,
        boundary: Boundary::default(),
    }
}

#[test]
fn tier1_feature_avoid_wall_signature_timeout_first_distance_second() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(70.0)),
            ],
            vec![],
        ),
        make_cmd("c3", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed");
    assert!(
        code.contains("drone.avoid_wall("),
        "code must call drone.avoid_wall: {code}"
    );

    let call_start = code.find("drone.avoid_wall(").expect("found call");
    let after_call = &code[call_start + "drone.avoid_wall(".len()..];
    let call_end = after_call.find(')').expect("found closing paren");
    let args_str = &after_call[..call_end];
    let args: Vec<&str> = args_str.split(',').map(|s| s.trim()).collect();
    assert!(
        args.len() >= 2,
        "avoid_wall requires at least 2 args (timeout, distance): {args_str}"
    );
    let timeout: f64 = args[0].parse().expect("first arg must be timeout");
    let distance: f64 = args[1].parse().expect("second arg must be distance");
    assert!(
        (timeout - 2.0).abs() < 1e-6,
        "timeout must be first argument with value 2: got {timeout}"
    );
    assert!(
        (distance - 70.0).abs() < 1e-6,
        "distance must be second argument with value 70: got {distance}"
    );
}

#[test]
fn tier1_feature_detect_wall_signature_in_cm() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::DetectWall,
            &[
                ("var", ParamValue::Str("wall_detected".to_string())),
                ("dist", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd("c3", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed");
    assert!(
        code.contains("wall_detected = drone.detect_wall("),
        "code must assign wall detection: {code}"
    );
}

#[test]
fn tier1_feature_sensor_range_getter_signatures() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::GetFrontRange,
            &[
                ("var", ParamValue::Str("front_d".to_string())),
                ("unit", ParamValue::Str("cm".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::GetBottomRange,
            &[
                ("var", ParamValue::Str("bottom_d".to_string())),
                ("unit", ParamValue::Str("cm".to_string())),
            ],
            vec![],
        ),
        make_cmd("c4", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed");
    assert!(
        code.contains("front_d = drone.get_front_range(unit=\"cm\")"),
        "front range getter signature mismatch: {code}"
    );
    assert!(
        code.contains("bottom_d = drone.get_bottom_range(unit=\"cm\")"),
        "bottom range getter signature mismatch: {code}"
    );
}

#[test]
fn tier1_feature_sensor_color_getter_signatures() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::GetFrontColor,
            &[
                ("var", ParamValue::Str("fc".to_string())),
                ("kind", ParamValue::Str("name".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::GetBackColor,
            &[
                ("var", ParamValue::Str("bc".to_string())),
                ("kind", ParamValue::Str("name".to_string())),
            ],
            vec![],
        ),
        make_cmd("c4", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed");
    assert!(
        code.contains("fc = drone.get_front_color(kind=\"name\")"),
        "front color getter signature mismatch: {code}"
    );
    assert!(
        code.contains("bc = drone.get_back_color(kind=\"name\")"),
        "back color getter signature mismatch: {code}"
    );
}

#[test]
fn tier1_feature_python_ast_clean_parse_and_token_integrity() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::MoveForward,
            &[
                ("dist", ParamValue::Number(60.0)),
                ("speed", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::GetFrontRange,
            &[
                ("var", ParamValue::Str("d".to_string())),
                ("unit", ParamValue::Str("cm".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "c4",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(40.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c5",
            CommandType::DetectWall,
            &[
                ("var", ParamValue::Str("detected".to_string())),
                ("dist", ParamValue::Number(30.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c6",
            CommandType::Hover,
            &[("dur", ParamValue::Number(1.0))],
            vec![],
        ),
        make_cmd("c7", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("generated flight code must parse cleanly via ast.parse");
    assert_no_forbidden_tokens(&code);
    assert!(
        code.starts_with("from codrone_edu.drone import *"),
        "must import from codrone_edu.drone"
    );
    assert!(code.contains("drone = Drone()"));
    assert!(code.contains("drone.pair()"));
    assert!(code.contains("drone.close()"));
}

#[test]
fn tier1_feature_front_range_decreases_as_drone_approaches_wall() {
    let wall = make_obstacle(
        "wall_target",
        "wall",
        [1.5, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Wall Target",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let reading_far = evaluate(
        &sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_far.hit, "sensor should hit wall from 0.0");
    let dist_far = reading_far.distance_m.expect("distance exists");

    let reading_mid = evaluate(
        &sensor,
        [0.4, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_mid.hit, "sensor should hit wall from 0.4");
    let dist_mid = reading_mid.distance_m.expect("distance exists");

    let reading_close = evaluate(
        &sensor,
        [0.8, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_close.hit, "sensor should hit wall from 0.8");
    let dist_close = reading_close.distance_m.expect("distance exists");

    assert!(
        dist_far > dist_mid,
        "front range must decrease as drone approaches: far {dist_far} vs mid {dist_mid}"
    );
    assert!(
        dist_mid > dist_close,
        "front range must decrease as drone approaches: mid {dist_mid} vs close {dist_close}"
    );

    let delta_far_mid = dist_far - dist_mid;
    assert!(
        (delta_far_mid - 0.4).abs() < 1e-6,
        "delta distance must match delta X (0.4): got {delta_far_mid}"
    );

    let delta_mid_close = dist_mid - dist_close;
    assert!(
        (delta_mid_close - 0.4).abs() < 1e-6,
        "delta distance must match delta X (0.4): got {delta_mid_close}"
    );
}

#[test]
fn tier1_feature_bottom_range_responds_to_altitude_and_surfaces() {
    let desk = make_obstacle(
        "desk_surface",
        "wall",
        [0.0, 0.4, 0.0],
        [1.0, 0.8, 1.0],
        "Desk Surface",
        None,
    );
    let obs_set = make_obstacle_set(vec![desk]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::BottomRange);

    let reading_high = evaluate(
        &sensor,
        [0.0, 1.2, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_high.hit, "bottom sensor should detect desk at y=1.2");
    let dist_high = reading_high.distance_m.expect("distance exists");

    let reading_low = evaluate(
        &sensor,
        [0.0, 0.9, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_low.hit, "bottom sensor should detect desk at y=0.9");
    let dist_low = reading_low.distance_m.expect("distance exists");

    assert!(
        dist_high > dist_low,
        "bottom range must decrease as altitude decreases: high {dist_high} vs low {dist_low}"
    );
    let delta = dist_high - dist_low;
    assert!(
        (delta - 0.3).abs() < 1e-6,
        "delta altitude must match 0.3: got {delta}"
    );
}

#[test]
fn tier1_feature_color_sensor_detects_surface_color() {
    let mut table = ColorTable::new();
    table.insert("landing_pad_red", "red");
    let pad = make_obstacle(
        "landing_pad_red",
        "wall",
        [0.0, 0.05, 0.0],
        [0.6, 0.1, 0.6],
        "Landing Pad Red",
        Some("red"),
    );
    let obs_set = make_obstacle_set(vec![pad]);
    let mut sensor = default_sensor(SensorKind::FrontColor);
    sensor.mount_position = [0.0, 0.0, 0.0];

    let reading = evaluate(
        &sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &table,
    );
    assert!(reading.hit, "color sensor must detect landing pad");
    assert_eq!(
        reading.value, "red",
        "detected color must match assigned pad color"
    );
}

#[test]
fn tier2_boundary_zero_and_minimal_distance() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(1.0)),
                ("dist", ParamValue::Number(0.0)),
            ],
            vec![],
        ),
        make_cmd("c3", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for zero distance");

    let wall = make_obstacle(
        "close_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Close Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let reading_boundary = evaluate(
        &sensor,
        [0.69, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(reading_boundary.hit);
    let d = reading_boundary.distance_m.unwrap();
    assert!(d > 0.0 && d < 0.1, "clearance should be small positive: {d}");
}

#[test]
fn tier2_boundary_distance_exceeding_100cm_and_sensor_max_range() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(3.0)),
                ("dist", ParamValue::Number(150.0)),
            ],
            vec![],
        ),
        make_cmd("c3", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for large distance");
    assert!(code.contains("150"));

    let far_wall = make_obstacle(
        "far_wall",
        "wall",
        [5.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Far Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![far_wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let reading = evaluate(
        &sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(!reading.hit, "out of range obstacle should not trigger hit");
    assert_eq!(reading.distance_m, None);
    assert_eq!(reading.value, "no hit");
}

#[test]
fn tier2_boundary_non_positive_timeout() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(0.0)),
                ("dist", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(-1.0)),
                ("dist", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd("c4", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for non-positive timeout");
}

#[test]
fn tier2_boundary_escapement_drone_facing_away() {
    let wall = make_obstacle(
        "behind_wall",
        "wall",
        [2.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Behind Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let reading = evaluate(
        &sensor,
        [0.0, 0.5, 0.0],
        180.0,
        &obs_set,
        &color_table,
    );
    assert!(!reading.hit, "drone facing 180 deg away must not hit obstacle behind it");
    assert_eq!(reading.distance_m, None);
}

#[test]
fn tier2_boundary_cone_fov_angular_cutoff() {
    let tower = make_obstacle(
        "tower_cone",
        "tower",
        [2.0, 0.75, 0.3],
        [1.0, 1.0, 1.0],
        "Tower Cone",
        None,
    );
    let obs_set = make_obstacle_set(vec![tower]);
    let color_table = ColorTable::new();

    let mut narrow_sensor = default_sensor(SensorKind::FrontRange);
    narrow_sensor.range_m = 10.0;
    narrow_sensor.fov_deg = 0.0;
    narrow_sensor.mount_position = [0.0, 0.0, 0.0];

    let r_narrow = evaluate(
        &narrow_sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(!r_narrow.hit, "narrow 0 deg sensor should miss off-axis target");

    let mut wide_sensor = narrow_sensor.clone();
    wide_sensor.fov_deg = 30.0;

    let r_wide = evaluate(
        &wide_sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(r_wide.hit, "wide 30 deg cone sensor should catch target");
}

#[test]
fn tier2_boundary_extreme_altitude_bottom_range() {
    let desk = make_obstacle(
        "low_desk",
        "wall",
        [0.0, 0.3, 0.0],
        [1.0, 0.6, 1.0],
        "Low Desk",
        None,
    );
    let obs_set = make_obstacle_set(vec![desk]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::BottomRange);

    let reading_extreme = evaluate(
        &sensor,
        [0.0, 10.0, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(!reading_extreme.hit, "extreme altitude 10m must exceed bottom range limit");
    assert_eq!(reading_extreme.distance_m, None);
}

#[test]
fn tier2_boundary_empty_obstacle_set_evaluation() {
    let empty_set = make_obstacle_set(vec![]);
    let color_table = ColorTable::new();

    let front_sensor = default_sensor(SensorKind::FrontRange);
    let r_front = evaluate(
        &front_sensor,
        [1.0, 1.0, 1.0],
        45.0,
        &empty_set,
        &color_table,
    );
    assert!(!r_front.hit);
    assert_eq!(r_front.distance_m, None);

    let bottom_sensor = default_sensor(SensorKind::BottomRange);
    let r_bottom = evaluate(
        &bottom_sensor,
        [1.0, 1.0, 1.0],
        45.0,
        &empty_set,
        &color_table,
    );
    assert!(!r_bottom.hit);
    assert_eq!(r_bottom.distance_m, None);

    let color_sensor = default_sensor(SensorKind::FrontColor);
    let r_color = evaluate(
        &color_sensor,
        [1.0, 1.0, 1.0],
        45.0,
        &empty_set,
        &color_table,
    );
    assert!(!r_color.hit);
    assert_eq!(r_color.value, "Unknown");
}

#[test]
fn tier3_cross_waypoint_navigation_with_dynamic_avoidance() {
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::MoveForward,
            &[
                ("dist", ParamValue::Number(100.0)),
                ("speed", ParamValue::Number(60.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(60.0)),
            ],
            vec![],
        ),
        make_cmd(
            "c4",
            CommandType::TurnRight,
            &[("deg", ParamValue::Number(90.0))],
            vec![],
        ),
        make_cmd(
            "c5",
            CommandType::MoveForward,
            &[
                ("dist", ParamValue::Number(80.0)),
                ("speed", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd("c6", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands.clone())]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for waypoint + avoidance");
    assert_no_forbidden_tokens(&code);

    let sim_result = simulate_commands(&commands, None);
    assert!(!sim_result.positions.is_empty());
    assert!(sim_result.total_duration > 0.0);
}

#[test]
fn tier3_cross_sensor_conditional_branching_ast_validation() {
    let avoid_subcommands = vec![
        make_cmd(
            "sub1",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(40.0)),
            ],
            vec![],
        ),
        make_cmd(
            "sub2",
            CommandType::TurnLeft,
            &[("deg", ParamValue::Number(90.0))],
            vec![],
        ),
    ];
    let else_subcommands = vec![make_cmd(
        "sub3",
        CommandType::MoveForward,
        &[
            ("dist", ParamValue::Number(50.0)),
            ("speed", ParamValue::Number(50.0)),
        ],
        vec![],
    )];

    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::GetFrontRange,
            &[
                ("var", ParamValue::Str("wall_dist".to_string())),
                ("unit", ParamValue::Str("cm".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::IfBlock,
            &[("condition", ParamValue::Str("wall_dist < 50".to_string()))],
            avoid_subcommands,
        ),
        make_cmd("c4", CommandType::ElseBlock, &[], else_subcommands),
        make_cmd("c5", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for conditional sensor logic");
    assert_no_forbidden_tokens(&code);
}

#[test]
fn tier3_cross_simultaneous_front_and_bottom_sensor_tracking() {
    let table = make_obstacle(
        "table",
        "wall",
        [1.0, 0.3, 0.0],
        [1.0, 0.6, 1.0],
        "Table",
        None,
    );
    let back_wall = make_obstacle(
        "back_wall",
        "wall",
        [2.5, 1.0, 0.0],
        [0.5, 2.0, 2.0],
        "Back Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![table, back_wall]);
    let color_table = ColorTable::new();

    let front_sensor = default_sensor(SensorKind::FrontRange);
    let bottom_sensor = default_sensor(SensorKind::BottomRange);

    let front_reading = evaluate(
        &front_sensor,
        [1.0, 0.8, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    let bottom_reading = evaluate(
        &bottom_sensor,
        [1.0, 0.8, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );

    assert!(front_reading.hit, "front sensor should detect back wall");
    assert!(bottom_reading.hit, "bottom sensor should detect table");

    let front_dist = front_reading.distance_m.unwrap();
    let bottom_dist = bottom_reading.distance_m.unwrap();

    assert!(
        front_dist > 1.0,
        "front wall is ~1.5m away: got {front_dist}"
    );
    assert!(
        bottom_dist < 0.5,
        "table surface is 0.2m below: got {bottom_dist}"
    );
}

#[test]
fn tier3_cross_color_guided_waypoint_search_and_landing() {
    let loop_body = vec![
        make_cmd(
            "l1",
            CommandType::MoveForward,
            &[
                ("dist", ParamValue::Number(10.0)),
                ("speed", ParamValue::Number(20.0)),
            ],
            vec![],
        ),
        make_cmd(
            "l2",
            CommandType::GetFrontColor,
            &[
                ("var", ParamValue::Str("c".to_string())),
                ("kind", ParamValue::Str("name".to_string())),
            ],
            vec![],
        ),
    ];
    let commands = vec![
        make_cmd("c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "c2",
            CommandType::VarDeclare,
            &[
                ("name", ParamValue::Str("c".to_string())),
                ("value", ParamValue::Str("\"\"".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "c3",
            CommandType::WhileBlock,
            &[("condition", ParamValue::Str("c != \"red\"".to_string()))],
            loop_body,
        ),
        make_cmd(
            "c4",
            CommandType::Hover,
            &[("dur", ParamValue::Number(1.0))],
            vec![],
        ),
        make_cmd("c5", CommandType::Land, &[], vec![]),
    ];
    let plan = make_plan(vec![make_drone("d1", commands)]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("ast parse must succeed for color guided search loop");
    assert_no_forbidden_tokens(&code);
}

#[test]
fn tier4_realworld_multi_obstacle_classroom_field_flight() {
    let desk1 = make_obstacle(
        "desk1",
        "wall",
        [1.5, 0.75, 0.0],
        [0.91, 0.1, 0.61],
        "Desk 1",
        None,
    );
    let stool = make_obstacle(
        "stool",
        "tower",
        [3.0, 0.45, 0.0],
        [0.3, 0.9, 0.3],
        "Stool",
        None,
    );
    let pad = make_obstacle(
        "target_pad",
        "wall",
        [4.5, 0.05, 0.0],
        [0.5, 0.1, 0.5],
        "Target Pad",
        Some("purple"),
    );
    let obs_set = make_obstacle_set(vec![desk1, stool, pad]);

    let mut color_table = ColorTable::new();
    color_table.insert("target_pad", "purple");

    let front_sensor = default_sensor(SensorKind::FrontRange);
    let bottom_sensor = default_sensor(SensorKind::BottomRange);
    let color_sensor = default_sensor(SensorKind::FrontColor);

    let reading_front = evaluate(
        &front_sensor,
        [2.0, 0.45, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(
        reading_front.hit,
        "front sensor approaching stool at x=3.0 must hit"
    );

    let reading_bottom = evaluate(
        &bottom_sensor,
        [1.5, 0.9, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(
        reading_bottom.hit,
        "bottom sensor over desk at x=1.5 must hit"
    );

    let mut color_sensor_zero = color_sensor.clone();
    color_sensor_zero.mount_position = [0.0, 0.0, 0.0];
    let reading_color = evaluate(
        &color_sensor_zero,
        [4.5, 0.2, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(
        reading_color.hit,
        "color sensor over pad at x=4.5 must detect purple"
    );
    assert_eq!(reading_color.value, "purple");
}

#[test]
fn tier4_realworld_corridor_obstacle_avoidance() {
    let left_wall = make_obstacle(
        "wall_left",
        "wall",
        [2.0, 0.5, 1.0],
        [4.0, 1.0, 0.2],
        "Wall Left",
        None,
    );
    let right_wall = make_obstacle(
        "wall_right",
        "wall",
        [2.0, 0.5, -1.0],
        [4.0, 1.0, 0.2],
        "Wall Right",
        None,
    );
    let end_wall = make_obstacle(
        "wall_end",
        "wall",
        [4.0, 0.5, 0.0],
        [0.2, 1.0, 2.0],
        "Wall End",
        None,
    );
    let obs_set = make_obstacle_set(vec![left_wall, right_wall, end_wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let r_start = evaluate(
        &sensor,
        [0.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(!r_start.hit, "corridor start is clear beyond 1.5m range");

    let r_approached = evaluate(
        &sensor,
        [3.0, 0.5, 0.0],
        0.0,
        &obs_set,
        &color_table,
    );
    assert!(r_approached.hit, "approaching end wall must trigger detection");
    let d_end = r_approached.distance_m.unwrap();
    assert!(
        d_end > 0.7 && d_end < 1.0,
        "end wall distance should be around 0.85m: {d_end}"
    );

    let r_turned = evaluate(
        &sensor,
        [2.0, 0.5, 0.0],
        90.0,
        &obs_set,
        &color_table,
    );
    assert!(r_turned.hit, "drone turned 90 deg must hit left wall");
    let d_turned = r_turned.distance_m.unwrap();
    assert!(
        d_turned > 0.6 && d_turned < 0.9,
        "left wall distance should be around 0.75m: {d_turned}"
    );
}

#[test]
fn tier4_realworld_swarm_multi_drone_avoidance_code_ast() {
    let d1_commands = vec![
        make_cmd("d1_c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "d1_c2",
            CommandType::MoveForward,
            &[
                ("dist", ParamValue::Number(80.0)),
                ("speed", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd(
            "d1_c3",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(50.0)),
            ],
            vec![],
        ),
        make_cmd("d1_c4", CommandType::Land, &[], vec![]),
    ];
    let d2_commands = vec![
        make_cmd("d2_c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "d2_c2",
            CommandType::Hover,
            &[("dur", ParamValue::Number(2.0))],
            vec![],
        ),
        make_cmd(
            "d2_c3",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(2.0)),
                ("dist", ParamValue::Number(60.0)),
            ],
            vec![],
        ),
        make_cmd("d2_c4", CommandType::Land, &[], vec![]),
    ];
    let d3_commands = vec![
        make_cmd("d3_c1", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "d3_c2",
            CommandType::GetFrontRange,
            &[
                ("var", ParamValue::Str("dist".to_string())),
                ("unit", ParamValue::Str("cm".to_string())),
            ],
            vec![],
        ),
        make_cmd(
            "d3_c3",
            CommandType::AvoidWall,
            &[
                ("timeout", ParamValue::Number(3.0)),
                ("dist", ParamValue::Number(45.0)),
            ],
            vec![],
        ),
        make_cmd("d3_c4", CommandType::Land, &[], vec![]),
    ];

    let plan = make_plan(vec![
        make_drone("d1", d1_commands),
        make_drone("d2", d2_commands),
        make_drone("d3", d3_commands),
    ]);
    let swarm_code = generate_swarm_code(&plan);
    validate_python_ast(&swarm_code).expect("swarm generated code must parse cleanly via ast.parse");
    assert_no_forbidden_tokens(&swarm_code);
    assert!(swarm_code.contains("import threading"));
    assert!(swarm_code.contains("drone1.close()"));
    assert!(swarm_code.contains("drone2.close()"));
    assert!(swarm_code.contains("drone3.close()"));
}

#[test]
fn tier5_adversarial_corridor_tunnel_sensor_readings() {
    use planner_core::sim::runtime;

    let left_wall = make_obstacle(
        "wall_left",
        "wall",
        [2.0, 0.5, 0.7],
        [8.0, 1.0, 0.2],
        "Left Corridor Wall",
        None,
    );
    let right_wall = make_obstacle(
        "wall_right",
        "wall",
        [2.0, 0.5, -0.7],
        [8.0, 1.0, 0.2],
        "Right Corridor Wall",
        None,
    );
    let end_wall = make_obstacle(
        "wall_end",
        "wall",
        [4.0, 0.5, 0.0],
        [0.4, 1.0, 1.0],
        "End Corridor Wall",
        None,
    );
    let platform = make_obstacle(
        "platform",
        "wall",
        [2.0, 0.1, 0.0],
        [2.0, 0.2, 0.8],
        "Floor Platform",
        Some("yellow"),
    );

    let obs_set = make_obstacle_set(vec![left_wall, right_wall, end_wall, platform]);
    let color_table = ColorTable::new();
    let front_sensor = default_sensor(SensorKind::FrontRange);

    // 1. Centerline flight along X at altitude Y=0.6m, Z=0.0m
    // Far from end wall (x=0.5): front sensor should not falsely hit side walls 0.5m away
    let probe_start = [0.5, 0.6, 0.0];
    let r_start = evaluate(&front_sensor, probe_start, 0.0, &obs_set, &color_table);
    assert!(!r_start.hit, "corridor side walls 0.5m away must not hit 1.5m cone (distance along 15 deg cone is 1.93m)");
    let sim_front_start = runtime::eval_front_range(probe_start, 0.0, Some(&obs_set));
    assert_eq!(sim_front_start, 100.0, "escapement ceiling must be 100cm");

    // Bottom sensor at start (over bare ground): altitude is 60cm
    let sim_bottom_start = runtime::eval_bottom_range(probe_start, 0.0, 0.6, Some(&obs_set));
    assert_eq!(sim_bottom_start, 60.0, "bottom range must equal ground altitude 60cm");

    // 2. Over elevated platform at x=2.0 (platform top face is at Y=0.2m)
    let probe_plat = [2.0, 0.6, 0.0];
    let sim_front_plat = runtime::eval_front_range(probe_plat, 0.0, Some(&obs_set));
    assert_eq!(sim_front_plat, 100.0, "end wall is 1.85m away, exceeding 1.5m sensor range");
    let sim_bottom_plat = runtime::eval_bottom_range(probe_plat, 0.0, 0.6, Some(&obs_set));
    assert_eq!(sim_bottom_plat, 40.0, "bottom range must decrease to 40cm over 20cm platform (0.6 - 0.2 = 0.4m)");
    let color_plat = runtime::eval_color(probe_plat, 0.0, SensorKind::FrontColor, Some(&obs_set));
    assert_eq!(color_plat, "yellow", "optical sensor over platform must detect yellow");

    // 3. Approaching end wall at x=3.0 (inner face of end wall is at X=3.9m, sensor mount at 3.05m -> distance 0.85m)
    let probe_close = [3.0, 0.6, 0.0];
    let sim_front_close = runtime::eval_front_range(probe_close, 0.0, Some(&obs_set));
    assert!((sim_front_close - 85.0).abs() < 1e-6, "end wall front range must be 85cm: got {sim_front_close}");
    let sim_bottom_close = runtime::eval_bottom_range(probe_close, 0.0, 0.6, Some(&obs_set));
    assert_eq!(sim_bottom_close, 60.0, "bottom range past platform must return to 60cm");

    // 4. Closer to end wall at x=3.4 (distance 3.9 - 3.45 = 0.45m -> 45cm)
    let probe_closer = [3.4, 0.6, 0.0];
    let sim_front_closer = runtime::eval_front_range(probe_closer, 0.0, Some(&obs_set));
    assert!((sim_front_closer - 45.0).abs() < 1e-6, "end wall front range must be 45cm: got {sim_front_closer}");

    // Monotonic decrease verification
    assert!(sim_front_close > sim_front_closer, "front range must decrease monotonically toward end wall");

    // 5. Facing backward in corridor (heading=180 deg)
    let sim_front_back = runtime::eval_front_range(probe_closer, 180.0, Some(&obs_set));
    assert_eq!(sim_front_back, 100.0, "facing away from end wall toward corridor entrance must return 100cm");
}

#[test]
fn tier5_adversarial_corridor_lateral_drift_fov_cone() {
    let left_wall = make_obstacle(
        "wall_left",
        "wall",
        [2.0, 0.5, 0.7],
        [8.0, 1.0, 0.2],
        "Left Corridor Wall",
        None,
    );
    let right_wall = make_obstacle(
        "wall_right",
        "wall",
        [2.0, 0.5, -0.7],
        [8.0, 1.0, 0.2],
        "Right Corridor Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![left_wall, right_wall]);
    let color_table = ColorTable::new();
    let front_sensor = default_sensor(SensorKind::FrontRange);

    // Centerline Z=0.0: side wall distance is 0.5m. Ray along 15 deg cone travels 0.5 / sin(15°) = 1.932m > 1.5m range.
    let r_center = evaluate(&front_sensor, [1.0, 0.5, 0.0], 0.0, &obs_set, &color_table);
    assert!(!r_center.hit, "centerline must have no hit on corridor walls");

    // Mild drift Z=0.05 (distance to left wall = 0.45m): 0.45 / sin(15°) = 1.738m > 1.5m range -> No hit
    let r_drift_small = evaluate(&front_sensor, [1.0, 0.5, 0.05], 0.0, &obs_set, &color_table);
    assert!(!r_drift_small.hit, "mild drift within safe zone must have no hit");

    // Lateral drift Z=0.20 (distance to left wall = 0.30m): 0.30 / sin(15°) = 1.159m <= 1.5m range -> HITS left wall!
    let r_drift_left = evaluate(&front_sensor, [1.0, 0.5, 0.20], 0.0, &obs_set, &color_table);
    assert!(r_drift_left.hit, "lateral drift within 0.388m of wall must trigger 30 deg cone detection");
    let d_left = r_drift_left.distance_m.unwrap();
    assert!((d_left - 1.159).abs() < 0.05, "cone ray hit distance should be ~1.16m: got {d_left}");

    // Symmetric lateral drift Z=-0.20 (distance to right wall = 0.30m) -> HITS right wall!
    let r_drift_right = evaluate(&front_sensor, [1.0, 0.5, -0.20], 0.0, &obs_set, &color_table);
    assert!(r_drift_right.hit, "symmetric drift to right wall must trigger detection");
    let d_right = r_drift_right.distance_m.unwrap();
    assert!((d_right - 1.159).abs() < 0.05, "cone ray hit distance should be ~1.16m: got {d_right}");
}

#[test]
fn tier5_adversarial_obstacle_boundary_glancing_1mm() {
    let wall = make_obstacle(
        "wall_test",
        "wall",
        [2.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Wall Target",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();

    // AABB bounds for wall at position [2.0, 0.5, 0.0], scale [1.0, 1.0, 1.0]:
    // X in [1.75, 2.25] (front face at 1.75)
    // Y in [0.00, 1.00] (min 0.0, max 1.0)
    // Z in [-1.00, 1.00] (min -1.0, max 1.0)
    let mut narrow_sensor = default_sensor(SensorKind::FrontRange);
    narrow_sensor.fov_deg = 0.0;
    narrow_sensor.range_m = 3.0;

    // Front face is at X=1.75. Sensor mount is at X=0.05. Expected hit distance = 1.70m.

    // 1. Glancing Z max boundary (1.000m):
    // 1mm inside (Z=0.999m) -> HITS
    let r_z_in = evaluate(&narrow_sensor, [0.0, 0.5, 0.999], 0.0, &obs_set, &color_table);
    assert!(r_z_in.hit, "1mm inside Z max boundary must hit");
    assert!((r_z_in.distance_m.unwrap() - 1.70).abs() < 1e-6);

    // 1mm outside (Z=1.001m) -> MISSES
    let r_z_out = evaluate(&narrow_sensor, [0.0, 0.5, 1.001], 0.0, &obs_set, &color_table);
    assert!(!r_z_out.hit, "1mm outside Z max boundary must miss");
    assert_eq!(r_z_out.distance_m, None);

    // Exact boundary (Z=1.000m) -> HITS (closed interval [min, max])
    let r_z_exact = evaluate(&narrow_sensor, [0.0, 0.5, 1.000], 0.0, &obs_set, &color_table);
    assert!(r_z_exact.hit, "exact Z max boundary must hit");

    // 2. Glancing Z min boundary (-1.000m):
    // 1mm inside (Z=-0.999m) -> HITS
    let r_zn_in = evaluate(&narrow_sensor, [0.0, 0.5, -0.999], 0.0, &obs_set, &color_table);
    assert!(r_zn_in.hit, "1mm inside Z min boundary must hit");
    assert!((r_zn_in.distance_m.unwrap() - 1.70).abs() < 1e-6);

    // 1mm outside (Z=-1.001m) -> MISSES
    let r_zn_out = evaluate(&narrow_sensor, [0.0, 0.5, -1.001], 0.0, &obs_set, &color_table);
    assert!(!r_zn_out.hit, "1mm outside Z min boundary must miss");

    // Exact boundary (Z=-1.000m) -> HITS
    let r_zn_exact = evaluate(&narrow_sensor, [0.0, 0.5, -1.000], 0.0, &obs_set, &color_table);
    assert!(r_zn_exact.hit, "exact Z min boundary must hit");

    // 3. Glancing Y max boundary (1.000m):
    // 1mm inside (Y=0.999m) -> HITS
    let r_y_in = evaluate(&narrow_sensor, [0.0, 0.999, 0.0], 0.0, &obs_set, &color_table);
    assert!(r_y_in.hit, "1mm inside Y max boundary must hit");

    // 1mm outside (Y=1.001m) -> MISSES
    let r_y_out = evaluate(&narrow_sensor, [0.0, 1.001, 0.0], 0.0, &obs_set, &color_table);
    assert!(!r_y_out.hit, "1mm outside Y max boundary must miss");

    // Exact boundary (Y=1.000m) -> HITS
    let r_y_exact = evaluate(&narrow_sensor, [0.0, 1.000, 0.0], 0.0, &obs_set, &color_table);
    assert!(r_y_exact.hit, "exact Y max boundary must hit");

    // 4. Glancing Y min boundary (0.000m):
    // 1mm inside (Y=0.001m) -> HITS
    let r_yn_in = evaluate(&narrow_sensor, [0.0, 0.001, 0.0], 0.0, &obs_set, &color_table);
    assert!(r_yn_in.hit, "1mm inside Y min boundary must hit");

    // 1mm outside (Y=-0.001m) -> MISSES
    let r_yn_out = evaluate(&narrow_sensor, [0.0, -0.001, 0.0], 0.0, &obs_set, &color_table);
    assert!(!r_yn_out.hit, "1mm outside Y min boundary must miss");
}

#[test]
fn tier5_adversarial_bottom_sensor_glancing_boundary_transition() {
    use planner_core::sim::runtime;

    let platform = make_obstacle(
        "raised_box",
        "wall",
        [1.5, 0.25, 0.0],
        [1.0, 0.5, 1.0],
        "Raised Box",
        None,
    );
    let obs_set = make_obstacle_set(vec![platform]);

    // AABB of platform:
    // X in [1.25, 1.75]
    // Y in [0.00, 0.50] (top surface at Y=0.5m)
    // Z in [-1.00, 1.00]
    // Drone altitude Y=1.0m (distance to ground is 100cm, distance to platform is 50cm)

    // X min boundary transition (1.250m):
    // 1mm before min X (X=1.249m) -> reads ground floor (100cm)
    let b_xmin_out = runtime::eval_bottom_range([1.249, 1.0, 0.0], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_xmin_out, 100.0, "1mm outside min X boundary must read ground floor 100cm");

    // 1mm after min X (X=1.251m) -> reads platform surface (50cm)
    let b_xmin_in = runtime::eval_bottom_range([1.251, 1.0, 0.0], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_xmin_in, 50.0, "1mm inside min X boundary must read platform surface 50cm");

    // X max boundary transition (1.750m):
    // 1mm before max X (X=1.749m) -> reads platform surface (50cm)
    let b_xmax_in = runtime::eval_bottom_range([1.749, 1.0, 0.0], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_xmax_in, 50.0, "1mm inside max X boundary must read platform surface 50cm");

    // 1mm after max X (X=1.751m) -> reads ground floor (100cm)
    let b_xmax_out = runtime::eval_bottom_range([1.751, 1.0, 0.0], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_xmax_out, 100.0, "1mm outside max X boundary must read ground floor 100cm");

    // Z max boundary transition (1.000m):
    let b_zmax_in = runtime::eval_bottom_range([1.5, 1.0, 0.999], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_zmax_in, 50.0, "1mm inside max Z boundary must read platform surface 50cm");
    let b_zmax_out = runtime::eval_bottom_range([1.5, 1.0, 1.001], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_zmax_out, 100.0, "1mm outside max Z boundary must read ground floor 100cm");

    // Z min boundary transition (-1.000m):
    let b_zmin_in = runtime::eval_bottom_range([1.5, 1.0, -0.999], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_zmin_in, 50.0, "1mm inside min Z boundary must read platform surface 50cm");
    let b_zmin_out = runtime::eval_bottom_range([1.5, 1.0, -1.001], 0.0, 1.0, Some(&obs_set));
    assert_eq!(b_zmin_out, 100.0, "1mm outside min Z boundary must read ground floor 100cm");
}

#[test]
fn tier5_adversarial_multiple_obstacles_in_line_of_sight_occlusion() {
    use planner_core::sim::runtime;

    let near = make_obstacle(
        "near_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.4, 1.0, 1.0],
        "Near Wall",
        None,
    );
    let mid = make_obstacle(
        "mid_wall",
        "wall",
        [2.0, 0.5, 0.0],
        [0.4, 1.0, 1.0],
        "Mid Wall",
        None,
    );
    let far = make_obstacle(
        "far_wall",
        "wall",
        [3.0, 0.5, 0.0],
        [0.4, 1.0, 1.0],
        "Far Wall",
        None,
    );

    // Front face distances from drone at X=0.0 (mount at X=0.05):
    // Near: front face X=0.90 -> dist = 0.85m (85.0cm)
    // Mid:  front face X=1.90 -> dist = 1.85m
    // Far:  front face X=2.90 -> dist = 2.85m

    // Test permutation 1: Near, Mid, Far
    let set1 = make_obstacle_set(vec![near.clone(), mid.clone(), far.clone()]);
    let d1 = runtime::eval_front_range([0.0, 0.5, 0.0], 0.0, Some(&set1));
    assert!((d1 - 85.0).abs() < 1e-6, "near obstacle must occlude mid and far: got {d1}");

    // Test permutation 2: Far, Mid, Near (reverse array order)
    let set2 = make_obstacle_set(vec![far.clone(), mid.clone(), near.clone()]);
    let d2 = runtime::eval_front_range([0.0, 0.5, 0.0], 0.0, Some(&set2));
    assert!((d2 - 85.0).abs() < 1e-6, "reverse array order must still return nearest obstacle (85cm): got {d2}");

    // Test permutation 3: Mid, Far, Near
    let set3 = make_obstacle_set(vec![mid.clone(), far.clone(), near.clone()]);
    let d3 = runtime::eval_front_range([0.0, 0.5, 0.0], 0.0, Some(&set3));
    assert!((d3 - 85.0).abs() < 1e-6, "arbitrary order must still return nearest obstacle (85cm): got {d3}");

    // Test removal of Near obstacle: Mid must now be the nearest obstacle
    let set_no_near = make_obstacle_set(vec![mid.clone(), far.clone()]);
    // Drone moves forward to X=1.0 (mount at X=1.05) -> distance to Mid is 1.90 - 1.05 = 0.85m (85cm)
    let d_mid = runtime::eval_front_range([1.0, 0.5, 0.0], 0.0, Some(&set_no_near));
    assert!((d_mid - 85.0).abs() < 1e-6, "mid obstacle must now be detected after near is cleared: got {d_mid}");

    // Test removal of Mid obstacle: Far must now be detected
    let set_far_only = make_obstacle_set(vec![far.clone()]);
    // Drone moves forward to X=2.0 (mount at X=2.05) -> distance to Far is 2.90 - 2.05 = 0.85m (85cm)
    let d_far = runtime::eval_front_range([2.0, 0.5, 0.0], 0.0, Some(&set_far_only));
    assert!((d_far - 85.0).abs() < 1e-6, "far obstacle must now be detected after mid is cleared: got {d_far}");
}

#[test]
fn tier5_adversarial_vertical_color_sensor_occlusion_and_ordering() {
    use planner_core::sim::runtime;

    let red_pad = make_obstacle(
        "pad_red",
        "square",
        [0.0, 0.45, 0.0],
        [0.5, 0.1, 0.5],
        "Red Top Pad",
        Some("red"),
    );
    let green_table = make_obstacle(
        "table_green",
        "square",
        [0.0, 0.25, 0.0],
        [0.8, 0.1, 0.8],
        "Green Mid Table",
        Some("green"),
    );
    let blue_base = make_obstacle(
        "base_blue",
        "square",
        [0.0, 0.05, 0.0],
        [1.0, 0.1, 1.0],
        "Blue Bottom Base",
        Some("blue"),
    );

    // Drone at altitude Y=0.6m looking straight down with optical photodiode
    let probe = [0.0, 0.6, 0.0];

    // Permutation 1: Red, Green, Blue
    let set1 = make_obstacle_set(vec![red_pad.clone(), green_table.clone(), blue_base.clone()]);
    let c1 = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set1));
    assert_eq!(c1, "red", "top red pad must occlude green table and blue base");

    // Permutation 2: Blue, Green, Red (reverse array order)
    let set2 = make_obstacle_set(vec![blue_base.clone(), green_table.clone(), red_pad.clone()]);
    let c2 = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set2));
    assert_eq!(c2, "red", "reverse array order must still return nearest surface (red)");

    // Permutation 3: Green, Blue, Red
    let set3 = make_obstacle_set(vec![green_table.clone(), blue_base.clone(), red_pad.clone()]);
    let c3 = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set3));
    assert_eq!(c3, "red", "mixed array order must still return nearest surface (red)");

    // Remove Red: Green must now be detected
    let set_no_red = make_obstacle_set(vec![green_table.clone(), blue_base.clone()]);
    let c_mid = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set_no_red));
    assert_eq!(c_mid, "green", "green table must be detected when red pad is removed");

    // Remove Green: Blue must now be detected
    let set_blue_only = make_obstacle_set(vec![blue_base.clone()]);
    let c_bot = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set_blue_only));
    assert_eq!(c_bot, "blue", "blue base must be detected when green table is removed");

    // Remove Blue: Unknown returned
    let set_empty = make_obstacle_set(vec![]);
    let c_none = runtime::eval_color(probe, 0.0, SensorKind::FrontColor, Some(&set_empty));
    assert_eq!(c_none, "Unknown", "must return Unknown over empty floor");
}

#[test]
fn tier5_adversarial_negative_space_corridor_and_glancing() {
    use planner_core::sim::runtime;

    let left_wall = make_obstacle(
        "neg_left",
        "wall",
        [-2.0, 0.5, -0.3],
        [4.0, 1.0, 0.2],
        "Negative Left Wall",
        None,
    );
    let right_wall = make_obstacle(
        "neg_right",
        "wall",
        [-2.0, 0.5, -1.7],
        [4.0, 1.0, 0.2],
        "Negative Right Wall",
        None,
    );
    let end_wall = make_obstacle(
        "neg_end",
        "wall",
        [-0.1, 0.5, -1.0],
        [0.4, 1.0, 1.0],
        "Negative End Wall",
        None,
    );

    // Left wall inner face at Z = -0.5
    // Right wall inner face at Z = -1.5
    // Corridor centerline at Z = -1.0 (width 1.0m)
    // End wall inner face at X = -0.2
    let obs_set = make_obstacle_set(vec![left_wall, right_wall, end_wall]);

    // Drone at X = -0.8, Y = 0.5, Z = -1.0 (sensor mount at X = -0.75)
    // Distance to end wall is -0.2 - (-0.75) = 0.55m = 55cm
    let d = runtime::eval_front_range([-0.8, 0.5, -1.0], 0.0, Some(&obs_set));
    assert!((d - 55.0).abs() < 1e-6, "negative coordinate corridor front range must be 55cm: got {d}");

    // Bottom range at altitude 0.5m in negative coordinates
    let b = runtime::eval_bottom_range([-0.8, 0.5, -1.0], 0.0, 0.5, Some(&obs_set));
    assert_eq!(b, 50.0, "bottom range at altitude 0.5m must be 50cm");
}

#[test]
fn tier5_adversarial_diagonal_boundary_glancing() {
    // Corner vertex of square obstacle at [1.0, 0.5, 0.0], scale [1.0, 1.0, 1.0]
    // AABB: X in [0.75, 1.25], Y in [0.0, 1.0], Z in [-1.0, 1.0]
    // Corner at [0.75, 0.5, 1.0]
    let wall = make_obstacle(
        "corner_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Corner Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();

    let mut sensor = default_sensor(SensorKind::FrontRange);
    sensor.fov_deg = 0.0;
    sensor.range_m = 5.0;
    sensor.mount_position = [0.0, 0.0, 0.0];

    // Drone at [0.0, 0.5, 0.0]. Aim at corner [0.75, 0.5, 1.0].
    // Angle to corner: atan2(1.0, 0.75) = 53.13010235415598 deg
    let angle_corner = (1.0_f64).atan2(0.75).to_degrees();

    // Aim 0.05 deg inside corner (towards obstacle body) -> HITS
    let r_inside = evaluate(&sensor, [0.0, 0.5, 0.0], angle_corner - 0.05, &obs_set, &color_table);
    assert!(r_inside.hit, "aiming slightly inside corner must hit obstacle");

    // Aim 0.05 deg outside corner (skimming past) -> MISSES
    let r_outside = evaluate(&sensor, [0.0, 0.5, 0.0], angle_corner + 0.05, &obs_set, &color_table);
    assert!(!r_outside.hit, "aiming slightly outside corner must miss obstacle");
}

#[test]
fn tier5_adversarial_contact_zero_distance() {
    use planner_core::sim::runtime;

    let wall = make_obstacle(
        "touch_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [1.0, 1.0, 1.0],
        "Touch Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);

    // Front face of wall is at X = 0.75m.
    // Front sensor mount is at X = 0.05m ahead of drone position.
    // Place drone at X = 0.70m so sensor mount is exactly at X = 0.75m (touching front face).
    let d_touch = runtime::eval_front_range([0.70, 0.5, 0.0], 0.0, Some(&obs_set));
    assert_eq!(d_touch, 0.0, "sensor mount in contact with obstacle face must return exactly 0.0cm");

    // Place drone slightly inside obstacle (X = 0.71m, sensor mount at 0.76m inside [0.75, 1.25])
    let d_inside = runtime::eval_front_range([0.71, 0.5, 0.0], 0.0, Some(&obs_set));
    assert_eq!(d_inside, 0.0, "sensor mount inside obstacle volume must return 0.0cm");

    // Bottom sensor contact with ground: drone at altitude Z=0.0m
    let b_ground = runtime::eval_bottom_range([0.0, 0.0, 0.0], 0.0, 0.0, Some(&obs_set));
    assert_eq!(b_ground, 0.0, "bottom sensor at ground level must return 0.0cm");
}

#[test]
fn tier5_adversarial_pillar_in_front_of_wall_cone_occlusion() {
    let pillar = make_obstacle(
        "narrow_pillar",
        "tower",
        [1.0, 0.5, 0.0],
        [0.67, 1.0, 0.67], // base tower is 0.3x1.5x0.3 -> half-width 0.1m, depth 0.2m
        "Narrow Pillar",
        None,
    );
    let back_wall = make_obstacle(
        "back_wall",
        "wall",
        [1.8, 0.5, 0.0],
        [1.0, 1.0, 4.0], // wide wall spanning Z in [-4.0, 4.0], front face at X = 1.8 - 0.25 = 1.55m
        "Wide Back Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![pillar, back_wall]);
    let color_table = ColorTable::new();
    let front_sensor = default_sensor(SensorKind::FrontRange);

    // Front sensor has 30 deg FOV cone (center ray + 8 outer rays).
    // Pillar front face is at X = 0.90m. Distance from drone mount (0.05m) is 0.85m.
    // Back wall front face is at X = 1.55m. Distance from drone mount is 1.50m.
    // Outer cone rays at 15 deg spread out to Z = 0.85 * tan(15°) = 0.228m > pillar half-width (0.1m),
    // so outer rays hit the back wall at ~1.5m, while center ray hits the pillar at 0.85m.
    // The sensor must report the minimum distance (0.85m from pillar), proving the pillar occludes the wall!
    let r_aligned = evaluate(&front_sensor, [0.0, 0.5, 0.0], 0.0, &obs_set, &color_table);
    assert!(r_aligned.hit, "sensor must detect hit");
    let d_aligned = r_aligned.distance_m.unwrap();
    assert!((d_aligned - 0.8495).abs() < 1e-4, "nearest hit must be pillar at 0.8495m despite wide wall in cone: got {d_aligned}");

    // When drone is shifted to Z = 0.35m:
    // Center ray misses pillar, but the 15 deg cone ray clips the side face of the pillar at ~0.964m!
    let r_clip = evaluate(&front_sensor, [0.0, 0.5, 0.35], 0.0, &obs_set, &color_table);
    assert!(r_clip.hit, "15 deg cone ray must clip pillar side face");
    let d_clip = r_clip.distance_m.unwrap();
    assert!((d_clip - 0.964).abs() < 0.005, "cone ray must clip side face at ~0.964m: got {d_clip}");

    // Now steer drone further laterally to Z = 0.50m (completely clearing the 15 deg cone for the pillar).
    // All rays in the cone now clear the pillar and strike the wide back wall at 1.50m!
    let r_clear = evaluate(&front_sensor, [0.0, 0.5, 0.50], 0.0, &obs_set, &color_table);
    assert!(r_clear.hit, "sensor must detect back wall when clear of pillar");
    let d_clear = r_clear.distance_m.unwrap();
    assert!((d_clear - 1.50).abs() < 1e-4, "hit must now be back wall at 1.50m: got {d_clear}");
}

#[test]
fn tier1_feature_autonomous_point_a_to_point_b_navigation_ast_and_signatures() {
    let code = generate_autonomous_navigation_script(200.0, 150.0, 80.0);
    validate_python_ast(&code).expect("ast parse must succeed for autonomous navigation");
    assert_no_forbidden_tokens(&code);

    assert!(code.contains("drone.get_pos_x(\"cm\")"));
    assert!(code.contains("drone.get_pos_y(\"cm\")"));
    assert!(code.contains("drone.get_pos_z(\"cm\")"));
    assert!(code.contains("drone.turn_degree("));
    assert!(code.contains("drone.detect_wall(50)"));
    assert!(code.contains("drone.get_front_range(\"cm\")"));
    assert!(code.contains("drone.avoid_wall(1.5, 40)"));
    assert!(code.contains("drone.move_right(35, speed=0.5)"));
    assert!(code.contains("drone.move_left(35, speed=0.5)"));
    assert!(code.contains("drone.move_forward("));
    assert!(code.contains("drone.takeoff()"));
    assert!(code.contains("drone.land()"));
    assert!(code.contains("drone.close()"));

    let dyn_code = generate_dynamic_navigation_code([300.0, 250.0, 100.0]);
    validate_python_ast(&dyn_code).expect("dynamic navigation script ast parse");
    assert!(dyn_code.contains("navigate_to_waypoint(drone, 300, 250, 100)"));
}

#[test]
fn tier1_feature_multi_waypoint_dynamic_avoidance_script_ast() {
    let waypoints = [[100.0, 100.0, 80.0], [200.0, 150.0, 80.0], [300.0, 50.0, 80.0]];
    let code = generate_waypoint_avoidance_code(&waypoints);
    validate_python_ast(&code).expect("multi-waypoint script ast parse");
    assert_no_forbidden_tokens(&code);
    assert!(code.contains("navigate_to_waypoint(drone, 100, 100, 80)"));
    assert!(code.contains("navigate_to_waypoint(drone, 200, 150, 80)"));
    assert!(code.contains("navigate_to_waypoint(drone, 300, 50, 80)"));
}

#[test]
fn tier4_realworld_dynamic_navigation_plan_simulation() {
    let wall = make_obstacle(
        "mid_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.2, 1.0, 1.0],
        "Mid Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let plan = build_dynamic_navigation_plan(&[[150.0, 0.0, 80.0]]);
    let code = generate_code(&plan);
    validate_python_ast(&code).expect("plan code ast parse");
    assert_no_forbidden_tokens(&code);

    let sim_result = simulate_commands(&plan.drones[0].commands, Some(&obs_set));
    assert!(!sim_result.positions.is_empty());
    assert!(sim_result.total_duration > 0.0);
    assert!(sim_result.collisions.is_empty());
}
