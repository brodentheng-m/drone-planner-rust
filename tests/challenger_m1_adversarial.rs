use planner_core::commands::{Command, CommandType, ParamValue};
use planner_core::obstacles::{Boundary, Obstacle, ObstacleSet};
use planner_core::sensors::{
    default_sensor, evaluate, ColorTable, SensorKind,
};
use planner_core::sim::runtime::{eval_bottom_range, eval_front_range};
use planner_core::sim::simulate_commands;
use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

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

fn make_obstacle(
    id: &str,
    obstacle_type: &str,
    position: [f64; 3],
    rotation: [f64; 3],
    scale: [f64; 3],
    name: &str,
    color: Option<&str>,
) -> Obstacle {
    Obstacle {
        id: id.to_string(),
        obstacle_type: obstacle_type.to_string(),
        position,
        rotation,
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

// =========================================================================
// Group 1: Slanted / Angled Walls & FOV Cone Math
// =========================================================================

#[test]
fn test_angled_heading_30_deg_wall_intersection() {
    // Wall located at x = 1.0, y = 0.5, z = 0.0 with width 0.5 (min_x = 0.75, max_x = 1.25)
    let wall = make_obstacle(
        "wall_30",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Wall 30",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();

    let deg30 = 30.0_f64;
    let rad30 = deg30.to_radians();
    let probe = [0.0, 0.5, 0.0];
    let mount_x = probe[0] + 0.05 * rad30.cos();

    // 1. Center ray only (pencil beam, fov = 0.0):
    // t_center = (0.75 - mount_x) / cos(30°)
    let mut sensor_pencil = default_sensor(SensorKind::FrontRange);
    sensor_pencil.fov_deg = 0.0;
    let reading_pencil = evaluate(&sensor_pencil, probe, deg30, &obs_set, &color_table);
    assert!(reading_pencil.hit);
    let expected_center_t = (0.75 - mount_x) / rad30.cos();
    let dist_pencil = reading_pencil.distance_m.unwrap();
    assert!(
        (dist_pencil - expected_center_t).abs() < 1e-4,
        "pencil beam center ray must hit at theoretical {expected_center_t}, got {dist_pencil}"
    );

    // 2. 30° FOV cone (hardware CoDrone EDU sensor):
    // The cone has rays at azimuth 90°/270° offset by 15° half-angle, which projects
    // to 30° - 15° = 15° in the ground plane.
    // The closest hit is at t_cone = (0.75 - mount_x) / cos(15°).
    let sensor_cone = default_sensor(SensorKind::FrontRange);
    let reading_cone = evaluate(&sensor_cone, probe, deg30, &obs_set, &color_table);
    assert!(reading_cone.hit);
    let rad15 = 15.0_f64.to_radians();
    let expected_cone_t = (0.75 - mount_x) / rad15.cos();
    let dist_cone = reading_cone.distance_m.unwrap();
    assert!(
        (dist_cone - expected_cone_t).abs() < 1e-4,
        "30° FOV cone ray at 15° must hit at theoretical {expected_cone_t}, got {dist_cone}"
    );

    let runtime_val = eval_front_range(probe, deg30, Some(&obs_set));
    let expected_cone_cm = (expected_cone_t * 100.0).min(100.0);
    assert!(
        (runtime_val - expected_cone_cm).abs() < 1e-2,
        "runtime cm {runtime_val} must match {expected_cone_cm}"
    );

    // 3. Monotonic approach test at 30 deg heading
    let dist_far = eval_front_range([0.0, 0.5, 0.0], deg30, Some(&obs_set));
    let dist_mid = eval_front_range([0.2, 0.5, 0.0], deg30, Some(&obs_set));
    let dist_near = eval_front_range([0.4, 0.5, 0.0], deg30, Some(&obs_set));

    assert!(dist_far > dist_mid, "distance must strictly decrease as drone advances at 30°: far={dist_far}, mid={dist_mid}");
    assert!(dist_mid > dist_near, "distance must strictly decrease: mid={dist_mid}, near={dist_near}");
}

#[test]
fn test_angled_heading_45_deg_wall_intersection() {
    let wall = make_obstacle(
        "wall_45",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Wall 45",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();

    let deg45 = 45.0_f64;
    let rad45 = deg45.to_radians();
    let probe = [0.2, 0.5, 0.0];
    let mount_x = probe[0] + 0.05 * rad45.cos();

    // 1. Center ray (pencil beam):
    let mut sensor_pencil = default_sensor(SensorKind::FrontRange);
    sensor_pencil.fov_deg = 0.0;
    let reading_pencil = evaluate(&sensor_pencil, probe, deg45, &obs_set, &color_table);
    assert!(reading_pencil.hit);
    let expected_center_t = (0.75 - mount_x) / rad45.cos();
    let dist_pencil = reading_pencil.distance_m.unwrap();
    assert!(
        (dist_pencil - expected_center_t).abs() < 1e-4,
        "pencil beam center ray must hit at theoretical {expected_center_t}, got {dist_pencil}"
    );

    // 2. 30° FOV cone (45° - 15° = 30° ray in ground plane):
    let sensor_cone = default_sensor(SensorKind::FrontRange);
    let reading_cone = evaluate(&sensor_cone, probe, deg45, &obs_set, &color_table);
    assert!(reading_cone.hit);
    let rad30 = 30.0_f64.to_radians();
    let expected_cone_t = (0.75 - mount_x) / rad30.cos();
    let dist_cone = reading_cone.distance_m.unwrap();
    assert!(
        (dist_cone - expected_cone_t).abs() < 1e-4,
        "30° FOV cone ray at 30° must hit at theoretical {expected_cone_t}, got {dist_cone}"
    );

    let runtime_val = eval_front_range(probe, deg45, Some(&obs_set));
    let expected_cone_cm = (expected_cone_t * 100.0).min(100.0);
    assert!(
        (runtime_val - expected_cone_cm).abs() < 1e-2,
        "runtime cm {runtime_val} must match {expected_cone_cm}"
    );

    // 3. Test distance clamping when raw distance exceeds 1.0 m (100.0 cm)
    // From x = -0.3 at 45°: expected_cone_t = (0.75 - (-0.3 + 0.0353)) / cos(30°) = 1.17m > 1.0m
    let probe_far = [-0.3, 0.5, 0.0];
    let reading_far = evaluate(&sensor_cone, probe_far, deg45, &obs_set, &color_table);
    assert!(reading_far.hit, "sensor range is 1.5m, so 1.17m should hit in evaluate");
    assert!(reading_far.distance_m.unwrap() > 1.0);
    let runtime_clamped = eval_front_range(probe_far, deg45, Some(&obs_set));
    assert_eq!(
        runtime_clamped, 100.0,
        "raw distance > 100cm must be clamped to 100.0cm hardware ceiling"
    );
}

#[test]
fn test_angled_heading_60_deg_wall_intersection() {
    let wall = make_obstacle(
        "wall_60",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Wall 60",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();

    let deg60 = 60.0_f64;
    let rad60 = deg60.to_radians();
    let probe = [0.35, 0.5, 0.0];
    let mount_x = probe[0] + 0.05 * rad60.cos();

    // 1. Center ray (pencil beam):
    let mut sensor_pencil = default_sensor(SensorKind::FrontRange);
    sensor_pencil.fov_deg = 0.0;
    let reading_pencil = evaluate(&sensor_pencil, probe, deg60, &obs_set, &color_table);
    assert!(reading_pencil.hit);
    let expected_center_t = (0.75 - mount_x) / rad60.cos();
    let dist_pencil = reading_pencil.distance_m.unwrap();
    assert!(
        (dist_pencil - expected_center_t).abs() < 1e-4,
        "pencil beam center ray must hit at theoretical {expected_center_t}, got {dist_pencil}"
    );

    // 2. 30° FOV cone (60° - 15° = 45° ray in ground plane):
    let sensor_cone = default_sensor(SensorKind::FrontRange);
    let reading_cone = evaluate(&sensor_cone, probe, deg60, &obs_set, &color_table);
    assert!(reading_cone.hit);
    let rad45 = 45.0_f64.to_radians();
    let expected_cone_t = (0.75 - mount_x) / rad45.cos();
    let dist_cone = reading_cone.distance_m.unwrap();
    assert!(
        (dist_cone - expected_cone_t).abs() < 1e-4,
        "30° FOV cone ray at 45° must hit at theoretical {expected_cone_t}, got {dist_cone}"
    );

    let runtime_val = eval_front_range(probe, deg60, Some(&obs_set));
    let expected_cone_cm = (expected_cone_t * 100.0).min(100.0);
    assert!(
        (runtime_val - expected_cone_cm).abs() < 1e-2,
        "runtime cm {runtime_val} must match {expected_cone_cm}"
    );
}

#[test]
fn test_rotated_obstacle_slanted_geometry() {
    let rotated_wall = make_obstacle(
        "rot_wall",
        "wall",
        [1.5, 0.5, 0.0],
        [0.0, FRAC_PI_4, 0.0],
        [1.0, 1.0, 1.0],
        "Rotated Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![rotated_wall]);
    let aabb = obs_set.world_box(&obs_set.obstacles[0]);

    assert!(aabb.min[0] < 1.5);
    assert!(aabb.max[0] > 1.5);
    assert!(aabb.min[2] < 0.0);
    assert!(aabb.max[2] > 0.0);

    let dist_0 = eval_front_range([0.0, 0.5, 0.0], 0.0, Some(&obs_set));
    let dist_30 = eval_front_range([0.0, 0.5, 0.0], 30.0, Some(&obs_set));
    let dist_45 = eval_front_range([0.0, 0.5, 0.0], 45.0, Some(&obs_set));

    assert!(dist_0 < 100.0, "should detect rotated wall AABB at 0°");
    assert!(dist_30 <= 100.0, "should detect or clamp rotated wall at 30°");
    assert!(dist_45 <= 100.0, "should detect or clamp rotated wall at 45°");
}

#[test]
fn test_fov_cone_catches_sideways_wall_when_center_ray_misses() {
    let col = make_obstacle(
        "column",
        "wall",
        [1.0, 0.5, 0.35],
        [0.0, 0.0, 0.0],
        [0.2, 1.0, 0.2],
        "Column",
        None,
    );
    let obs_set = make_obstacle_set(vec![col]);
    let color_table = ColorTable::new();

    let sensor_fov = default_sensor(SensorKind::FrontRange);
    let reading_fov = evaluate(&sensor_fov, [0.0, 0.5, 0.0], 0.0, &obs_set, &color_table);

    let mut sensor_pencil = default_sensor(SensorKind::FrontRange);
    sensor_pencil.fov_deg = 0.0;
    let reading_pencil = evaluate(&sensor_pencil, [0.0, 0.5, 0.0], 0.0, &obs_set, &color_table);

    assert!(
        !reading_pencil.hit,
        "pencil beam center ray must miss obstacle offset at z = 0.35"
    );
    assert!(
        reading_fov.hit,
        "30° FOV cone must catch obstacle offset at z = 0.35"
    );

    let far_col = make_obstacle(
        "far_column",
        "wall",
        [1.0, 0.5, 0.8],
        [0.0, 0.0, 0.0],
        [0.2, 1.0, 0.2],
        "Far Column",
        None,
    );
    let far_obs_set = make_obstacle_set(vec![far_col]);
    let reading_far = evaluate(&sensor_fov, [0.0, 0.5, 0.0], 0.0, &far_obs_set, &color_table);
    assert!(
        !reading_far.hit,
        "obstacle at 0.8m offset (outside 15° cone) must not hit"
    );
}

#[test]
fn test_cone_directions_basis_transition_near_0_9() {
    let wall = make_obstacle(
        "wall_trans",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Wall Transition",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let test_angles = [24.0, 25.0, 25.8, 25.84, 25.85, 25.9, 26.0, 27.0];
    for deg in test_angles {
        let reading = evaluate(&sensor, [0.2, 0.5, 0.0], deg, &obs_set, &color_table);
        assert!(reading.hit, "heading {deg}° must hit wall");
        let d = reading.distance_m.unwrap();
        assert!(d.is_finite(), "distance at {deg}° must be finite");
        assert!(d > 0.0 && d < 1.5, "distance at {deg}° must be within valid range");
    }
}

// =========================================================================
// Group 2: Vertical Edge Cases & Negative Coordinates
// =========================================================================

#[test]
fn test_vertical_ceiling_high_altitude_5m() {
    let desk = make_obstacle(
        "low_desk",
        "wall",
        [0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.6, 1.0],
        "Low Desk",
        None,
    );
    let obs_set = make_obstacle_set(vec![desk]);
    let color_table = ColorTable::new();

    let bottom_sensor = default_sensor(SensorKind::BottomRange);

    let r_5m = evaluate(&bottom_sensor, [0.0, 5.0, 0.0], 0.0, &obs_set, &color_table);
    assert!(!r_5m.hit, "at 5.0m altitude bottom sensor (range 1.5m) must not hit 0.6m desk");
    assert_eq!(r_5m.distance_m, None);

    let sim_bottom_5m = eval_bottom_range([0.0, 5.0, 0.0], 0.0, 5.0, Some(&obs_set));
    assert_eq!(
        sim_bottom_5m, 500.0,
        "eval_bottom_range must return ground height (500cm) when out of sensor range"
    );

    let sim_bottom_10m = eval_bottom_range([0.0, 10.0, 0.0], 0.0, 10.0, Some(&obs_set));
    assert_eq!(sim_bottom_10m, 1000.0);

    let front_sensor = default_sensor(SensorKind::FrontRange);
    let r_front_5m = evaluate(&front_sensor, [0.0, 5.0, 0.0], 0.0, &obs_set, &color_table);
    assert!(!r_front_5m.hit, "front ray at 5m passes above 1m wall");

    let sim_front_5m = eval_front_range([0.0, 5.0, 0.0], 0.0, Some(&obs_set));
    assert_eq!(
        sim_front_5m, 100.0,
        "front range above obstacle ceiling must return 100cm escapement"
    );
}

#[test]
fn test_vertical_ground_level_0m() {
    let wall = make_obstacle(
        "ground_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Ground Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);

    let bottom_at_ground = eval_bottom_range([0.0, 0.0, 0.0], 0.0, 0.0, Some(&obs_set));
    assert_eq!(
        bottom_at_ground, 0.0,
        "bottom range at ground level Z=0.0m must be 0.0cm"
    );

    let front_at_ground = eval_front_range([0.0, 0.0, 0.0], 0.0, Some(&obs_set));
    assert!(
        front_at_ground < 100.0,
        "front range at ground level must hit wall resting on ground: got {front_at_ground}"
    );
    assert!(
        (front_at_ground - 70.0).abs() < 1.0,
        "wall at 1.0m, half-width 0.25m, mount offset 0.05m -> dist ~ 70cm"
    );
}

#[test]
fn test_negative_coordinates_geometry() {
    let wall = make_obstacle(
        "neg_wall",
        "wall",
        [-1.0, 0.5, -1.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Neg Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);
    let color_table = ColorTable::new();
    let sensor = default_sensor(SensorKind::FrontRange);

    let probe = [-2.0, 0.5, -1.0];
    let reading = evaluate(&sensor, probe, 0.0, &obs_set, &color_table);
    assert!(reading.hit, "drone in negative space facing +X must hit wall");
    let dist_m = reading.distance_m.unwrap();
    assert!(
        (dist_m - 0.70).abs() < 1e-4,
        "distance in negative space must be 0.70m, got {dist_m}"
    );

    let runtime_val = eval_front_range(probe, 0.0, Some(&obs_set));
    assert!(
        (runtime_val - 70.0).abs() < 1e-2,
        "runtime reading must be 70.0cm, got {runtime_val}"
    );

    let bottom_neg = eval_bottom_range([0.0, -0.5, 0.0], 0.0, -0.5, Some(&obs_set));
    assert!(
        bottom_neg.is_finite(),
        "negative altitude bottom range must be finite, got {bottom_neg}"
    );
}

#[test]
fn test_thin_wall_boundary_intersection() {
    let thin_wall = make_obstacle(
        "thin_wall",
        "wall",
        [0.6, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [0.02, 1.0, 1.0],
        "Thin Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![thin_wall]);
    let sensor = default_sensor(SensorKind::FrontRange);
    let color_table = ColorTable::new();

    let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &obs_set, &color_table);
    assert!(
        reading.hit,
        "raycaster must not miss thin 1cm wall due to numerical precision"
    );
    assert!(reading.distance_m.unwrap() > 0.5 && reading.distance_m.unwrap() < 0.6);
}

// =========================================================================
// Group 3: Rapid Succession Queries & Tight While Loops
// =========================================================================

#[test]
fn test_tight_while_loop_sensor_dynamic_break() {
    let wall = make_obstacle(
        "loop_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Loop Wall",
        None,
    );
    let obs_set = make_obstacle_set(vec![wall]);

    let commands = vec![
        make_cmd("cmd_to", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "cmd_init_sensor",
            CommandType::GetFrontRange,
            &[("var", ParamValue::Str("d".to_string()))],
            vec![],
        ),
        make_cmd(
            "cmd_while",
            CommandType::WhileBlock,
            &[("condition", ParamValue::Str("d > 35".to_string()))],
            vec![
                make_cmd(
                    "cmd_move",
                    CommandType::MoveForward,
                    &[
                        ("dist", ParamValue::Number(10.0)),
                        ("speed", ParamValue::Number(50.0)),
                    ],
                    vec![],
                ),
                make_cmd(
                    "cmd_read",
                    CommandType::GetFrontRange,
                    &[("var", ParamValue::Str("d".to_string()))],
                    vec![],
                ),
            ],
        ),
    ];

    let result = simulate_commands(&commands, Some(&obs_set));
    let final_pos = result.positions.last().expect("must have final position");

    assert!(
        final_pos.x > 0.30 && final_pos.x < 0.60,
        "drone must stop cleanly when d <= 35cm, final x={}",
        final_pos.x
    );
    assert!(
        result.positions.len() < 300,
        "loop must terminate in few iterations, got {} points",
        result.positions.len()
    );
}

#[test]
fn test_tight_while_loop_rapid_multi_sensor_queries_100_iterations() {
    let wall = make_obstacle(
        "multi_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Multi Wall",
        Some("red"),
    );
    let obs_set = make_obstacle_set(vec![wall]);

    let mut loop_body = Vec::new();
    loop_body.push(make_cmd(
        "s_fr",
        CommandType::GetFrontRange,
        &[("var", ParamValue::Str("fr".to_string()))],
        vec![],
    ));
    loop_body.push(make_cmd(
        "s_br",
        CommandType::GetBottomRange,
        &[("var", ParamValue::Str("br".to_string()))],
        vec![],
    ));
    loop_body.push(make_cmd(
        "s_fc",
        CommandType::GetFrontColor,
        &[("var", ParamValue::Str("fc".to_string()))],
        vec![],
    ));
    loop_body.push(make_cmd(
        "s_bc",
        CommandType::GetBackColor,
        &[("var", ParamValue::Str("bc".to_string()))],
        vec![],
    ));

    let commands = vec![
        make_cmd("to", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "for_loop",
            CommandType::ForBlock,
            &[
                ("var", ParamValue::Str("i".to_string())),
                ("start", ParamValue::Number(0.0)),
                ("end_val", ParamValue::Number(100.0)),
                ("step", ParamValue::Number(1.0)),
            ],
            loop_body,
        ),
    ];

    let start_time = std::time::Instant::now();
    let result = simulate_commands(&commands, Some(&obs_set));
    let elapsed = start_time.elapsed();

    assert!(result.positions.len() > 0);
    assert!(
        elapsed.as_millis() < 500,
        "100 iterations of 4 sensor reads took {:?}, must be under 500ms",
        elapsed
    );
}

#[test]
fn test_dynamic_obstacle_updates_between_simulations() {
    let mut obs_set = make_obstacle_set(vec![]);
    let probe = [0.0, 0.5, 0.0];

    // State 1: No obstacles -> front range escapement 100cm
    assert_eq!(eval_front_range(probe, 0.0, Some(&obs_set)), 100.0);

    // State 2: Add obstacle at x = 1.0 (surface at 0.75m -> 70cm)
    obs_set.obstacles.push(make_obstacle(
        "dyn_wall",
        "wall",
        [1.0, 0.5, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        "Dyn Wall",
        Some("blue"),
    ));
    let d1 = eval_front_range(probe, 0.0, Some(&obs_set));
    assert!((d1 - 70.0).abs() < 1e-2);

    // State 3: Move obstacle closer to x = 0.6 (surface at 0.35m -> 30cm)
    obs_set.obstacles[0].position[0] = 0.6;
    let d2 = eval_front_range(probe, 0.0, Some(&obs_set));
    assert!((d2 - 30.0).abs() < 1e-2);

    // State 4: Remove obstacle -> returns to 100cm escapement
    obs_set.obstacles.clear();
    let d3 = eval_front_range(probe, 0.0, Some(&obs_set));
    assert_eq!(d3, 100.0);
}

#[test]
fn test_while_loop_boundary_max_while_loops_500() {
    let obs_set = make_obstacle_set(vec![]);

    let commands = vec![
        make_cmd("to", CommandType::Takeoff, &[], vec![]),
        make_cmd(
            "infinite_while",
            CommandType::WhileBlock,
            &[("condition", ParamValue::Str("1 == 1".to_string()))],
            vec![make_cmd(
                "read_s",
                CommandType::GetFrontRange,
                &[("var", ParamValue::Str("val".to_string()))],
                vec![],
            )],
        ),
    ];

    let result = simulate_commands(&commands, Some(&obs_set));
    assert!(result.positions.len() > 0);
}

#[test]
fn test_four_cardinal_and_diagonal_symmetry() {
    // 4 identical walls rotated properly around Y so each presents its 0.5m-wide face
    // at a distance of 1.0m from the origin:
    // East:  pos [1.0, 0.5, 0.0], rot [0, 0, 0]
    // North: pos [0.0, 0.5, 1.0], rot [0, PI/2, 0]
    // West:  pos [-1.0, 0.5, 0.0], rot [0, PI, 0]
    // South: pos [0.0, 0.5, -1.0], rot [0, -PI/2, 0]
    let walls = vec![
        make_obstacle("w_east", "wall", [1.0, 0.5, 0.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], "East", None),
        make_obstacle("w_north", "wall", [0.0, 0.5, 1.0], [0.0, FRAC_PI_2, 0.0], [1.0, 1.0, 1.0], "North", None),
        make_obstacle("w_west", "wall", [-1.0, 0.5, 0.0], [0.0, std::f64::consts::PI, 0.0], [1.0, 1.0, 1.0], "West", None),
        make_obstacle("w_south", "wall", [0.0, 0.5, -1.0], [0.0, -FRAC_PI_2, 0.0], [1.0, 1.0, 1.0], "South", None),
    ];
    let obs_set = make_obstacle_set(walls);

    let probe = [0.0, 0.5, 0.0];
    let d_east = eval_front_range(probe, 0.0, Some(&obs_set));     // +X (East)
    let d_north = eval_front_range(probe, 90.0, Some(&obs_set));   // +Z (North)
    let d_west = eval_front_range(probe, 180.0, Some(&obs_set));   // -X (West)
    let d_south = eval_front_range(probe, 270.0, Some(&obs_set));  // -Z (South)

    assert!((d_east - d_north).abs() < 1e-4, "East ({d_east}) and North ({d_north}) must match");
    assert!((d_north - d_west).abs() < 1e-4, "North ({d_north}) and West ({d_west}) must match");
    assert!((d_west - d_south).abs() < 1e-4, "West ({d_west}) and South ({d_south}) must match");
    assert_eq!(d_east, 70.0, "distance should be 70.0cm");
}
