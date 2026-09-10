use planner_core::codegen::{generate_code, generate_swarm_code};
use planner_core::commands::CommandType;
use planner_core::golden::{GoldenFile, GoldenPlan, LedValue, Point, SimResult};
use planner_core::obstacles::{Obstacle, ObstacleSet};
use planner_core::planio::{Plan, PlanDrone};
use planner_core::sim::simulate_swarm;

#[test]
fn golden_loads() {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let path = format!("{manifest}/../../tests/golden/golden.json");
    let golden = GoldenFile::load(&path).expect("golden.json must load");
    assert_eq!(golden.plans.len(), 11);
    assert_eq!(golden.command_types.len(), 58);
    for plan in &golden.plans {
        assert!(
            !plan.result.positions.is_empty(),
            "plan {} has empty positions",
            plan.name
        );
        assert!(!plan.code.is_empty(), "plan {} has empty code", plan.name);
    }
    assert_eq!(golden.obstacles.obstacles.len(), 6);
    assert_eq!(golden.obstacles.rejected_count, 1);
    assert_eq!(golden.collision_plan.result.collisions.len(), 1);
}

#[test]
fn command_types_match_golden() {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let path = format!("{manifest}/../../tests/golden/golden.json");
    let golden = GoldenFile::load(&path).expect("golden.json must load");
    assert_eq!(golden.command_types.len(), 58);
    for name in &golden.command_types {
        let parsed: CommandType =
            serde_json::from_value(serde_json::Value::String(name.clone())).expect(name);
        assert_eq!(
            &serde_json::to_value(parsed).expect("serialize command type"),
            &serde_json::Value::String(name.clone()),
            "command type round trip failed for {name}"
        );
    }
}

fn load_golden() -> GoldenFile {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let path = format!("{manifest}/../../tests/golden/golden.json");
    GoldenFile::load(&path).expect("golden.json must load")
}

fn approx_eq_pos(a: &Point, b: &Point) -> bool {
    (a.x - b.x).abs() <= 1e-2 && (a.y - b.y).abs() <= 1e-2 && (a.z - b.z).abs() <= 1e-2
}

fn approx_eq_pos_xyz(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> bool {
    (ax - bx).abs() <= 1e-2 && (ay - by).abs() <= 1e-2 && (az - bz).abs() <= 1e-2
}

fn approx_eq_angle(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-1
}

fn approx_eq_rel(a: f64, b: f64) -> bool {
    if a == 0.0 && b == 0.0 {
        return true;
    }
    (a - b).abs() <= 1e-2 * a.abs().max(b.abs()) || (a - b).abs() <= 1e-9
}

fn turn_radius_matches(a: &Point, b: &Point) -> bool {
    match (a.turn_radius_m, b.turn_radius_m) {
        (None, None) => true,
        (Some(x), Some(y)) => approx_eq_rel(x, y),
        _ => false,
    }
}

fn points_match(ours: &Point, golden: &Point) -> bool {
    approx_eq_pos(ours, golden)
        && approx_eq_angle(ours.heading, golden.heading)
        && approx_eq_angle(ours.pitch, golden.pitch)
        && approx_eq_angle(ours.roll, golden.roll)
        && approx_eq_rel(ours.speed, golden.speed)
        && approx_eq_rel(ours.energy_used, golden.energy_used)
        && approx_eq_rel(ours.battery_percent, golden.battery_percent)
        && turn_radius_matches(ours, golden)
        && ours.led == golden.led
}

fn compare_results(ours: &SimResult, golden: &SimResult, label: &str) {
    assert_eq!(
        ours.positions.len(),
        golden.positions.len(),
        "{label} point count ours {} golden {}",
        ours.positions.len(),
        golden.positions.len()
    );
    for (index, (ours_point, golden_point)) in ours
        .positions
        .iter()
        .zip(golden.positions.iter())
        .enumerate()
    {
        assert!(
            points_match(ours_point, golden_point),
            "{label} point {index} mismatch ours {:?} golden {:?}",
            ours_point,
            golden_point
        );
    }
    assert!(
        (ours.total_duration - golden.total_duration).abs() <= 1e-2,
        "{label} total duration ours {} golden {}",
        ours.total_duration,
        golden.total_duration
    );
}

fn single_drone_plan(name: &str, commands: &[planner_core::commands::Command]) -> Plan {
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

fn swarm_plan(drones: &[PlanDrone]) -> Plan {
    Plan {
        name: "swarm".to_string(),
        drones: drones.to_vec(),
        active_drone_id: None,
    }
}

fn find_plan<'a>(golden: &'a GoldenFile, name: &str) -> &'a GoldenPlan {
    golden
        .plans
        .iter()
        .find(|plan| plan.name == name)
        .unwrap_or_else(|| panic!("golden plan {name} missing"))
}

macro_rules! plan_parity_cases {
    ($($test:ident => $plan:literal),* $(,)?) => {
        $(
            #[test]
            fn $test() {
                let golden = load_golden();
                let plan = find_plan(&golden, $plan);
                let built = single_drone_plan(&plan.name, &plan.commands);
                let results = simulate_swarm(&built, None);
                let ours = results.get("d1").expect("d1 result missing");
                compare_results(ours, &plan.result, $plan);
            }
        )*
    };
}

macro_rules! plan_code_cases {
    ($($test:ident => $plan:literal),* $(,)?) => {
        $(
            #[test]
            fn $test() {
                let golden = load_golden();
                let plan = find_plan(&golden, $plan);
                let built = single_drone_plan(&plan.name, &plan.commands);
                assert_eq!(generate_code(&built), plan.code, "{} code drift", $plan);
            }
        )*
    };
}

plan_parity_cases! {
    plan_parity_takeoff_hover_land => "takeoff_hover_land",
    plan_parity_square_path => "square_path",
    plan_parity_flip_all4 => "flip_all4",
    plan_parity_shapes => "shapes",
    plan_parity_go_all_dirs => "go_all_dirs",
    plan_parity_fast_moves_climb => "fast_moves_climb",
    plan_parity_buzz_led => "buzz_led",
    plan_parity_sensors => "sensors",
    plan_parity_range_safety => "range_safety",
    plan_parity_control_flow => "control_flow",
    plan_parity_emergency => "emergency",
}

plan_code_cases! {
    plan_code_byte_identical_takeoff_hover_land => "takeoff_hover_land",
    plan_code_byte_identical_square_path => "square_path",
    plan_code_byte_identical_flip_all4 => "flip_all4",
    plan_code_byte_identical_shapes => "shapes",
    plan_code_byte_identical_go_all_dirs => "go_all_dirs",
    plan_code_byte_identical_fast_moves_climb => "fast_moves_climb",
    plan_code_byte_identical_buzz_led => "buzz_led",
    plan_code_byte_identical_sensors => "sensors",
    plan_code_byte_identical_range_safety => "range_safety",
    plan_code_byte_identical_control_flow => "control_flow",
    plan_code_byte_identical_emergency => "emergency",
}

#[test]
fn start_point_matches_golden() {
    let golden = load_golden();
    for plan in &golden.plans {
        let built = single_drone_plan(&plan.name, &plan.commands);
        let results = simulate_swarm(&built, None);
        let ours = &results["d1"].positions[0];
        let golden_start = &plan.result.positions[0];
        assert!(
            approx_eq_pos(ours, golden_start),
            "plan {} start position ours {:?} golden {:?}",
            plan.name,
            ours,
            golden_start
        );
        assert!(
            approx_eq_angle(ours.heading, golden_start.heading)
                && approx_eq_angle(ours.pitch, golden_start.pitch)
                && approx_eq_angle(ours.roll, golden_start.roll),
            "plan {} start angles ours {:?} golden {:?}",
            plan.name,
            ours,
            golden_start
        );
        assert!(
            approx_eq_rel(ours.speed, golden_start.speed)
                && approx_eq_rel(ours.energy_used, golden_start.energy_used)
                && approx_eq_rel(ours.battery_percent, golden_start.battery_percent),
            "plan {} start telemetry ours {:?} golden {:?}",
            plan.name,
            ours,
            golden_start
        );
        assert_eq!(
            ours.turn_radius_m, None,
            "plan {} start turn radius must be null",
            plan.name
        );
        assert_eq!(
            ours.led,
            LedValue::Name("off".to_string()),
            "plan {} start led must be off",
            plan.name
        );
    }
}

#[test]
fn swarm_parity() {
    let golden = load_golden();
    let plan = swarm_plan(&golden.swarm.drones);
    let results = simulate_swarm(&plan, None);
    let mut max_duration = 0.0f64;
    for (id, golden_result) in &golden.swarm.result {
        let ours = results
            .get(id)
            .unwrap_or_else(|| panic!("swarm result missing for {id}"));
        assert_eq!(
            ours.positions.len(),
            golden_result.positions.len(),
            "swarm {id} point count ours {} golden {}",
            ours.positions.len(),
            golden_result.positions.len()
        );
        for (index, (ours_point, golden_point)) in ours
            .positions
            .iter()
            .zip(golden_result.positions.iter())
            .enumerate()
        {
            assert!(
                points_match(ours_point, golden_point),
                "swarm {id} point {index} mismatch ours {:?} golden {:?}",
                ours_point,
                golden_point
            );
        }
        max_duration = max_duration.max(golden_result.total_duration);
    }
    for (id, golden_result) in &golden.swarm.result {
        let ours = &results[id];
        assert!(
            (ours.total_duration - golden_result.total_duration).abs() <= 1e-2,
            "swarm {id} duration ours {} golden {}",
            ours.total_duration,
            golden_result.total_duration
        );
        assert!(
            (ours.total_duration - max_duration).abs() <= 1e-2,
            "swarm {id} duration not normalized to max {}",
            max_duration
        );
    }
}

#[test]
fn swarm_code_byte_identical() {
    let golden = load_golden();
    let plan = swarm_plan(&golden.swarm.drones);
    let expected = golden.swarm.code.clone().unwrap_or_default();
    assert_eq!(generate_swarm_code(&plan), expected, "swarm code drift");
}

#[test]
fn collision_plan_parity() {
    let golden = load_golden();
    let mut set = ObstacleSet::new();
    set.obstacles = golden.obstacles.obstacles.clone();
    set.boundary = golden.obstacles.boundary;
    let plan = swarm_plan(&golden.collision_plan.drones);
    let results = simulate_swarm(&plan, Some(&set));
    let ours = results
        .get("d1")
        .unwrap_or_else(|| panic!("collision plan d1 result missing"));
    let golden_result = &golden.collision_plan.result;
    assert_eq!(
        ours.positions.len(),
        golden_result.positions.len(),
        "collision plan point count ours {} golden {}",
        ours.positions.len(),
        golden_result.positions.len()
    );
    for (index, (ours_point, golden_point)) in ours
        .positions
        .iter()
        .zip(golden_result.positions.iter())
        .enumerate()
    {
        assert!(
            points_match(ours_point, golden_point),
            "collision plan point {index} mismatch ours {:?} golden {:?}",
            ours_point,
            golden_point
        );
    }
    assert!(
        (ours.total_duration - golden_result.total_duration).abs() <= 1e-2,
        "collision plan duration ours {} golden {}",
        ours.total_duration,
        golden_result.total_duration
    );
    assert_eq!(
        ours.collisions.len(),
        golden_result.collisions.len(),
        "collision count ours {} golden {}",
        ours.collisions.len(),
        golden_result.collisions.len()
    );
    for (ours_hit, golden_hit) in ours.collisions.iter().zip(golden_result.collisions.iter()) {
        assert!(
            approx_eq_pos_xyz(
                ours_hit.position.x,
                ours_hit.position.y,
                ours_hit.position.z,
                golden_hit.position.x,
                golden_hit.position.y,
                golden_hit.position.z
            ),
            "collision position ours {:?} golden {:?}",
            ours_hit.position,
            golden_hit.position
        );
        assert_eq!(
            ours_hit.obstacle.obstacle_type, golden_hit.obstacle.obstacle_type,
            "collision obstacle type mismatch"
        );
        let mate = set
            .obstacles
            .iter()
            .find(|obstacle| {
                obstacle.obstacle_type == golden_hit.obstacle.obstacle_type
                    && approx_eq_pos_xyz(
                        obstacle.position[0],
                        obstacle.position[1],
                        obstacle.position[2],
                        golden_hit.obstacle.position.x,
                        golden_hit.obstacle.position.y,
                        golden_hit.obstacle.position.z,
                    )
            })
            .unwrap_or_else(|| panic!("no obstacle in set matches golden collision"));
        assert_eq!(
            ours_hit.obstacle.id, mate.id,
            "collision obstacle identity mismatch ours {} set {}",
            ours_hit.obstacle.id, mate.id
        );
    }
}

#[test]
fn obstacles_import_roundtrip() {
    let golden = load_golden();
    let mut defs = golden.obstacles.obstacles.clone();
    defs.push(Obstacle {
        id: "out_of_bounds".to_string(),
        obstacle_type: "tower".to_string(),
        position: [9.0, 9.0, 9.0],
        rotation: [0.0, 0.0, 0.0],
        scale: [1.0, 1.0, 1.0],
        name: "Reject Me".to_string(),
        color: None,
    });
    let mut set = ObstacleSet::new();
    set.boundary = golden.obstacles.boundary;
    let rejected = set.import(defs);
    assert_eq!(rejected, 1, "rejected count");
    assert_eq!(set.obstacles.len(), 6, "accepted count");
    for (ours, golden_obstacle) in set.obstacles.iter().zip(golden.obstacles.obstacles.iter()) {
        assert_eq!(ours.id, golden_obstacle.id, "roundtrip id");
        assert_eq!(
            ours.obstacle_type, golden_obstacle.obstacle_type,
            "roundtrip type"
        );
        assert_eq!(
            ours.position, golden_obstacle.position,
            "roundtrip position"
        );
        assert_eq!(
            ours.rotation, golden_obstacle.rotation,
            "roundtrip rotation"
        );
        assert_eq!(ours.scale, golden_obstacle.scale, "roundtrip scale");
        assert_eq!(ours.name, golden_obstacle.name, "roundtrip name");
    }
}
