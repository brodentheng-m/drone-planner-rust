use planner_core::commands::CommandType;
use planner_core::golden::GoldenFile;

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
        assert!(
            !plan.code.is_empty(),
            "plan {} has empty code",
            plan.name
        );
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
