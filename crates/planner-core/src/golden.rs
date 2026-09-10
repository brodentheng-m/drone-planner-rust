use crate::commands::Command;
use crate::obstacles::{Boundary, Obstacle};
use crate::planio::PlanDrone;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenMeta {
    #[serde(rename = "capturedAt")]
    pub captured_at: String,
    pub source: String,
    pub plans: u32,
    pub note: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenFile {
    pub meta: GoldenMeta,
    #[serde(rename = "commandDefCount")]
    pub command_def_count: u32,
    #[serde(rename = "commandTypes")]
    pub command_types: Vec<String>,
    pub plans: Vec<GoldenPlan>,
    pub swarm: GoldenSwarm,
    pub obstacles: GoldenObstacles,
    #[serde(rename = "collisionPlan")]
    pub collision_plan: GoldenCollisionPlan,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenPlan {
    pub name: String,
    pub commands: Vec<Command>,
    pub code: String,
    pub result: SimResult,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenSwarm {
    pub drones: Vec<PlanDrone>,
    pub result: BTreeMap<String, SimResult>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenCollisionPlan {
    pub drones: Vec<PlanDrone>,
    pub result: SimResult,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoldenObstacles {
    pub obstacles: Vec<Obstacle>,
    pub boundary: Boundary,
    #[serde(rename = "rejectedCount")]
    pub rejected_count: u32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimResult {
    pub positions: Vec<Point>,
    #[serde(rename = "totalDuration")]
    pub total_duration: f64,
    pub collisions: Vec<Collision>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub speed: f64,
    #[serde(rename = "energyUsed")]
    pub energy_used: f64,
    #[serde(rename = "batteryPercent")]
    pub battery_percent: f64,
    #[serde(rename = "turnRadiusM")]
    pub turn_radius_m: Option<f64>,
    pub led: LedValue,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum LedValue {
    Name(String),
    Rgb {
        r: f64,
        g: f64,
        b: f64,
        brightness: f64,
    },
}

impl Default for LedValue {
    fn default() -> Self {
        LedValue::Name("off".to_string())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Collision {
    pub position: Position,
    pub obstacle: CollisionObstacle,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionObstacle {
    pub id: String,
    #[serde(rename = "type")]
    pub obstacle_type: String,
    pub position: Position,
    pub rotation: EulerRotation,
    pub name: String,
    pub scale: Position,
    #[serde(rename = "_worldBox")]
    pub world_box: WorldBox,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EulerRotation {
    #[serde(rename = "isEuler")]
    pub is_euler: bool,
    #[serde(rename = "_x")]
    pub x: f64,
    #[serde(rename = "_y")]
    pub y: f64,
    #[serde(rename = "_z")]
    pub z: f64,
    #[serde(rename = "_order")]
    pub order: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldBox {
    #[serde(rename = "isBox3")]
    pub is_box3: bool,
    pub min: Position,
    pub max: Position,
}

impl GoldenFile {
    pub fn load(path: &str) -> Result<GoldenFile, serde_json::Error> {
        let text = std::fs::read_to_string(path).map_err(serde_json::Error::io)?;
        serde_json::from_str(&text)
    }
}
