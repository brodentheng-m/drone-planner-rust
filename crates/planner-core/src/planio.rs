use crate::commands::Command;
use crate::obstacles::{Boundary, Obstacle};
use serde::{Deserialize, Serialize};

pub type Vec3 = [f64; 3];

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanDrone {
    pub id: String,
    pub name: String,
    pub color: String,
    pub commands: Vec<Command>,
    pub offset: Vec3,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plan {
    pub name: String,
    pub drones: Vec<PlanDrone>,
    #[serde(default, rename = "activeDroneId")]
    pub active_drone_id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObstacleFile {
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
    #[serde(default)]
    pub boundary: Option<Boundary>,
}

impl Default for Plan {
    fn default() -> Self {
        Plan::default_plan()
    }
}

impl Plan {
    pub fn default_plan() -> Plan {
        Plan {
            name: "Untitled Flight Plan".to_string(),
            drones: vec![PlanDrone {
                id: "d1".to_string(),
                name: "Drone 1".to_string(),
                color: "#58a6ff".to_string(),
                commands: Vec::new(),
                offset: [0.0, 0.0, 0.0],
            }],
            active_drone_id: None,
        }
    }
}

impl Default for ObstacleFile {
    fn default() -> Self {
        ObstacleFile {
            obstacles: Vec::new(),
            boundary: Some(Boundary::default()),
        }
    }
}

pub fn load_plan(path: &str) -> Result<Plan, serde_json::Error> {
    let text = std::fs::read_to_string(path).map_err(serde_json::Error::io)?;
    serde_json::from_str(&text)
}

pub fn save_plan(path: &str, plan: &Plan) -> Result<(), serde_json::Error> {
    let text = serde_json::to_string_pretty(plan)?;
    std::fs::write(path, text).map_err(serde_json::Error::io)
}

pub fn load_obstacles(path: &str) -> Result<ObstacleFile, serde_json::Error> {
    let text = std::fs::read_to_string(path).map_err(serde_json::Error::io)?;
    match serde_json::from_str::<ObstacleFile>(&text) {
        Ok(file) => Ok(file),
        Err(struct_error) => match serde_json::from_str::<Vec<Obstacle>>(&text) {
            Ok(obstacles) => Ok(ObstacleFile {
                obstacles,
                boundary: Some(Boundary::default()),
            }),
            Err(_) => Err(struct_error),
        },
    }
}

pub fn save_obstacles(path: &str, file: &ObstacleFile) -> Result<(), serde_json::Error> {
    let text = serde_json::to_string_pretty(file)?;
    std::fs::write(path, text).map_err(serde_json::Error::io)
}
