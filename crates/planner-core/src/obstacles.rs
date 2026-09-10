use crate::planio::Vec3;
use serde::{Deserialize, Serialize};
use std::sync::LazyLock;

pub const COLLISION_DRONE_SIZE: f64 = 0.1;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Boundary {
    #[serde(rename = "minX")]
    pub min_x: f64,
    #[serde(rename = "maxX")]
    pub max_x: f64,
    #[serde(rename = "minZ")]
    pub min_z: f64,
    #[serde(rename = "maxZ")]
    pub max_z: f64,
    #[serde(rename = "maxY")]
    pub max_y: f64,
}

impl Default for Boundary {
    fn default() -> Self {
        Boundary {
            min_x: -4.0,
            max_x: 4.0,
            min_z: -4.0,
            max_z: 4.0,
            max_y: 4.0,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn from_center_half(center: Vec3, half: Vec3) -> Aabb {
        Aabb {
            min: [
                center[0] - half[0],
                center[1] - half[1],
                center[2] - half[2],
            ],
            max: [
                center[0] + half[0],
                center[1] + half[1],
                center[2] + half[2],
            ],
        }
    }

    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleDims {
    pub width: f64,
    pub height: f64,
    pub depth: f64,
    pub inner_radius: f64,
    pub outer_radius: f64,
    pub radius: f64,
}

pub fn type_dims(obstacle_type: &str) -> Option<ObstacleDims> {
    match obstacle_type {
        "wall" => Some(ObstacleDims {
            width: 0.5,
            height: 1.0,
            depth: 2.0,
            inner_radius: 0.0,
            outer_radius: 0.0,
            radius: 0.0,
        }),
        "tower" => Some(ObstacleDims {
            width: 0.3,
            height: 1.5,
            depth: 0.3,
            inner_radius: 0.0,
            outer_radius: 0.0,
            radius: 0.0,
        }),
        "hoop" => Some(ObstacleDims {
            width: 0.0,
            height: 0.05,
            depth: 0.0,
            inner_radius: 0.4,
            outer_radius: 0.5,
            radius: 0.0,
        }),
        "cone" => Some(ObstacleDims {
            width: 0.0,
            height: 0.4,
            depth: 0.0,
            inner_radius: 0.0,
            outer_radius: 0.0,
            radius: 0.2,
        }),
        "square" => Some(ObstacleDims {
            width: 0.5,
            height: 1.0,
            depth: 2.0,
            inner_radius: 0.0,
            outer_radius: 0.0,
            radius: 0.0,
        }),
        "sphere" => Some(ObstacleDims {
            width: 0.0,
            height: 0.0,
            depth: 0.0,
            inner_radius: 0.0,
            outer_radius: 0.0,
            radius: 0.25,
        }),
        _ => None,
    }
}

fn base_obstacle(id: &str, obstacle_type: &str, position: Vec3, rotation: Vec3, name: &str) -> Obstacle {
    Obstacle {
        id: id.to_string(),
        obstacle_type: obstacle_type.to_string(),
        position,
        rotation,
        scale: [1.0, 1.0, 1.0],
        name: name.to_string(),
        color: None,
    }
}

pub static BASE_OBSTACLES: LazyLock<Vec<Obstacle>> = LazyLock::new(|| {
    vec![
        base_obstacle("base_wall_1", "wall", [1.0, 0.5, 0.0], [0.0, 0.0, 0.0], "Wall 1"),
        base_obstacle("base_wall_2", "wall", [-1.0, 0.5, 0.0], [0.0, 0.0, 0.0], "Wall 2"),
        base_obstacle(
            "base_wall_3",
            "wall",
            [0.0, 0.5, 1.0],
            [0.0, std::f64::consts::FRAC_PI_2, 0.0],
            "Wall 3",
        ),
        base_obstacle(
            "base_wall_4",
            "wall",
            [0.0, 0.5, -1.0],
            [0.0, std::f64::consts::FRAC_PI_2, 0.0],
            "Wall 4",
        ),
        base_obstacle("base_tower", "tower", [0.0, 0.75, 0.0], [0.0, 0.0, 0.0], "Tower"),
        base_obstacle(
            "base_hoop",
            "hoop",
            [0.8, 1.2, 0.8],
            [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
            "Hoop",
        ),
        base_obstacle("base_cone_1", "cone", [-0.8, 0.2, 0.8], [0.0, 0.0, 0.0], "Cone 1"),
        base_obstacle("base_cone_2", "cone", [0.8, 0.2, -0.8], [0.0, 0.0, 0.0], "Cone 2"),
    ]
});

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obstacle {
    pub id: String,
    #[serde(rename = "type")]
    pub obstacle_type: String,
    pub position: Vec3,
    pub rotation: Vec3,
    pub scale: Vec3,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}

#[derive(Debug, Clone)]
pub struct CollisionHit {
    pub obstacle: Obstacle,
    pub world_box: Aabb,
    pub distance: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObstacleSet {
    pub obstacles: Vec<Obstacle>,
    pub boundary: Boundary,
}

impl Default for ObstacleSet {
    fn default() -> Self {
        ObstacleSet {
            obstacles: Vec::new(),
            boundary: Boundary::default(),
        }
    }
}

impl ObstacleSet {
    pub fn new() -> ObstacleSet {
        ObstacleSet::default()
    }

    pub fn world_box(&self, obstacle: &Obstacle) -> Aabb {
        let _ = obstacle;
        Aabb::from_center_half(obstacle.position, [0.5, 0.5, 0.5])
    }

    pub fn check_collision(
        &self,
        sim_x: f64,
        sim_y: f64,
        sim_z: f64,
        drone_size: f64,
    ) -> Option<CollisionHit> {
        let _ = (sim_x, sim_y, sim_z, drone_size);
        None
    }

    pub fn within_boundary(&self, position: Vec3) -> bool {
        let b = self.boundary;
        position[0] >= b.min_x
            && position[0] <= b.max_x
            && position[2] >= b.min_z
            && position[2] <= b.max_z
            && position[1] >= 0.0
            && position[1] <= b.max_y
    }

    pub fn import(&mut self, obstacles: Vec<Obstacle>) -> usize {
        let mut rejected = 0;
        let mut accepted = Vec::new();
        for obstacle in obstacles {
            if self.within_boundary(obstacle.position) {
                accepted.push(obstacle);
            } else {
                rejected += 1;
            }
        }
        self.obstacles = accepted;
        rejected
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleFormat {
    Json,
    GeoJson,
    Csv,
    Obj,
}

pub fn import_json(text: &str) -> Result<Vec<Obstacle>, String> {
    let _ = text;
    Ok(Vec::new())
}

pub fn import_geojson(text: &str) -> Result<Vec<Obstacle>, String> {
    let _ = text;
    Ok(Vec::new())
}

pub fn import_csv(text: &str) -> Result<Vec<Obstacle>, String> {
    let _ = text;
    Ok(Vec::new())
}

pub fn import_obj(text: &str) -> Result<Vec<Obstacle>, String> {
    let _ = text;
    Ok(Vec::new())
}
