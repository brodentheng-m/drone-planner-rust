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

fn rotation_matrix_xyz(rx: f64, ry: f64, rz: f64) -> [[f64; 3]; 3] {
    let (sx, cx) = (rx.sin(), rx.cos());
    let (sy, cy) = (ry.sin(), ry.cos());
    let (sz, cz) = (rz.sin(), rz.cos());
    [
        [cy * cz, -cy * sz, sy],
        [cx * sz + sx * sy * cz, cx * cz - sx * sy * sz, -sx * cy],
        [sx * sz - cx * sy * cz, sx * cz + cx * sy * sz, cx * cy],
    ]
}

fn local_bounds(obstacle_type: &str) -> Option<([f64; 3], [f64; 3])> {
    let dims = type_dims(obstacle_type)?;
    match obstacle_type {
        "wall" | "tower" | "square" => {
            let half = [dims.width / 2.0, dims.height / 2.0, dims.depth / 2.0];
            Some(([-half[0], -half[1], -half[2]], half))
        }
        "hoop" => {
            let ring = dims.outer_radius + dims.inner_radius;
            Some((
                [-ring, -ring, -dims.inner_radius],
                [ring, ring, dims.inner_radius],
            ))
        }
        "cone" => Some((
            [-dims.radius, -dims.height / 2.0, -dims.radius],
            [dims.radius, dims.height / 2.0, dims.radius],
        )),
        "sphere" => {
            let radius = dims.radius;
            Some(([-radius, -radius, -radius], [radius, radius, radius]))
        }
        _ => None,
    }
}

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
        let (local_min, local_max) = match local_bounds(&obstacle.obstacle_type) {
            Some(bounds) => bounds,
            None => return Aabb { min: obstacle.position, max: obstacle.position },
        };
        let matrix = rotation_matrix_xyz(
            obstacle.rotation[0],
            obstacle.rotation[1],
            obstacle.rotation[2],
        );
        let corners = [
            [local_min[0], local_min[1], local_min[2]],
            [local_min[0], local_min[1], local_max[2]],
            [local_min[0], local_max[1], local_min[2]],
            [local_min[0], local_max[1], local_max[2]],
            [local_max[0], local_min[1], local_min[2]],
            [local_max[0], local_min[1], local_max[2]],
            [local_max[0], local_max[1], local_min[2]],
            [local_max[0], local_max[1], local_max[2]],
        ];
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for corner in corners {
            let scaled = [
                corner[0] * obstacle.scale[0],
                corner[1] * obstacle.scale[1],
                corner[2] * obstacle.scale[2],
            ];
            let world = [
                matrix[0][0] * scaled[0] + matrix[0][1] * scaled[1] + matrix[0][2] * scaled[2]
                    + obstacle.position[0],
                matrix[1][0] * scaled[0] + matrix[1][1] * scaled[1] + matrix[1][2] * scaled[2]
                    + obstacle.position[1],
                matrix[2][0] * scaled[0] + matrix[2][1] * scaled[1] + matrix[2][2] * scaled[2]
                    + obstacle.position[2],
            ];
            for axis in 0..3 {
                if world[axis] < min[axis] {
                    min[axis] = world[axis];
                }
                if world[axis] > max[axis] {
                    max[axis] = world[axis];
                }
            }
        }
        Aabb { min, max }
    }

    pub fn check_collision(
        &self,
        sim_x: f64,
        sim_y: f64,
        sim_z: f64,
        drone_size: f64,
    ) -> Option<CollisionHit> {
        let half = drone_size / 2.0;
        let probe = [sim_x, sim_z, sim_y];
        'obstacles: for obstacle in &self.obstacles {
            if type_dims(&obstacle.obstacle_type).is_none() {
                continue;
            }
            let world_box = self.world_box(obstacle);
            for axis in 0..3 {
                if probe[axis] + half < world_box.min[axis]
                    || probe[axis] - half > world_box.max[axis]
                {
                    continue 'obstacles;
                }
            }
            let dx = probe[0] - obstacle.position[0];
            let dy = probe[1] - obstacle.position[1];
            let dz = probe[2] - obstacle.position[2];
            return Some(CollisionHit {
                obstacle: obstacle.clone(),
                world_box,
                distance: (dx * dx + dy * dy + dz * dz).sqrt(),
            });
        }
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
        for mut obstacle in obstacles {
            if self.within_boundary(obstacle.position) {
                if obstacle.name.is_empty() {
                    obstacle.name = format!("{} {}", obstacle.obstacle_type, accepted.len() + 1);
                }
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

const OBSTACLE_TYPE_NAMES: [&str; 6] = ["wall", "tower", "hoop", "cone", "square", "sphere"];

struct ImportedDef {
    obstacle_type: String,
    position: Vec3,
    rotation: Vec3,
    scale: Option<Vec3>,
    name: Option<String>,
    color: Option<String>,
}

fn js_number(text: &str) -> Option<f64> {
    if let Some(digits) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        if digits.is_empty() {
            return None;
        }
        return i128::from_str_radix(digits, 16).ok().map(|value| value as f64);
    }
    if let Some(digits) = text.strip_prefix("0b").or_else(|| text.strip_prefix("0B")) {
        if digits.is_empty() {
            return None;
        }
        return i128::from_str_radix(digits, 2).ok().map(|value| value as f64);
    }
    if let Some(digits) = text.strip_prefix("0o").or_else(|| text.strip_prefix("0O")) {
        if digits.is_empty() {
            return None;
        }
        return i128::from_str_radix(digits, 8).ok().map(|value| value as f64);
    }
    text.parse::<f64>().ok()
}

fn maybe_num_str(text: &str) -> Option<f64> {
    if text.is_empty() {
        return None;
    }
    let trimmed = text.trim();
    let value = if trimmed.is_empty() {
        0.0
    } else {
        js_number(trimmed)?
    };
    if value.is_finite() {
        Some(value)
    } else {
        None
    }
}

fn js_number_text(number: &serde_json::Number) -> String {
    if let Some(value) = number.as_i64() {
        value.to_string()
    } else if let Some(value) = number.as_u64() {
        value.to_string()
    } else if let Some(value) = number.as_f64() {
        let text = value.to_string();
        match text.strip_suffix(".0") {
            Some(stripped) => stripped.to_string(),
            None => text,
        }
    } else {
        number.to_string()
    }
}

fn js_stringify(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::Null => String::new(),
        serde_json::Value::Bool(flag) => flag.to_string(),
        serde_json::Value::Number(number) => js_number_text(number),
        serde_json::Value::String(text) => text.clone(),
        serde_json::Value::Array(items) => items
            .iter()
            .map(js_stringify)
            .collect::<Vec<String>>()
            .join(","),
        serde_json::Value::Object(_) => "[object Object]".to_string(),
    }
}

fn json_num(value: Option<&serde_json::Value>) -> Option<f64> {
    match value {
        None | Some(serde_json::Value::Null) => None,
        Some(serde_json::Value::String(text)) => maybe_num_str(text),
        Some(serde_json::Value::Number(number)) => {
            let value = number.as_f64()?;
            if value.is_finite() {
                Some(value)
            } else {
                None
            }
        }
        Some(serde_json::Value::Bool(flag)) => Some(if *flag { 1.0 } else { 0.0 }),
        Some(serde_json::Value::Array(items)) => {
            let text = items
                .iter()
                .map(js_stringify)
                .collect::<Vec<String>>()
                .join(",");
            let trimmed = text.trim();
            let value = if trimmed.is_empty() {
                0.0
            } else {
                js_number(trimmed)?
            };
            if value.is_finite() {
                Some(value)
            } else {
                None
            }
        }
        _ => None,
    }
}

fn json_num_or(value: Option<&serde_json::Value>, fallback: f64) -> f64 {
    json_num(value).unwrap_or(fallback)
}

fn json_present(obj: &serde_json::Map<String, serde_json::Value>, key: &str) -> bool {
    !matches!(obj.get(key), None | Some(serde_json::Value::Null))
}

fn json_truthy(value: Option<&serde_json::Value>) -> bool {
    match value {
        Some(serde_json::Value::Null) | None => false,
        Some(serde_json::Value::Bool(flag)) => *flag,
        Some(serde_json::Value::Number(number)) => {
            number.as_f64().map_or(false, |n| n != 0.0)
        }
        Some(serde_json::Value::String(text)) => !text.is_empty(),
        Some(_) => true,
    }
}

fn norm_type(value: Option<&str>) -> String {
    match value {
        Some(text) => {
            let lower = text.to_lowercase();
            if OBSTACLE_TYPE_NAMES.contains(&lower.as_str()) {
                lower
            } else {
                "wall".to_string()
            }
        }
        None => "wall".to_string(),
    }
}

fn norm_rotation(value: Option<&serde_json::Value>) -> Vec3 {
    match value {
        Some(serde_json::Value::Array(items)) => [
            json_num_or(items.first(), 0.0),
            json_num_or(items.get(1), 0.0),
            json_num_or(items.get(2), 0.0),
        ],
        _ => [0.0, 0.0, 0.0],
    }
}

fn value_to_string(value: &serde_json::Value) -> String {
    match value {
        serde_json::Value::String(text) => text.clone(),
        serde_json::Value::Number(number) => number.to_string(),
        serde_json::Value::Bool(flag) => flag.to_string(),
        other => serde_json::to_string(other).unwrap_or_default(),
    }
}

fn json_color(value: Option<&serde_json::Value>) -> Option<String> {
    match value {
        Some(serde_json::Value::Null) | None => None,
        Some(serde_json::Value::Number(number)) => {
            let raw = number.as_f64()?;
            if !raw.is_finite() {
                return None;
            }
            let packed = (raw as i64 as u64) & 0x00ff_ffff;
            Some(format!("#{packed:06x}"))
        }
        Some(value) => Some(value_to_string(value)),
    }
}

fn coerce_name(obj: &serde_json::Map<String, serde_json::Value>) -> Option<String> {
    if let Some(serde_json::Value::String(text)) = obj.get("name") {
        if !text.trim().is_empty() {
            return Some(text.clone());
        }
    }
    match obj.get("label") {
        Some(serde_json::Value::Null) | None => None,
        Some(value) => {
            let text = value_to_string(value);
            if text.is_empty() {
                None
            } else {
                Some(text)
            }
        }
    }
}

fn importer_dims(obstacle_type: &str) -> (f64, f64, f64, f64) {
    match obstacle_type {
        "tower" => (0.3, 1.5, 0.3, 0.0),
        "hoop" => (0.5, 1.0, 0.5, 0.0),
        "cone" => (0.4, 0.4, 0.4, 0.0),
        "square" => (0.5, 1.0, 2.0, 0.0),
        "sphere" => (0.5, 0.5, 0.5, 0.25),
        _ => (0.5, 1.0, 2.0, 0.0),
    }
}

fn json_bbox_dims(obstacle_type: &str) -> (f64, f64, f64) {
    if obstacle_type == "sphere" {
        (f64::NAN, f64::NAN, f64::NAN)
    } else {
        let (dim_w, dim_h, dim_d, _) = importer_dims(obstacle_type);
        (dim_w, dim_h, dim_d)
    }
}

struct GroundedSpec {
    height: Option<f64>,
    width: Option<f64>,
    depth: Option<f64>,
    radius: Option<f64>,
    name: Option<String>,
    rotation: Option<Vec3>,
    y: Option<f64>,
    color: Option<String>,
}

fn grounded(obstacle_type: &str, x: f64, z: f64, spec: GroundedSpec) -> ImportedDef {
    let obstacle_type = norm_type(Some(obstacle_type));
    if obstacle_type == "hoop" {
        let height = spec.height.unwrap_or(1.0);
        let rotation = spec
            .rotation
            .unwrap_or([std::f64::consts::FRAC_PI_2, 0.0, 0.0]);
        let y = spec.y.unwrap_or(height);
        return ImportedDef {
            obstacle_type,
            position: [x, y, z],
            rotation,
            scale: Some([1.0, 1.0, 1.0]),
            name: spec.name.filter(|name| !name.is_empty()),
            color: spec.color,
        };
    }
    let (dim_w, dim_h, dim_d, dim_r) = importer_dims(&obstacle_type);
    let mut height = spec.height.unwrap_or(dim_h);
    let mut width = spec.width;
    let mut depth = spec.depth;
    if obstacle_type == "cone" {
        if let Some(radius) = spec.radius {
            width = Some(radius * 2.0);
            depth = Some(radius * 2.0);
            if spec.height.is_none() {
                height = radius * 2.0;
            }
        }
    }
    if obstacle_type == "sphere" {
        let radius = spec.radius.unwrap_or(dim_r);
        width = Some(radius * 2.0);
        depth = Some(radius * 2.0);
        if spec.height.is_none() {
            height = radius * 2.0;
        }
    }
    let width = width.unwrap_or(dim_w);
    let depth = depth.unwrap_or(dim_d);
    let width = if width <= 0.0 { dim_w } else { width };
    let height = if height <= 0.0 { dim_h } else { height };
    let depth = if depth <= 0.0 { dim_d } else { depth };
    let y = spec.y.unwrap_or(height / 2.0);
    let scale = if obstacle_type == "sphere" {
        [
            width / (dim_r * 2.0),
            height / (dim_r * 2.0),
            depth / (dim_r * 2.0),
        ]
    } else {
        [width / dim_w, height / dim_h, depth / dim_d]
    };
    ImportedDef {
        obstacle_type,
        position: [x, y, z],
        rotation: spec.rotation.unwrap_or([0.0, 0.0, 0.0]),
        scale: Some(scale),
        name: spec.name.filter(|name| !name.is_empty()),
        color: spec.color,
    }
}

fn coerce_json_def(item: &serde_json::Value) -> Option<ImportedDef> {
    let obj = item.as_object()?;
    let has_bbox = ["cx", "cy", "cz", "width", "height", "depth", "radius"]
        .iter()
        .any(|key| json_present(obj, key));
    let mut obstacle_type = if json_truthy(obj.get("type")) {
        norm_type(obj.get("type").and_then(serde_json::Value::as_str))
    } else {
        "wall".to_string()
    };
    let mut scale: Option<Vec3> = match obj.get("scale") {
        Some(serde_json::Value::Array(items)) => Some([
            json_num_or(items.first(), 1.0),
            json_num_or(items.get(1), 1.0),
            json_num_or(items.get(2), 1.0),
        ]),
        _ => None,
    };
    let mut position: Option<Vec3> = None;
    if let Some(serde_json::Value::Array(items)) = obj.get("position") {
        if items.len() >= 3 {
            position = Some([
                json_num_or(items.first(), 0.0),
                json_num_or(items.get(1), 0.0),
                json_num_or(items.get(2), 0.0),
            ]);
        }
    }
    if position.is_none()
        && (json_present(obj, "x") || json_present(obj, "y") || json_present(obj, "z"))
    {
        position = Some([
            json_num_or(obj.get("x"), 0.0),
            json_num_or(obj.get("y"), 0.0),
            json_num_or(obj.get("z"), 0.0),
        ]);
    }
    if has_bbox {
        let cx = json_num(obj.get("cx")).unwrap_or_else(|| json_num_or(obj.get("x"), 0.0));
        let cz = json_num(obj.get("cz")).unwrap_or_else(|| json_num_or(obj.get("z"), 0.0));
        let cy_explicit = json_num(obj.get("cy")).or_else(|| json_num(obj.get("y")));
        let w = json_num(obj.get("width"));
        let h = json_num(obj.get("height"));
        let d = json_num(obj.get("depth"));
        let r = json_num(obj.get("radius"));
        if let Some(r) = r {
            obstacle_type = if json_truthy(obj.get("type")) {
                norm_type(obj.get("type").and_then(serde_json::Value::as_str))
            } else {
                "cone".to_string()
            };
            let hh = h.unwrap_or(r * 2.0);
            let cy = cy_explicit.unwrap_or(hh / 2.0);
            position = Some([cx, cy, cz]);
            scale = Some([r * 2.0 / 0.4, hh / 0.4, r * 2.0 / 0.4]);
        } else {
            if !json_truthy(obj.get("type")) {
                obstacle_type = if h.is_some()
                    && w.is_some()
                    && d.is_some()
                    && h.unwrap() > w.unwrap()
                    && h.unwrap() > d.unwrap()
                {
                    "tower".to_string()
                } else {
                    "wall".to_string()
                };
            }
            let (dim_w, dim_h, dim_d) = json_bbox_dims(&obstacle_type);
            let cy = cy_explicit.unwrap_or(h.unwrap_or(dim_h) / 2.0);
            position = Some([cx, cy, cz]);
            scale = Some([
                w.unwrap_or(dim_w) / dim_w,
                h.unwrap_or(dim_h) / dim_h,
                d.unwrap_or(dim_d) / dim_d,
            ]);
        }
    }
    Some(ImportedDef {
        obstacle_type,
        position: position.unwrap_or([0.0, 0.0, 0.0]),
        rotation: norm_rotation(obj.get("rotation")),
        scale,
        name: coerce_name(obj),
        color: json_color(obj.get("color")),
    })
}

fn finish(defs: Vec<ImportedDef>) -> Vec<Obstacle> {
    defs.into_iter()
        .enumerate()
        .map(|(index, def)| Obstacle {
            id: format!("obs_{index}"),
            obstacle_type: def.obstacle_type,
            position: def.position,
            rotation: def.rotation,
            scale: def.scale.unwrap_or([1.0, 1.0, 1.0]),
            name: def.name.unwrap_or_default(),
            color: def.color,
        })
        .collect()
}

fn normalize_json_list(list: &serde_json::Value) -> Vec<ImportedDef> {
    match list {
        serde_json::Value::Array(items) => items.iter().filter_map(coerce_json_def).collect(),
        _ => Vec::new(),
    }
}

pub fn import_json(text: &str) -> Result<Vec<Obstacle>, String> {
    let data: serde_json::Value = serde_json::from_str(text).map_err(|error| error.to_string())?;
    let message =
        "JSON must be an array of obstacles or an object with an \"obstacles\" array".to_string();
    match &data {
        serde_json::Value::Array(_) => Ok(finish(normalize_json_list(&data))),
        serde_json::Value::Object(obj) => {
            if matches!(obj.get("obstacles"), Some(serde_json::Value::Array(_))) {
                return Ok(finish(normalize_json_list(&obj["obstacles"])));
            }
            match obj.get("type").and_then(serde_json::Value::as_str) {
                Some("FeatureCollection") | Some("Feature") => {
                    let features = extract_features(&data)?;
                    Ok(finish(feature_defs(&features, 1.0)))
                }
                _ => Err(message),
            }
        }
        _ => Err(message),
    }
}

fn extract_features(data: &serde_json::Value) -> Result<Vec<serde_json::Value>, String> {
    if let serde_json::Value::Array(items) = data {
        return Ok(items.clone());
    }
    let obj = data
        .as_object()
        .ok_or_else(|| "Not a valid GeoJSON document".to_string())?;
    let doc_type = obj.get("type").and_then(serde_json::Value::as_str);
    if doc_type == Some("FeatureCollection") {
        if let Some(serde_json::Value::Array(features)) = obj.get("features") {
            return Ok(features.clone());
        }
    }
    if doc_type == Some("Feature") {
        return Ok(vec![data.clone()]);
    }
    if doc_type == Some("GeometryCollection") {
        if let Some(serde_json::Value::Array(geometries)) = obj.get("geometries") {
            return Ok(geometries
                .iter()
                .map(|geometry| {
                    serde_json::json!({"type": "Feature", "properties": {}, "geometry": geometry})
                })
                .collect());
        }
    }
    if let Some(serde_json::Value::Array(features)) = obj.get("features") {
        return Ok(features.clone());
    }
    if doc_type.is_some() && obj.contains_key("coordinates") {
        return Ok(vec![serde_json::json!({
            "type": "Feature",
            "properties": obj.get("properties").cloned().unwrap_or(serde_json::json!({})),
            "geometry": data.clone()
        })]);
    }
    Err("Not a valid GeoJSON document".to_string())
}

#[derive(Debug, Clone, Copy)]
struct Extents {
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
    cx: f64,
    cz: f64,
}

fn extents_from_pairs(points: &[(f64, f64)]) -> Option<Extents> {
    if points.is_empty() {
        return None;
    }
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    let mut sum_x = 0.0;
    let mut sum_z = 0.0;
    for (x, z) in points {
        if *x < min_x {
            min_x = *x;
        }
        if *x > max_x {
            max_x = *x;
        }
        if *z < min_z {
            min_z = *z;
        }
        if *z > max_z {
            max_z = *z;
        }
        sum_x += *x;
        sum_z += *z;
    }
    let n = points.len() as f64;
    Some(Extents {
        min_x,
        max_x,
        min_z,
        max_z,
        cx: sum_x / n,
        cz: sum_z / n,
    })
}

fn ring_extents(coords: Option<&serde_json::Value>) -> Option<Extents> {
    let points = coords?.as_array()?;
    let mut pairs = Vec::new();
    for point in points {
        let Some(pair) = point.as_array() else {
            continue;
        };
        let Some(x) = json_num(pair.first()) else {
            continue;
        };
        let Some(z) = json_num(pair.get(1)) else {
            continue;
        };
        pairs.push((x, z));
    }
    extents_from_pairs(&pairs)
}

fn collect_points(coords: Option<&serde_json::Value>, out: &mut Vec<(f64, f64)>) {
    let Some(serde_json::Value::Array(items)) = coords else {
        return;
    };
    if items.len() >= 2 && items[0].is_number() && items[1].is_number() {
        if let (Some(x), Some(z)) = (json_num(items.first()), json_num(items.get(1))) {
            out.push((x, z));
        }
        return;
    }
    for item in items {
        collect_points(Some(item), out);
    }
}

fn feature_defs(features: &[serde_json::Value], scale: f64) -> Vec<ImportedDef> {
    features
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| feature_to_def(feature, index, scale))
        .collect()
}

fn feature_to_def(
    feature: &serde_json::Value,
    index: usize,
    scale: f64,
) -> Option<ImportedDef> {
    let geom_value = match feature.get("geometry") {
        Some(serde_json::Value::Null) | None => feature,
        Some(geometry) => geometry,
    };
    let geom = geom_value.as_object()?;
    let geom_type = geom.get("type").and_then(serde_json::Value::as_str)?;
    let props = feature.get("properties").and_then(serde_json::Value::as_object);
    let prop = |key: &str| props.and_then(|map| map.get(key));
    let name = prop("name")
        .filter(|value| !value.is_null())
        .map(value_to_string)
        .map(|text| text.trim().to_string())
        .filter(|text| !text.is_empty())
        .or_else(|| {
            prop("title")
                .filter(|value| !value.is_null())
                .map(value_to_string)
                .filter(|text| !text.is_empty())
        })
        .or_else(|| {
            feature
                .get("id")
                .filter(|value| !value.is_null())
                .map(value_to_string)
                .filter(|text| !text.is_empty())
        })
        .unwrap_or_else(|| format!("Geo Obstacle {}", index + 1));
    let height = json_num(prop("height"));
    let width = json_num(prop("width"));
    let depth = json_num(prop("depth"));
    let radius = json_num(prop("radius"));
    let prop_type = norm_type(prop("type").and_then(serde_json::Value::as_str));
    let color = json_color(prop("color"));
    if geom_type == "Point" {
        let default_coords = serde_json::json!([0, 0]);
        let coords = geom
            .get("coordinates")
            .filter(|value| !value.is_null())
            .unwrap_or(&default_coords);
        let x = json_num_or(coords.get(0), 0.0) * scale;
        let z = json_num_or(coords.get(1), 0.0) * scale;
        return Some(grounded(
            &prop_type,
            x,
            z,
            GroundedSpec {
                height,
                width,
                depth,
                radius,
                name: Some(name),
                rotation: None,
                y: None,
                color,
            },
        ));
    }
    if geom_type == "Polygon" {
        let ring = geom.get("coordinates").and_then(|coords| coords.get(0));
        let ext = ring_extents(ring)?;
        let x = ext.cx * scale;
        let z = ext.cz * scale;
        return Some(grounded(
            &prop_type,
            x,
            z,
            GroundedSpec {
                height,
                width: Some((ext.max_x - ext.min_x) * scale),
                depth: Some((ext.max_z - ext.min_z) * scale),
                radius,
                name: Some(name),
                rotation: None,
                y: None,
                color,
            },
        ));
    }
    if geom_type == "LineString" {
        let ext = ring_extents(geom.get("coordinates"))?;
        let x = ext.cx * scale;
        let z = ext.cz * scale;
        let len = (ext.max_x - ext.min_x).max(ext.max_z - ext.min_z) * scale;
        return Some(grounded(
            &prop_type,
            x,
            z,
            GroundedSpec {
                height,
                width,
                depth: Some(len.max(0.5)),
                radius,
                name: Some(name),
                rotation: None,
                y: None,
                color,
            },
        ));
    }
    let mut points = Vec::new();
    collect_points(geom.get("coordinates"), &mut points);
    let ext = extents_from_pairs(&points)?;
    let x = ext.cx * scale;
    let z = ext.cz * scale;
    let span_x = (ext.max_x - ext.min_x) * scale;
    let span_z = (ext.max_z - ext.min_z) * scale;
    Some(grounded(
        &prop_type,
        x,
        z,
        GroundedSpec {
            height,
            width: if span_x > 0.0 { Some(span_x) } else { None },
            depth: if span_z > 0.0 { Some(span_z) } else { None },
            radius,
            name: Some(name),
            rotation: None,
            y: None,
            color,
        },
    ))
}

pub fn import_geojson(text: &str) -> Result<Vec<Obstacle>, String> {
    let data: serde_json::Value = serde_json::from_str(text).map_err(|error| error.to_string())?;
    let features = extract_features(&data)?;
    Ok(finish(feature_defs(&features, 1.0)))
}

fn parse_csv_rows(text: &str) -> Vec<Vec<String>> {
    let mut rows = Vec::new();
    for line in text.split(|character| character == '\r' || character == '\n') {
        if line.trim().is_empty() {
            continue;
        }
        let mut cells = Vec::new();
        let mut current = String::new();
        let mut in_quotes = false;
        let mut chars = line.char_indices().peekable();
        while let Some((_, character)) = chars.next() {
            if in_quotes {
                if character == '"' {
                    match chars.peek() {
                        Some((_, '"')) => {
                            current.push('"');
                            chars.next();
                        }
                        _ => in_quotes = false,
                    }
                } else {
                    current.push(character);
                }
            } else if character == '"' {
                in_quotes = true;
            } else if character == ',' {
                cells.push(std::mem::take(&mut current));
            } else {
                current.push(character);
            }
        }
        cells.push(current);
        rows.push(cells);
    }
    rows
}

fn csv_row_to_def(header: &[String], row: &[String]) -> Option<ImportedDef> {
    let get = |key: &str| -> Option<&String> {
        header
            .iter()
            .position(|column| column == key)
            .and_then(|index| row.get(index))
    };
    let cell_num = |key: &str| get(key).and_then(|cell| maybe_num_str(cell));
    let type_raw = get("type")
        .map(|cell| cell.trim().to_lowercase())
        .unwrap_or_default();
    let obstacle_type = if OBSTACLE_TYPE_NAMES.contains(&type_raw.as_str()) {
        type_raw
    } else {
        "wall".to_string()
    };
    let name = get("name")
        .map(|cell| cell.trim().to_string())
        .filter(|trimmed| !trimmed.is_empty());
    let rotation = [
        cell_num("rotation_x").unwrap_or(0.0),
        cell_num("rotation_y").unwrap_or(0.0),
        cell_num("rotation_z").unwrap_or(0.0),
    ];
    Some(grounded(
        &obstacle_type,
        cell_num("x").unwrap_or(0.0),
        cell_num("z").unwrap_or(0.0),
        GroundedSpec {
            height: cell_num("height"),
            width: cell_num("width"),
            depth: cell_num("depth"),
            radius: cell_num("radius"),
            name,
            rotation: Some(rotation),
            y: cell_num("y").or_else(|| cell_num("altitude")),
            color: get("color").cloned(),
        },
    ))
}

pub fn import_csv(text: &str) -> Result<Vec<Obstacle>, String> {
    let rows = parse_csv_rows(text);
    if rows.is_empty() {
        return Ok(Vec::new());
    }
    let header: Vec<String> = rows[0]
        .iter()
        .map(|cell| cell.trim().to_lowercase())
        .collect();
    let mut defs = Vec::new();
    for row in rows.iter().skip(1) {
        if row.is_empty() || (row.len() == 1 && row[0].trim().is_empty()) {
            continue;
        }
        if let Some(def) = csv_row_to_def(&header, row) {
            defs.push(def);
        }
    }
    Ok(finish(defs))
}

fn js_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

fn vertex_bounds(vertices: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for vertex in vertices {
        for axis in 0..3 {
            let value = vertex[axis];
            if value < min[axis] {
                min[axis] = value;
            }
            if value > max[axis] {
                max[axis] = value;
            }
        }
    }
    (min, max)
}

pub fn import_obj(text: &str) -> Result<Vec<Obstacle>, String> {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut current_name: Option<String> = None;
    let mut face_count = 0usize;
    for raw in text.split(|character| character == '\r' || character == '\n') {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let tag = line.chars().next().unwrap_or(' ');
        if tag == 'v' {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.first() == Some(&"v") && parts.len() >= 4 {
                vertices.push([
                    crate::commands::js_parse_float(parts[1]).unwrap_or(f64::NAN),
                    crate::commands::js_parse_float(parts[2]).unwrap_or(f64::NAN),
                    crate::commands::js_parse_float(parts[3]).unwrap_or(f64::NAN),
                ]);
            }
        } else if tag == 'o' || tag == 'g' {
            let name = line.split_whitespace().skip(1).collect::<Vec<&str>>().join(" ");
            current_name = if name.is_empty() { None } else { Some(name) };
        } else if tag == 'f' {
            face_count += 1;
        }
    }
    if vertices.is_empty() {
        return Ok(Vec::new());
    }
    let mut defs = Vec::new();
    if face_count > 0 || vertices.len() > 200 {
        let (min, max) = vertex_bounds(&vertices);
        let cx = (min[0] + max[0]) / 2.0;
        let cy = (min[1] + max[1]) / 2.0;
        let cz = (min[2] + max[2]) / 2.0;
        let w = js_max(max[0] - min[0], 0.001);
        let h = js_max(max[1] - min[1], 0.001);
        let d = js_max(max[2] - min[2], 0.001);
        let obstacle_type = if h > w && h > d { "tower" } else { "wall" };
        let (dim_w, dim_h, dim_d, _) = importer_dims(obstacle_type);
        defs.push(ImportedDef {
            obstacle_type: obstacle_type.to_string(),
            position: [cx, cy, cz],
            rotation: [0.0, 0.0, 0.0],
            scale: Some([w / dim_w, h / dim_h, d / dim_d]),
            name: current_name,
            color: None,
        });
    } else {
        for (index, vertex) in vertices.iter().enumerate() {
            defs.push(ImportedDef {
                obstacle_type: "wall".to_string(),
                position: *vertex,
                rotation: [0.0, 0.0, 0.0],
                scale: Some([1.0, 1.0, 1.0]),
                name: Some(
                    current_name
                        .clone()
                        .unwrap_or_else(|| format!("OBJ Obstacle {}", index + 1)),
                ),
                color: None,
            });
        }
    }
    Ok(finish(defs))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_obstacle(obstacle_type: &str, position: Vec3, rotation: Vec3) -> Obstacle {
        Obstacle {
            id: format!("t_{obstacle_type}"),
            obstacle_type: obstacle_type.to_string(),
            position,
            rotation,
            scale: [1.0, 1.0, 1.0],
            name: obstacle_type.to_string(),
            color: None,
        }
    }

    #[test]
    fn world_box_anchor_boxes() {
        let set = ObstacleSet::new();
        let cases = [
            (
                "wall",
                [1.0, 0.5, 0.0],
                [0.0, 0.0, 0.0],
                [0.75, 0.0, -1.0],
                [1.25, 1.0, 1.0],
            ),
            (
                "wall",
                [0.0, 0.5, 2.0],
                [0.0, std::f64::consts::FRAC_PI_2, 0.0],
                [-1.0, 0.0, 1.75],
                [1.0, 1.0, 2.25],
            ),
            (
                "hoop",
                [2.0, 1.2, 2.0],
                [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
                [1.1, 0.8, 1.1],
                [2.9, 1.6, 2.9],
            ),
            (
                "cone",
                [-2.0, 0.2, 2.0],
                [0.0, 0.0, 0.0],
                [-2.2, 0.0, 1.8],
                [-1.8, 0.4, 2.2],
            ),
            (
                "sphere",
                [-2.0, 1.0, -2.0],
                [0.0, 0.0, 0.0],
                [-2.25, 0.75, -2.25],
                [-1.75, 1.25, -1.75],
            ),
        ];
        for (obstacle_type, position, rotation, expected_min, expected_max) in cases {
            let world_box = set.world_box(&test_obstacle(obstacle_type, position, rotation));
            for axis in 0..3 {
                assert!(
                    (world_box.min[axis] - expected_min[axis]).abs() <= 1e-9,
                    "{obstacle_type} min axis {axis} got {} want {}",
                    world_box.min[axis],
                    expected_min[axis]
                );
                assert!(
                    (world_box.max[axis] - expected_max[axis]).abs() <= 1e-9,
                    "{obstacle_type} max axis {axis} got {} want {}",
                    world_box.max[axis],
                    expected_max[axis]
                );
            }
        }
    }

    #[test]
    fn world_box_rotated_primitives_match_set_from_object() {
        let set = ObstacleSet::new();
        let cases = [
            (
                "sphere",
                [1.0, 0.5, -0.4],
                [0.3, 0.7, 0.1],
                [0.629601085, 0.139403342, -0.817258660],
                [1.370398915, 0.860596658, 0.017258660],
            ),
            (
                "cone",
                [-2.0, 0.2, 2.0],
                [0.6, 0.3, 0.0],
                [-2.250171343, -0.106324491, 1.680596185],
                [-1.749828657, 0.506324491, 2.319403815],
            ),
            (
                "hoop",
                [0.0, 1.5, 0.0],
                [0.4, 0.3, 0.2],
                [-1.131688540, 0.293139333, -0.914589019],
                [1.131688540, 2.706860667, 0.914589019],
            ),
        ];
        for (obstacle_type, position, rotation, expected_min, expected_max) in cases {
            let world_box = set.world_box(&test_obstacle(obstacle_type, position, rotation));
            for axis in 0..3 {
                assert!(
                    (world_box.min[axis] - expected_min[axis]).abs() <= 1e-6,
                    "{obstacle_type} min axis {axis} got {} want {}",
                    world_box.min[axis],
                    expected_min[axis]
                );
                assert!(
                    (world_box.max[axis] - expected_max[axis]).abs() <= 1e-6,
                    "{obstacle_type} max axis {axis} got {} want {}",
                    world_box.max[axis],
                    expected_max[axis]
                );
            }
        }
    }

    #[test]
    fn check_collision_hits_wall_probe() {
        let mut set = ObstacleSet::new();
        set.obstacles.push(test_obstacle("wall", [1.0, 0.5, 0.0], [0.0, 0.0, 0.0]));
        let hit = set
            .check_collision(0.7, 0.0, 0.8189221476226725, COLLISION_DRONE_SIZE)
            .expect("probe must hit WallX");
        assert_eq!(hit.obstacle.id, "t_wall");
        assert!((hit.world_box.min[0] - 0.75).abs() <= 1e-9);
        assert!((hit.world_box.max[0] - 1.25).abs() <= 1e-9);
        assert!(set
            .check_collision(0.6, 0.0, 0.8189221476226725, COLLISION_DRONE_SIZE)
            .is_none());
        assert!(set
            .check_collision(0.7, 5.0, 0.8189221476226725, COLLISION_DRONE_SIZE)
            .is_none());
        assert!(set.check_collision(0.7, 0.0, 0.8189221476226725, 0.0).is_none());
    }

    #[test]
    fn import_roundtrip_keeps_and_rejects() {
        let manifest = env!("CARGO_MANIFEST_DIR");
        let path = format!("{manifest}/../../tests/golden/golden.json");
        let golden = crate::golden::GoldenFile::load(&path).expect("golden loads");
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
        for (ours, expected) in set.obstacles.iter().zip(golden.obstacles.obstacles.iter()) {
            assert_eq!(ours.id, expected.id, "roundtrip id");
            assert_eq!(ours.obstacle_type, expected.obstacle_type, "roundtrip type");
            assert_eq!(ours.position, expected.position, "roundtrip position");
            assert_eq!(ours.rotation, expected.rotation, "roundtrip rotation");
            assert_eq!(ours.scale, expected.scale, "roundtrip scale");
            assert_eq!(ours.name, expected.name, "roundtrip name");
        }
    }

    #[test]
    fn import_json_supports_arrays_and_files() {
        let obstacles = import_json(r#"[{"type":"tower","position":[1,2,3]}]"#).expect("array");
        assert_eq!(obstacles.len(), 1);
        assert_eq!(obstacles[0].obstacle_type, "tower");
        assert_eq!(obstacles[0].position, [1.0, 2.0, 3.0]);
        assert_eq!(obstacles[0].scale, [1.0, 1.0, 1.0]);
        assert_eq!(obstacles[0].name, "");
        let mut set = ObstacleSet::new();
        set.import(obstacles);
        assert_eq!(set.obstacles[0].name, "tower 1");
        let obstacles = import_json(
            r#"{"obstacles":[{"type":"cone","position":[0,1,0],"radius":0.3}]}"#,
        )
        .expect("file");
        assert_eq!(obstacles[0].position, [0.0, 0.3, 0.0]);
        for value in obstacles[0].scale {
            assert!((value - 1.5).abs() <= 1e-12, "cone scale {value}");
        }
        assert!(import_json("\"nope\"").is_err());
        assert!(import_json(r#"{"type":"wall","position":[0,0,0]}"#).is_err());
    }

    #[test]
    fn import_geojson_grounds_features() {
        let text = r#"{"type":"FeatureCollection","features":[
            {"type":"Feature","properties":{"name":"P1"},"geometry":{"type":"Point","coordinates":[1,2]}},
            {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[4,0]]}}
        ]}"#;
        let obstacles = import_geojson(text).expect("geojson import");
        assert_eq!(obstacles.len(), 2);
        assert_eq!(obstacles[0].name, "P1");
        assert_eq!(obstacles[0].obstacle_type, "wall");
        assert_eq!(obstacles[0].position, [1.0, 0.5, 2.0]);
        assert_eq!(obstacles[1].name, "Geo Obstacle 2");
        assert_eq!(obstacles[1].position, [2.0, 0.5, 0.0]);
        assert_eq!(obstacles[1].scale, [1.0, 1.0, 2.0]);
    }

    #[test]
    fn import_csv_rows_to_grounded() {
        let text = "type,x,z,height,name\nwall,1,2,0.5,Custom\n\nbogus,3,4,,\n";
        let obstacles = import_csv(text).expect("csv import");
        assert_eq!(obstacles.len(), 2);
        assert_eq!(obstacles[0].name, "Custom");
        assert_eq!(obstacles[0].position, [1.0, 0.25, 2.0]);
        assert_eq!(obstacles[1].name, "");
        assert_eq!(obstacles[1].position, [3.0, 0.5, 4.0]);
        let mut set = ObstacleSet::new();
        set.import(obstacles);
        assert_eq!(set.obstacles[1].name, "wall 2");
    }

    #[test]
    fn import_auto_names_number_by_accepted_index() {
        let defs = import_json(
            r#"[{"type":"tower","position":[99,1,0]},{"type":"cone","position":[0,1,0]},{"type":"sphere","position":[1,1,0]}]"#,
        )
        .expect("parse");
        let mut set = ObstacleSet::new();
        assert_eq!(set.import(defs), 1);
        let names: Vec<&str> = set
            .obstacles
            .iter()
            .map(|obstacle| obstacle.name.as_str())
            .collect();
        assert_eq!(names, ["cone 1", "sphere 2"]);
    }

    #[test]
    fn import_empty_label_falls_back_to_auto_name() {
        let obstacles = import_json(r#"[{"type":"cone","position":[0,1,0],"label":""}]"#)
            .expect("parse");
        assert_eq!(obstacles[0].name, "");
        let mut set = ObstacleSet::new();
        set.import(obstacles);
        assert_eq!(set.obstacles[0].name, "cone 1");
    }

    #[test]
    fn import_json_string_numbers_use_js_number() {
        let obstacles = import_json(
            r#"[{"type":"tower","position":["1.5","2","3"]},{"type":"cone","position":["0x10",1,0]}]"#,
        )
        .expect("parse");
        assert_eq!(obstacles[0].position, [1.5, 2.0, 3.0]);
        assert_eq!(obstacles[1].position[0], 16.0);
        let mut set = ObstacleSet::new();
        assert_eq!(set.import(obstacles), 1);
    }

    #[test]
    fn import_geojson_empty_title_falls_back_to_id() {
        let text = r#"{"type":"FeatureCollection","features":[
            {"type":"Feature","id":"ID9","properties":{"title":""},"geometry":{"type":"Point","coordinates":[0,0]}}
        ]}"#;
        let obstacles = import_geojson(text).expect("geojson import");
        assert_eq!(obstacles[0].name, "ID9");
    }

    #[test]
    fn import_json_numeric_color_normalizes_to_hex() {
        let obstacles = import_json(r#"[{"type":"cone","position":[0,1,0],"color":4889704}]"#)
            .expect("parse");
        assert_eq!(obstacles[0].color.as_deref(), Some("#4a9c68"));
    }

    #[test]
    fn import_obj_faces_and_points() {
        let text = "o Body\nv 0 0 0\nv 2 0 0\nv 2 1 0\nv 0 1 0\nf 1 2 3 4\n";
        let obstacles = import_obj(text).expect("obj faces");
        assert_eq!(obstacles.len(), 1);
        assert_eq!(obstacles[0].name, "Body");
        assert_eq!(obstacles[0].position, [1.0, 0.5, 0.0]);
        assert_eq!(obstacles[0].scale, [4.0, 1.0, 0.0005]);
        let obstacles = import_obj("v 0 0 0\nv 1 1 1\n").expect("obj points");
        assert_eq!(obstacles.len(), 2);
        assert_eq!(obstacles[1].name, "OBJ Obstacle 2");
        assert_eq!(obstacles[1].position, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn json_bbox_sphere_without_radius_rejected() {
        let defs = import_json(r#"{"obstacles":[{"type":"sphere","cx":1,"cz":1}]}"#).expect("parse");
        assert_eq!(defs.len(), 1);
        assert!(defs[0].position[1].is_nan(), "sphere cy must be NaN like JS DEFAULT_DIMS");
        assert!(defs[0].scale[0].is_nan(), "sphere scale must be NaN");
        let mut set = ObstacleSet::new();
        set.boundary = Boundary::default();
        let rejected = set.import(defs);
        assert_eq!(rejected, 1, "NaN position must be boundary-rejected");
        assert!(set.obstacles.is_empty());
        let kept = import_json(
            r#"{"obstacles":[{"type":"sphere","cx":0,"cy":2,"cz":0}]}"#,
        )
        .expect("explicit cy");
        let mut set = ObstacleSet::new();
        assert_eq!(set.import(kept), 0, "finite position with NaN scale is accepted like JS");
    }

    #[test]
    fn obj_malformed_vertices_rejected() {
        let defs = import_obj("o Bad\nv abc 0 0\nf 1 1 1\n").expect("obj parse");
        assert_eq!(defs.len(), 1);
        assert!(defs[0].position[0].is_nan(), "bbox center must be NaN like JS");
        let mut set = ObstacleSet::new();
        set.boundary = Boundary::default();
        let rejected = set.import(defs);
        assert_eq!(rejected, 1, "malformed vertex body must be boundary-rejected");
        assert!(set.obstacles.is_empty());
    }

    #[test]
    fn import_array_numbers_use_js_number_coercion() {
        let obstacles = import_json(r#"[{"type":"wall","position":[[5],1,0]}]"#).expect("parse");
        assert_eq!(obstacles[0].position, [5.0, 1.0, 0.0]);
        let mut set = ObstacleSet::new();
        assert_eq!(set.import(obstacles), 1);
        assert!(set.obstacles.is_empty());

        let obstacles =
            import_json(r#"[{"type":"wall","position":[0,1,0],"scale":[[2]]}]"#).expect("parse");
        assert_eq!(obstacles[0].scale, [2.0, 1.0, 1.0]);

        let obstacles =
            import_json(r#"[{"cx":0,"cz":0,"width":[3],"height":1,"depth":1}]"#).expect("parse");
        assert_eq!(obstacles[0].position, [0.0, 0.5, 0.0]);
        assert_eq!(obstacles[0].scale, [6.0, 1.0, 0.5]);

        let obstacles =
            import_json(r#"[{"type":"wall","position":[0,1,0],"rotation":[[1],2,3]}]"#)
                .expect("parse");
        assert_eq!(obstacles[0].rotation, [1.0, 2.0, 3.0]);

        let obstacles =
            import_json(r#"[{"type":"wall","position":[0,1,0],"scale":[[]]}]"#).expect("parse");
        assert_eq!(obstacles[0].scale, [0.0, 1.0, 1.0]);

        let obstacles =
            import_json(r#"[{"cx":0,"cz":0,"width":[],"height":1,"depth":1}]"#).expect("parse");
        assert_eq!(obstacles[0].scale, [0.0, 1.0, 0.5]);

        let text = r#"{"type":"Feature","properties":{"width":[3]},"geometry":{"type":"Point","coordinates":[0,0]}}"#;
        let obstacles = import_geojson(text).expect("geojson");
        assert_eq!(obstacles[0].scale, [6.0, 1.0, 1.0]);
    }

    #[test]
    fn obj_vertex_parse_float_quirks() {
        let obstacles = import_obj("v 1.5abc 0x10 +2.5\n").expect("quirk parse");
        assert_eq!(obstacles.len(), 1);
        assert_eq!(obstacles[0].position, [1.5, 0.0, 2.5]);
        assert_eq!(obstacles[0].scale, [1.0, 1.0, 1.0]);
        assert_eq!(obstacles[0].name, "OBJ Obstacle 1");
    }
}
