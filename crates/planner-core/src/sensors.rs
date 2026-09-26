use crate::obstacles::{Aabb, Obstacle, ObstacleSet, type_dims};
use crate::planio::Vec3;
use serde::{Deserialize, Serialize};

pub const COLOR_NAMES: [&str; 8] = [
    "black",
    "blue",
    "green",
    "lightblue",
    "purple",
    "red",
    "white",
    "yellow",
];

pub const DEFAULT_TEMPERATURE_C: f64 = 22.0;
pub const DEFAULT_BATTERY_PCT: f64 = 80.0;
pub const COLOR_LOOK_RANGE_M: f64 = 3.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SensorKind {
    FrontRange,
    BottomRange,
    FrontColor,
    BackColor,
    Temperature,
    Battery,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sensor {
    pub id: String,
    pub kind: SensorKind,
    pub mount_position: Vec3,
    pub facing: Vec3,
    pub range_m: f64,
    pub fov_deg: f64,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_enabled")]
    pub enabled: bool,
}

fn default_enabled() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: String,
    pub kind: SensorKind,
    pub value: String,
    pub hit: bool,
    pub distance_m: Option<f64>,
    pub hit_point: Option<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ColorTable {
    entries: Vec<(String, String)>,
}

impl ColorTable {
    pub fn new() -> ColorTable {
        ColorTable {
            entries: Vec::new(),
        }
    }

    pub fn insert(&mut self, key: &str, color: &str) {
        self.entries.push((key.to_string(), color.to_string()));
    }

    pub fn lookup(&self, key: &str) -> String {
        for (k, c) in &self.entries {
            if k == key {
                return c.clone();
            }
        }
        "Unknown".to_string()
    }
}

impl Default for ColorTable {
    fn default() -> Self {
        ColorTable::new()
    }
}

pub fn is_omnidirectional(kind: SensorKind) -> bool {
    matches!(kind, SensorKind::Temperature | SensorKind::Battery)
}

fn kind_id(kind: SensorKind) -> &'static str {
    match kind {
        SensorKind::FrontRange => "front_range",
        SensorKind::BottomRange => "bottom_range",
        SensorKind::FrontColor => "front_color",
        SensorKind::BackColor => "back_color",
        SensorKind::Temperature => "temperature",
        SensorKind::Battery => "battery",
    }
}

fn kind_name(kind: SensorKind) -> &'static str {
    match kind {
        SensorKind::FrontRange => "Front Range",
        SensorKind::BottomRange => "Bottom Range",
        SensorKind::FrontColor => "Front Color",
        SensorKind::BackColor => "Back Color",
        SensorKind::Temperature => "Temperature",
        SensorKind::Battery => "Battery",
    }
}

pub fn default_sensor(kind: SensorKind) -> Sensor {
    let (mount_position, facing, range_m, fov_deg) = match kind {
        SensorKind::FrontRange => ([0.05, 0.0, 0.0], [1.0, 0.0, 0.0], 1.5, 30.0),
        SensorKind::BottomRange => ([0.0, 0.0, 0.0], [0.0, -1.0, 0.0], 1.5, 0.0),
        SensorKind::FrontColor => ([0.05, 0.0, 0.0], [0.0, -1.0, 0.0], 0.1, 0.0),
        SensorKind::BackColor => ([-0.05, 0.0, 0.0], [0.0, -1.0, 0.0], 0.1, 0.0),
        SensorKind::Temperature => ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0),
        SensorKind::Battery => ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0),
    };
    Sensor {
        id: kind_id(kind).to_string(),
        kind,
        mount_position,
        facing,
        range_m,
        fov_deg,
        name: kind_name(kind).to_string(),
        enabled: true,
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorSet {
    #[serde(default)]
    pub sensors: Vec<Sensor>,
}

impl SensorSet {
    pub fn new() -> SensorSet {
        SensorSet {
            sensors: Vec::new(),
        }
    }

    pub fn add_default(&mut self, kind: SensorKind) -> Sensor {
        let base = kind_id(kind).to_string();
        let mut sensor = default_sensor(kind);
        let mut index = 1usize;
        while self.sensors.iter().any(|s| s.id == sensor.id) {
            index += 1;
            sensor.id = format!("{base}_{index}");
        }
        self.sensors.push(sensor.clone());
        sensor
    }

    pub fn remove(&mut self, id: &str) -> Option<Sensor> {
        let position = self.sensors.iter().position(|s| s.id == id)?;
        Some(self.sensors.remove(position))
    }

    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    pub fn from_json(text: &str) -> Result<SensorSet, serde_json::Error> {
        serde_json::from_str(text)
    }
}

impl Default for SensorSet {
    fn default() -> Self {
        SensorSet::new()
    }
}

fn rotate_local(v: Vec3, heading_deg: f64) -> Vec3 {
    let th = heading_deg.to_radians();
    let (s, c) = th.sin_cos();
    [
        v[0] * c - v[2] * s,
        v[1],
        v[0] * s + v[2] * c,
    ]
}

fn normalize(v: Vec3) -> Vec3 {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn basis(d: Vec3) -> (Vec3, Vec3) {
    let axis = if d[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = normalize(cross(axis, d));
    let v = cross(d, u);
    (u, v)
}

fn cone_directions(center: Vec3, fov_deg: f64) -> Vec<Vec3> {
    let mut dirs = vec![center];
    if fov_deg <= 0.0 {
        return dirs;
    }
    let half = (fov_deg / 2.0).to_radians();
    let (u, v) = basis(center);
    for frac in [0.5_f64, 1.0_f64] {
        let ang = half * frac;
        let (sa, ca) = ang.sin_cos();
        for az in [0.0_f64, 90.0_f64, 180.0_f64, 270.0_f64] {
            let azr = az.to_radians();
            let (saz, caz) = azr.sin_cos();
            let dir = [
                ca * center[0] + sa * (caz * u[0] + saz * v[0]),
                ca * center[1] + sa * (caz * u[1] + saz * v[1]),
                ca * center[2] + sa * (caz * u[2] + saz * v[2]),
            ];
            dirs.push(dir);
        }
    }
    dirs
}

fn ray_aabb(origin: Vec3, dir: Vec3, aabb: &Aabb) -> Option<f64> {
    let mut tmin = 0.0_f64;
    let mut tmax = f64::INFINITY;
    for axis in 0..3 {
        if dir[axis].abs() < 1e-12 {
            if origin[axis] < aabb.min[axis] || origin[axis] > aabb.max[axis] {
                return None;
            }
        } else {
            let inv = 1.0 / dir[axis];
            let mut t1 = (aabb.min[axis] - origin[axis]) * inv;
            let mut t2 = (aabb.max[axis] - origin[axis]) * inv;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            tmin = tmin.max(t1);
            tmax = tmax.min(t2);
            if tmin > tmax {
                return None;
            }
        }
    }
    Some(tmin)
}

fn evaluate_range(
    sensor: &Sensor,
    mount_world: Vec3,
    dir_world: Vec3,
    obstacles: &ObstacleSet,
) -> (bool, Option<f64>, Option<[f64; 3]>) {
    let dirs = cone_directions(dir_world, sensor.fov_deg);
    let mut best_t = f64::INFINITY;
    let mut best_hit: Option<[f64; 3]> = None;
    for d in &dirs {
        for obstacle in &obstacles.obstacles {
            if obstacle.is_pad() {
                continue;
            }
            if type_dims(&obstacle.obstacle_type).is_none() {
                continue;
            }
            let aabb = obstacles.world_box(obstacle);
            if let Some(t) = ray_aabb(mount_world, *d, &aabb) {
                if t >= 0.0 && t <= sensor.range_m && t < best_t {
                    best_t = t;
                    best_hit = Some([
                        mount_world[0] + d[0] * t,
                        mount_world[1] + d[1] * t,
                        mount_world[2] + d[2] * t,
                    ]);
                }
            }
        }
    }
    if best_hit.is_some() {
        (true, Some(best_t), best_hit)
    } else {
        (false, None, None)
    }
}

fn evaluate_color(
    mount_world: Vec3,
    obstacles: &ObstacleSet,
    surfaces: &ColorTable,
) -> (bool, String, Option<[f64; 3]>) {
    let down = [0.0, -1.0, 0.0];
    let mut best_t = f64::INFINITY;
    let mut best: Option<&Obstacle> = None;
    for obstacle in &obstacles.obstacles {
        if type_dims(&obstacle.obstacle_type).is_none() {
            continue;
        }
        let aabb = obstacles.world_box(obstacle);
        if let Some(t) = ray_aabb(mount_world, down, &aabb) {
            if t >= 0.0 && t <= COLOR_LOOK_RANGE_M && t < best_t {
                best_t = t;
                best = Some(obstacle);
            }
        }
    }
    match best {
        Some(obstacle) => {
            let mut color = surfaces.lookup(&obstacle.id);
            if color == "Unknown" {
                color = surfaces.lookup(&obstacle.obstacle_type);
            }
            let hit_point = [mount_world[0], mount_world[1] - best_t, mount_world[2]];
            (color != "Unknown", color, Some(hit_point))
        }
        None => (false, "Unknown".to_string(), None),
    }
}

pub fn format_temperature(celsius: f64) -> String {
    format!("{celsius:.1} C")
}

pub fn format_battery(percent: f64) -> String {
    format!("{percent:.1} %")
}

pub fn evaluate(
    sensor: &Sensor,
    drone_pos: Vec3,
    drone_heading_deg: f64,
    obstacles: &ObstacleSet,
    surfaces: &ColorTable,
) -> SensorReading {
    let sensor_id = sensor.id.clone();
    let kind = sensor.kind;
    let heading = drone_heading_deg;
    match kind {
        SensorKind::FrontRange | SensorKind::BottomRange => {
            let rotated = rotate_local(sensor.mount_position, heading);
            let mount_world = [
                drone_pos[0] + rotated[0],
                drone_pos[1] + rotated[1],
                drone_pos[2] + rotated[2],
            ];
            let dir_world = normalize(rotate_local(sensor.facing, heading));
            let (hit, distance_m, hit_point) =
                evaluate_range(sensor, mount_world, dir_world, obstacles);
            let value = match distance_m {
                Some(d) => format!("{d:.2} m"),
                None => "no hit".to_string(),
            };
            SensorReading {
                sensor_id,
                kind,
                value,
                hit,
                distance_m,
                hit_point,
            }
        }
        SensorKind::FrontColor | SensorKind::BackColor => {
            let rotated = rotate_local(sensor.mount_position, heading);
            let mount_world = [
                drone_pos[0] + rotated[0],
                drone_pos[1] + rotated[1],
                drone_pos[2] + rotated[2],
            ];
            let (hit, value, hit_point) = evaluate_color(mount_world, obstacles, surfaces);
            SensorReading {
                sensor_id,
                kind,
                value,
                hit,
                distance_m: None,
                hit_point,
            }
        }
        SensorKind::Temperature => SensorReading {
            sensor_id,
            kind,
            value: format_temperature(DEFAULT_TEMPERATURE_C),
            hit: false,
            distance_m: None,
            hit_point: None,
        },
        SensorKind::Battery => SensorReading {
            sensor_id,
            kind,
            value: format_battery(DEFAULT_BATTERY_PCT),
            hit: false,
            distance_m: None,
            hit_point: None,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacles::Boundary;

    fn obstacle(id: &str, obstacle_type: &str, position: Vec3, scale: Vec3, name: &str) -> Obstacle {
        Obstacle {
            id: id.to_string(),
            obstacle_type: obstacle_type.to_string(),
            position,
            rotation: [0.0, 0.0, 0.0],
            scale,
            name: name.to_string(),
            color: None,
        }
    }

    fn obs_set(obstacles: Vec<Obstacle>) -> ObstacleSet {
        ObstacleSet {
            obstacles,
            boundary: Boundary::default(),
        }
    }

    #[test]
    fn default_mounts_are_oriented() {
        let front = default_sensor(SensorKind::FrontRange);
        assert_eq!(front.facing, [1.0, 0.0, 0.0]);
        let bottom = default_sensor(SensorKind::BottomRange);
        assert_eq!(bottom.facing, [0.0, -1.0, 0.0]);
        let fcolor = default_sensor(SensorKind::FrontColor);
        assert_eq!(fcolor.facing, [0.0, -1.0, 0.0]);
        assert!(fcolor.mount_position[0] > 0.0);
        let bcolor = default_sensor(SensorKind::BackColor);
        assert_eq!(bcolor.facing, [0.0, -1.0, 0.0]);
        assert!(bcolor.mount_position[0] < 0.0);
        assert!(is_omnidirectional(SensorKind::Temperature));
        assert!(is_omnidirectional(SensorKind::Battery));
        assert!(!is_omnidirectional(SensorKind::FrontRange));
        assert!(!is_omnidirectional(SensorKind::BottomRange));
        assert!(!is_omnidirectional(SensorKind::FrontColor));
        assert!(!is_omnidirectional(SensorKind::BackColor));
    }

    #[test]
    fn range_facing_wall_hits_correct_distance() {
        let mut sensor = default_sensor(SensorKind::FrontRange);
        sensor.range_m = 10.0;
        sensor.fov_deg = 0.0;
        let set = obs_set(vec![obstacle("w1", "wall", [2.0, 0.5, 0.0], [1.0, 1.0, 1.0], "Wall")]);
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(reading.hit);
        let d = reading.distance_m.unwrap();
        assert!((d - 1.70).abs() < 1e-9, "distance {d}");
        assert!(reading.hit_point.is_some());
    }

    #[test]
    fn range_facing_away_is_escapement() {
        let mut sensor = default_sensor(SensorKind::FrontRange);
        sensor.range_m = 10.0;
        sensor.fov_deg = 0.0;
        sensor.facing = [0.0, 0.0, 1.0];
        let set = obs_set(vec![obstacle("w1", "wall", [2.0, 0.5, 0.0], [1.0, 1.0, 1.0], "Wall")]);
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(!reading.hit);
        assert_eq!(reading.distance_m, None);
        assert_eq!(reading.hit_point, None);
    }

    #[test]
    fn range_past_range_m_misses() {
        let mut sensor = default_sensor(SensorKind::FrontRange);
        sensor.range_m = 0.5;
        sensor.fov_deg = 0.0;
        let set = obs_set(vec![obstacle("w1", "wall", [2.0, 0.5, 0.0], [1.0, 1.0, 1.0], "Wall")]);
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(!reading.hit);
        assert_eq!(reading.distance_m, None);
    }

    #[test]
    fn range_ignores_pad_obstacles() {
        let mut sensor = default_sensor(SensorKind::FrontRange);
        sensor.range_m = 10.0;
        sensor.fov_deg = 0.0;
        let pad = obstacle("landing_pad_1", "wall", [1.0, 0.5, 0.0], [1.0, 1.0, 1.0], "Landing Pad");
        assert!(pad.is_pad());
        let set = obs_set(vec![pad]);
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(!reading.hit);
        assert_eq!(reading.distance_m, None);
    }

    #[test]
    fn fov_cone_catches_hit_center_ray_misses() {
        let tower = obstacle("t1", "tower", [2.0, 0.75, 0.3], [1.0, 1.0, 1.0], "Tower");
        let set = obs_set(vec![tower]);
        let mut narrow = default_sensor(SensorKind::FrontRange);
        narrow.range_m = 10.0;
        narrow.fov_deg = 0.0;
        narrow.mount_position = [0.0, 0.0, 0.0];
        let r_narrow = evaluate(&narrow, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(!r_narrow.hit);
        let mut wide = narrow.clone();
        wide.fov_deg = 30.0;
        let r_wide = evaluate(&wide, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert!(r_wide.hit);
    }

    #[test]
    fn color_over_pad_reads_color() {
        let mut table = ColorTable::new();
        table.insert("pad_red", "red");
        let pad = obstacle("pad_red", "square", [0.0, 0.05, 0.0], [0.5, 0.1, 0.5], "Landing Pad");
        assert!(pad.is_pad());
        let set = obs_set(vec![pad]);
        let mut sensor = default_sensor(SensorKind::FrontColor);
        sensor.mount_position = [0.0, 0.0, 0.0];
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &table);
        assert_eq!(reading.value, "red");
        assert!(reading.hit);
    }

    #[test]
    fn color_over_nothing_is_unknown() {
        let set = obs_set(vec![]);
        let sensor = default_sensor(SensorKind::FrontColor);
        let reading = evaluate(&sensor, [0.0, 0.5, 0.0], 0.0, &set, &ColorTable::new());
        assert_eq!(reading.value, "Unknown");
        assert!(!reading.hit);
    }

    #[test]
    fn sensorset_json_roundtrip_identical() {
        let mut set = SensorSet::new();
        set.add_default(SensorKind::FrontRange);
        set.add_default(SensorKind::BottomRange);
        set.add_default(SensorKind::Temperature);
        let json = set.to_json().unwrap();
        let parsed = SensorSet::from_json(&json).unwrap();
        assert_eq!(set, parsed);
    }

    #[test]
    fn old_obstacle_json_without_sensors_deserializes() {
        let json = r#"{"obstacles":[{"id":"w1","type":"wall","position":[1,0.5,0],"rotation":[0,0,0],"scale":[1,1,1],"name":"Wall"}],"boundary":null}"#;
        let set = SensorSet::from_json(json).expect("old obstacle file parses");
        assert!(set.sensors.is_empty());
    }

    #[test]
    fn sensor_json_missing_optional_fields_deserializes() {
        let json = r#"{"id":"s1","kind":"FrontRange","mount_position":[0,0,0],"facing":[1,0,0],"range_m":3.0,"fov_deg":30.0}"#;
        let sensor: Sensor = serde_json::from_str(json).expect("partial sensor parses");
        assert_eq!(sensor.name, "");
        assert!(sensor.enabled);
    }

    #[test]
    fn temp_battery_ignore_mount_facing() {
        let mut t = default_sensor(SensorKind::Temperature);
        t.mount_position = [99.0, 99.0, 99.0];
        t.facing = [1.0, 2.0, 3.0];
        let rt = evaluate(&t, [0.0, 0.0, 0.0], 42.0, &obs_set(vec![]), &ColorTable::new());
        assert_eq!(rt.value, "22.0 C");
        assert!(!rt.hit);
        assert_eq!(rt.distance_m, None);
        assert_eq!(rt.hit_point, None);

        let mut b = default_sensor(SensorKind::Battery);
        b.mount_position = [99.0, 99.0, 99.0];
        b.facing = [1.0, 2.0, 3.0];
        let rb = evaluate(&b, [0.0, 0.0, 0.0], 42.0, &obs_set(vec![]), &ColorTable::new());
        assert_eq!(rb.value, "80.0 %");
        assert!(!rb.hit);
        assert_eq!(rb.distance_m, None);
        assert_eq!(rb.hit_point, None);
    }

    #[test]
    fn remove_by_id() {
        let mut set = SensorSet::new();
        let sensor = set.add_default(SensorKind::FrontRange);
        let removed = set.remove(&sensor.id);
        assert!(removed.is_some());
        assert!(set.sensors.is_empty());
        assert!(set.remove("missing").is_none());
    }
}
