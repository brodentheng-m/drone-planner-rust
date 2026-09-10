use planner_core::commands::{Command, CommandType, ParamValue, command_defs};
use planner_core::golden::{LedValue, SimResult};
use planner_core::obstacles::ObstacleSet;
use planner_core::planio::{Plan, PlanDrone};
use planner_core::sim::drive::SimState;
use std::collections::BTreeMap;
use std::time::{SystemTime, UNIX_EPOCH};

pub const TELEMETRY_BUFFER_SIZE: usize = 300;
pub const CONSOLE_CAP: usize = 500;
pub const DRONE_COLORS: [&str; 8] = [
    "#58a6ff", "#3fb950", "#f0883e", "#bc8cff", "#39d2c0", "#f778ba", "#d29922", "#f85149",
];

#[derive(Debug, Clone, Default)]
pub struct Selection {
    #[allow(dead_code)]
    pub drone_index: Option<usize>,
    pub command_path: Vec<usize>,
    #[allow(dead_code)]
    pub obstacle_id: Option<String>,
}

#[derive(Debug, Clone)]
pub struct Playback {
    pub playing: bool,
    pub frame: usize,
    pub speed: f64,
}

impl Default for Playback {
    fn default() -> Self {
        Playback {
            playing: false,
            frame: 0,
            speed: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    MapLock,
    Follow,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct CameraState {
    pub mode: CameraMode,
    pub orbit_yaw: f64,
    pub orbit_pitch: f64,
    pub orbit_dist: f64,
    pub pan_offset: [f64; 2],
    pub last_drone_pos: Option<[f64; 3]>,
}

impl Default for CameraState {
    fn default() -> Self {
        CameraState {
            mode: CameraMode::MapLock,
            orbit_yaw: 0.0,
            orbit_pitch: 0.4019,
            orbit_dist: 2.1731,
            pan_offset: [0.0, 0.0],
            last_drone_pos: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FramePoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,
    pub led: LedValue,
    pub speed: f64,
    #[allow(dead_code)]
    pub energy_used: f64,
    pub battery_percent: f64,
    #[allow(dead_code)]
    pub turn_radius_m: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct FrameState {
    pub positions: BTreeMap<String, FramePoint>,
    pub command_index: i64,
}

#[derive(Debug, Clone)]
pub struct AppState {
    pub plan: Plan,
    pub obstacles: ObstacleSet,
    pub selection: Selection,
    pub sim_result: Option<SimResult>,
    pub sim_results: BTreeMap<String, SimResult>,
    pub playback: Playback,
    pub console: Vec<String>,
    pub playback_time: f64,
    pub playback_speed: f64,
    pub playback_scrub: f64,
    pub playback_ended: bool,
    pub flying: bool,
    pub has_collision: bool,
    pub telemetry_ring: Vec<f64>,
    pub telemetry_head: usize,
    pub telemetry_count: usize,
    pub status_text: String,
    pub camera_mode: u8,
    pub trail_generation: u64,
    pub route_map: BTreeMap<String, Vec<(usize, usize)>>,
    pub active_drone_route: Vec<(usize, usize)>,
    drone_seq: u64,
    cmd_seq: u64,
    collision_marks: Vec<(String, String)>,
    collision_points: Vec<[f64; 3]>,
    pub camera_reset_pending: bool,
    pub boundary_visible: bool,
    pub generated_code: String,
    pub dirty: bool,
    pub camera: CameraState,
    pub show_left: bool,
    pub show_right: bool,
    tick_epoch: u64,
    last_tick_epoch: u64,
}

impl Default for AppState {
    fn default() -> Self {
        AppState::new()
    }
}

impl AppState {
    pub fn new() -> AppState {
        AppState {
            plan: Plan::default_plan(),
            obstacles: ObstacleSet::default(),
            selection: Selection::default(),
            sim_result: None,
            sim_results: BTreeMap::new(),
            playback: Playback::default(),
            console: Vec::new(),
            playback_time: 0.0,
            playback_speed: 1.0,
            playback_scrub: 0.0,
            playback_ended: false,
            flying: false,
            has_collision: false,
            telemetry_ring: vec![0.0; TELEMETRY_BUFFER_SIZE],
            telemetry_head: 0,
            telemetry_count: 0,
            status_text: "READY".to_string(),
            camera_mode: 1,
            trail_generation: 0,
            route_map: BTreeMap::new(),
            active_drone_route: Vec::new(),
            drone_seq: 1,
            cmd_seq: 0,
            collision_marks: Vec::new(),
            collision_points: Vec::new(),
            camera_reset_pending: false,
            boundary_visible: true,
            generated_code: String::new(),
            dirty: false,
            camera: CameraState::default(),
            show_left: true,
            show_right: true,
            tick_epoch: 0,
            last_tick_epoch: u64::MAX,
        }
    }

    pub fn log(&mut self, msg: impl Into<String>) {
        self.push_line(msg.into());
    }

    pub fn log_level(&mut self, level: &str, msg: impl Into<String>) {
        let _ = level;
        self.push_line(msg.into());
    }

    fn push_line(&mut self, msg: String) {
        self.console.push(format!("[{}] {msg}", timestamp_now()));
        if self.console.len() > CONSOLE_CAP {
            let overflow = self.console.len() - CONSOLE_CAP;
            self.console.drain(..overflow);
        }
    }

    pub fn active_drone_index(&self) -> Option<usize> {
        if self.plan.drones.is_empty() {
            return None;
        }
        match &self.plan.active_drone_id {
            Some(id) => self.plan.drones.iter().position(|d| &d.id == id).or(Some(0)),
            None => Some(0),
        }
    }

    pub fn collision_points(&self) -> &[[f64; 3]] {
        &self.collision_points
    }

    pub fn max_duration(&self) -> f64 {
        let mut max_duration = 0.0f64;
        for result in self.sim_results.values() {
            if result.total_duration > max_duration {
                max_duration = result.total_duration;
            }
        }
        if max_duration <= 0.0 { 1.0 } else { max_duration }
    }

    fn update_flight_status(&mut self) {
        self.status_text = if self.has_collision {
            "COLLISION"
        } else if self.flying {
            "FLYING"
        } else {
            "READY"
        }
        .to_string();
    }

    fn end_status(&self) -> &'static str {
        let active_id = self.active_drone_index().map(|i| self.plan.drones[i].id.clone());
        let result = active_id
            .as_ref()
            .and_then(|id| self.sim_results.get(id))
            .or_else(|| self.sim_results.values().next());
        match result.and_then(|r| r.positions.last()) {
            Some(p) if p.z < 0.15 => "Landed",
            _ => "Completed",
        }
    }

    pub fn play(&mut self) {
        if self.sim_results.is_empty() {
            self.log_level("warn", "No commands to play");
            return;
        }
        let total: usize = self.plan.drones.iter().map(|d| d.commands.len()).sum();
        self.log(format!(
            "Play - {} drone(s), {} commands",
            self.plan.drones.len(),
            total
        ));
        if self.playback_ended || self.playback_time <= 0.0 {
            self.playback_time = 0.0;
            self.trail_generation += 1;
            self.flying = true;
            self.has_collision = false;
            self.collision_marks.clear();
            self.collision_points.clear();
            self.telemetry_head = 0;
            self.telemetry_count = 0;
        }
        self.playback_ended = false;
        self.playback.playing = true;
        self.update_flight_status();
    }

    pub fn pause(&mut self) {
        self.playback.playing = false;
        self.flying = false;
        self.update_flight_status();
        self.log_level("warn", "Playback paused");
    }

    pub fn stop_reset(&mut self) {
        self.playback.playing = false;
        self.playback_time = 0.0;
        self.playback_ended = false;
        self.trail_generation += 1;
        self.playback_scrub = 0.0;
        self.flying = false;
        self.has_collision = false;
        self.collision_marks.clear();
        self.collision_points.clear();
        self.telemetry_head = 0;
        self.telemetry_count = 0;
        self.update_flight_status();
        self.log("Playback reset");
    }

    pub fn set_speed(&mut self, s: f64) {
        self.playback_speed = s.clamp(0.1, 20.0);
        self.playback.speed = self.playback_speed;
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub fn mark_saved(&mut self) {
        self.dirty = false;
    }

    pub fn begin_frame(&mut self) {
        self.tick_epoch += 1;
    }

    pub fn sync_camera_mode(&mut self) {
        self.camera.mode = if self.camera_mode == 2 {
            CameraMode::Follow
        } else {
            CameraMode::MapLock
        };
    }

    pub fn set_scrub(&mut self, f: f64) {
        if self.sim_results.is_empty() {
            return;
        }
        self.playback.playing = false;
        self.flying = false;
        let t = f.clamp(0.0, 1.0);
        self.playback_scrub = t;
        self.playback_time = t * self.max_duration();
        self.apply_collisions();
        if t >= 1.0 && !self.playback_ended {
            self.playback_ended = true;
            self.status_text = self.end_status().to_string();
        } else if !self.playback_ended {
            self.update_flight_status();
        }
    }

    pub fn refresh_sim(&mut self) {
        self.log(format!("Simulating {} drone(s)...", self.plan.drones.len()));
        let mut route_map: BTreeMap<String, Vec<(usize, usize)>> = BTreeMap::new();
        for drone in &self.plan.drones {
            route_map.insert(drone.id.clone(), Vec::new());
        }
        let mut on_start = |command: &Command, index: usize, state: &SimState| {
            let _ = command;
            if let Some(entries) = route_map.get_mut(&state.drone_id) {
                entries.push((index, state.points.len()));
            }
        };
        self.sim_results = planner_core::sim::simulate_swarm_with(
            &self.plan,
            Some(&self.obstacles),
            Some(&mut on_start),
        );
        self.route_map = route_map;
        self.active_drone_route = self
            .active_drone_index()
            .and_then(|i| self.route_map.get(&self.plan.drones[i].id).cloned())
            .unwrap_or_default();
        self.sim_result = self
            .active_drone_index()
            .and_then(|i| self.sim_results.get(&self.plan.drones[i].id).cloned());
        self.playback_ended = false;
        let point_total: usize = self.sim_results.values().map(|r| r.positions.len()).sum();
        self.log(format!(
            "Simulation complete: {} points, {:.1} s",
            point_total,
            self.max_duration()
        ));
        self.generated_code = if self.plan.drones.len() > 1 {
            planner_core::codegen::generate_swarm_code(&self.plan)
        } else {
            planner_core::codegen::generate_code(&self.plan)
        };
        self.log(format!(
            "Generated {} chars Python",
            self.generated_code.chars().count()
        ));
        self.selection.drone_index = self.active_drone_index();
        self.collision_marks.clear();
        self.collision_points.clear();
        let mut hits = Vec::new();
        for (id, result) in &self.sim_results {
            for collision in &result.collisions {
                let key = (id.clone(), collision.obstacle.id.clone());
                if !self.collision_marks.contains(&key) {
                    self.collision_marks.push(key);
                    self.collision_points
                        .push([collision.position.x, collision.position.y, collision.position.z]);
                }
                hits.push(format!(
                    "COLLISION: {id} hit {} ({})",
                    collision.obstacle.obstacle_type, collision.obstacle.name
                ));
            }
        }
        self.has_collision = !hits.is_empty();
        for line in hits {
            self.log_level("error", line);
        }
        self.update_flight_status();
    }

    fn apply_collisions(&mut self) {
        let Some(frame) = self.current_frame() else {
            return;
        };
        let mut new_hits = Vec::new();
        for (id, point) in &frame.positions {
            if let Some(hit) = self.obstacles.check_collision(point.x, point.y, point.z, 0.1) {
                let key = (id.clone(), hit.obstacle.id.clone());
                if !self.collision_marks.contains(&key) {
                    new_hits.push((key, hit.obstacle.obstacle_type.clone(), hit.obstacle.name.clone(), [point.x, point.y, point.z]));
                }
            }
        }
        let mut hit_landed = false;
        for (key, obstacle_type, name, pos) in new_hits {
            self.has_collision = true;
            hit_landed = true;
            self.log_level("error", format!("COLLISION: {} hit {} ({})", key.0, obstacle_type, name));
            self.collision_marks.push(key);
            self.collision_points.push(pos);
        }
        if hit_landed && !self.playback_ended {
            self.update_flight_status();
        }
    }

    pub fn tick(&mut self, dt_seconds: f64) -> bool {
        if !self.playback.playing {
            return false;
        }
        if self.last_tick_epoch != self.tick_epoch {
            self.last_tick_epoch = self.tick_epoch;
            self.playback_time += dt_seconds * self.playback.speed;
        }
        let max_duration = self.max_duration();
        let t = (self.playback_time / max_duration).min(1.0);
        self.playback_scrub = t;
        self.apply_collisions();
        if t >= 1.0 && !self.playback_ended {
            self.playback_ended = true;
            self.playback.playing = false;
            self.flying = false;
            self.status_text = self.end_status().to_string();
        }
        if let Some(frame) = self.current_frame() {
            let active_id = self.active_drone_index().map(|i| self.plan.drones[i].id.clone());
            let point = active_id
                .as_ref()
                .and_then(|id| frame.positions.get(id))
                .or_else(|| frame.positions.values().next());
            if let Some(p) = point {
                self.camera.last_drone_pos = Some([p.x, p.y, p.z]);
            }
        }
        let speed = self.active_frame_speed().unwrap_or(0.0);
        self.push_telemetry_sample(speed);
        let active_id = self.active_drone_index().map(|i| self.plan.drones[i].id.clone());
        self.playback.frame = active_id
            .as_deref()
            .and_then(|id| self.frame_index_for(id))
            .unwrap_or(0);
        true
    }

    fn active_frame_speed(&self) -> Option<f64> {
        let frame = self.current_frame()?;
        let active_id = self.active_drone_index().map(|i| self.plan.drones[i].id.clone());
        active_id
            .as_ref()
            .filter(|id| frame.positions.contains_key(*id))
            .or_else(|| frame.positions.keys().next())
            .and_then(|id| frame.positions.get(id))
            .map(|p| p.speed)
    }

    pub fn push_telemetry_sample(&mut self, speed: f64) {
        if self.telemetry_ring.len() < TELEMETRY_BUFFER_SIZE {
            self.telemetry_ring.resize(TELEMETRY_BUFFER_SIZE, 0.0);
        }
        self.telemetry_ring[self.telemetry_head] = speed;
        self.telemetry_head = (self.telemetry_head + 1) % TELEMETRY_BUFFER_SIZE;
        if self.telemetry_count < TELEMETRY_BUFFER_SIZE {
            self.telemetry_count += 1;
        }
    }

    pub fn telemetry_series(&self) -> (Vec<f64>, f64, f64) {
        if self.telemetry_count == 0 {
            return (Vec::new(), 0.0, 0.0);
        }
        let start =
            (self.telemetry_head + TELEMETRY_BUFFER_SIZE - self.telemetry_count) % TELEMETRY_BUFFER_SIZE;
        let mut series = Vec::with_capacity(self.telemetry_count);
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        for i in 0..self.telemetry_count {
            let v = self.telemetry_ring[(start + i) % TELEMETRY_BUFFER_SIZE];
            min_v = min_v.min(v);
            max_v = max_v.max(v);
            series.push(v);
        }
        if max_v - min_v < 0.1 {
            max_v += 0.1;
            min_v -= 0.1;
        }
        if min_v < 0.0 {
            min_v = 0.0;
        }
        (series, min_v, max_v)
    }

    pub fn current_frame(&self) -> Option<FrameState> {
        if self.sim_results.is_empty() {
            return None;
        }
        let t = self.playback_scrub;
        let mut positions = BTreeMap::new();
        for (id, result) in &self.sim_results {
            let len = result.positions.len();
            if len == 0 {
                continue;
            }
            let (a, b, frac) = if len == 1 {
                (&result.positions[0], &result.positions[0], 0.0)
            } else {
                let raw = t * (len - 1) as f64;
                let idx = (raw.floor() as usize).min(len - 2);
                let frac = raw - idx as f64;
                (&result.positions[idx], &result.positions[(idx + 1).min(len - 1)], frac)
            };
            positions.insert(
                id.clone(),
                FramePoint {
                    x: a.x + (b.x - a.x) * frac,
                    y: a.y + (b.y - a.y) * frac,
                    z: a.z + (b.z - a.z) * frac,
                    heading: a.heading + (b.heading - a.heading) * frac,
                    pitch: a.pitch + (b.pitch - a.pitch) * frac,
                    roll: a.roll + (b.roll - a.roll) * frac,
                    led: if frac < 0.5 { a.led.clone() } else { b.led.clone() },
                    speed: a.speed,
                    energy_used: a.energy_used,
                    battery_percent: a.battery_percent,
                    turn_radius_m: a.turn_radius_m,
                },
            );
        }
        let command_index = self
            .active_drone_index()
            .and_then(|i| {
                let id = &self.plan.drones[i].id;
                self.frame_index_for(id).and_then(|idx| {
                    self.route_map
                        .get(id)
                        .and_then(|entries| entries.iter().rev().find(|e| e.1 <= idx))
                        .map(|e| e.0 as i64)
                })
            })
            .unwrap_or(-1);
        Some(FrameState {
            positions,
            command_index,
        })
    }

    fn frame_index_for(&self, id: &str) -> Option<usize> {
        let result = self.sim_results.get(id)?;
        let len = result.positions.len();
        if len == 0 {
            return None;
        }
        if len == 1 {
            return Some(0);
        }
        let raw = self.playback_scrub * (len - 1) as f64;
        Some((raw.floor() as usize).min(len - 2))
    }

    pub fn add_drone(&mut self) {
        let idx = self.plan.drones.len();
        let color = DRONE_COLORS[idx % DRONE_COLORS.len()];
        self.drone_seq += 1;
        let drone = PlanDrone {
            id: format!("d{}", self.drone_seq),
            name: format!("Drone {}", idx + 1),
            color: color.to_string(),
            commands: Vec::new(),
            offset: [0.0, 0.0, 0.0],
        };
        self.plan.active_drone_id = Some(drone.id.clone());
        self.log(format!("Added {}", drone.name));
        self.plan.drones.push(drone);
        self.selection = Selection::default();
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn remove_active_drone(&mut self) {
        if self.plan.drones.len() <= 1 {
            self.log_level("warn", "Cannot remove the only drone");
            return;
        }
        let Some(index) = self.active_drone_index() else {
            return;
        };
        let removed = self.plan.drones.remove(index);
        self.log(format!("Removed {}", removed.name));
        if self.plan.active_drone_id.as_deref() == Some(removed.id.as_str()) {
            self.plan.active_drone_id = self.plan.drones.first().map(|d| d.id.clone());
        }
        self.selection = Selection::default();
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn duplicate_active_drone(&mut self) {
        let Some(index) = self.active_drone_index() else {
            return;
        };
        let src = self.plan.drones[index].clone();
        let color = DRONE_COLORS[self.plan.drones.len() % DRONE_COLORS.len()];
        self.drone_seq += 1;
        let drone = PlanDrone {
            id: format!("d{}", self.drone_seq),
            name: format!("{} Copy", src.name),
            color: color.to_string(),
            commands: src.commands,
            offset: src.offset,
        };
        self.plan.active_drone_id = Some(drone.id.clone());
        self.log(format!("Duplicated {} as {}", src.name, drone.name));
        self.plan.drones.push(drone);
        self.selection = Selection::default();
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn set_formation(&mut self, kind: &str) {
        let count = self.plan.drones.len();
        let spacing = 0.5f64;
        for i in 0..count {
            let offset = match kind {
                "line" => [(i as f64 - (count as f64 - 1.0) / 2.0) * spacing, 0.0, 0.0],
                "grid" => {
                    let cols = (count as f64).sqrt().ceil() as usize;
                    let row = i / cols;
                    let col = i % cols;
                    [
                        (col as f64 - (cols as f64 - 1.0) / 2.0) * spacing,
                        0.0,
                        (row as f64 - ((count - 1) / cols) as f64 / 2.0) * spacing,
                    ]
                }
                "circle" => {
                    let angle = (i as f64 / count as f64) * std::f64::consts::PI * 2.0;
                    let radius = count as f64 * spacing / (2.0 * std::f64::consts::PI);
                    [angle.cos() * radius, 0.0, angle.sin() * radius]
                }
                "v" => {
                    let half = (i as f64 / 2.0).ceil();
                    let side = if i % 2 == 0 { 1.0 } else { -1.0 };
                    [side * half * spacing * 0.7, 0.0, -half * spacing * 0.5]
                }
                "column" => [0.0, 0.0, (i as f64 - (count as f64 - 1.0) / 2.0) * spacing],
                "arc" => {
                    let denom = if count > 1 { count - 1 } else { 1 };
                    let a = (i as f64 / denom as f64 - 0.5) * std::f64::consts::PI;
                    [a.sin() * 1.5, 0.0, -a.cos() * 1.5 + 1.5]
                }
                _ => [0.0, 0.0, 0.0],
            };
            self.plan.drones[i].offset = offset;
        }
        self.log(format!("Formation: {kind} ({count} drones)"));
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn add_command(&mut self, command_type: CommandType) {
        let Some(def) = command_defs().iter().find(|d| d.command_type == command_type) else {
            return;
        };
        let label = def.label;
        self.cmd_seq += 1;
        let drone_pos = self.active_drone_index().map(|i| i + 1).unwrap_or(1);
        let mut command = Command::new(&format!("d{}c{}", drone_pos, self.cmd_seq), command_type);
        for param in def.params {
            command
                .params
                .insert(param.key.to_string(), param.default.clone());
        }
        let Some(index) = self.active_drone_index() else {
            return;
        };
        let drone = &mut self.plan.drones[index];
        if !self.selection.command_path.is_empty() {
            let path = self.selection.command_path.clone();
            if let Some(target) = command_at_path_mut(&mut drone.commands, &path) {
                if target.command_type.is_block() {
                    target.children.push(command);
                    self.log(format!("Added: {label}"));
                    self.mark_dirty();
                    self.refresh_sim();
                    return;
                }
            }
            self.selection.command_path.clear();
        }
        drone.commands.push(command);
        self.log(format!("Added: {label}"));
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn move_command(&mut self, up: bool) {
        let path = self.selection.command_path.clone();
        if path.is_empty() {
            return;
        }
        let Some(index) = self.active_drone_index() else {
            return;
        };
        let last = path[path.len() - 1];
        let parent_path = &path[..path.len() - 1];
        let drone = &mut self.plan.drones[index];
        let list: &mut Vec<Command> = if parent_path.is_empty() {
            &mut drone.commands
        } else {
            match command_at_path_mut(&mut drone.commands, parent_path) {
                Some(parent) => &mut parent.children,
                None => return,
            }
        };
        if up {
            if last == 0 || last >= list.len() {
                return;
            }
            list.swap(last - 1, last);
            self.selection.command_path = [parent_path, &[last - 1]].concat();
        } else {
            if last + 1 >= list.len() {
                return;
            }
            list.swap(last, last + 1);
            self.selection.command_path = [parent_path, &[last + 1]].concat();
        }
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn delete_selected(&mut self) {
        let path = self.selection.command_path.clone();
        if path.is_empty() {
            return;
        }
        let Some(index) = self.active_drone_index() else {
            return;
        };
        let last = path[path.len() - 1];
        let parent_path = &path[..path.len() - 1];
        let drone = &mut self.plan.drones[index];
        let list: &mut Vec<Command> = if parent_path.is_empty() {
            &mut drone.commands
        } else {
            match command_at_path_mut(&mut drone.commands, parent_path) {
                Some(parent) => &mut parent.children,
                None => return,
            }
        };
        if last < list.len() {
            list.remove(last);
        }
        self.selection.command_path.clear();
        self.mark_dirty();
        self.refresh_sim();
    }

    pub fn set_param(&mut self, key: &str, value: ParamValue) {
        let path = self.selection.command_path.clone();
        if path.is_empty() {
            return;
        }
        let Some(index) = self.active_drone_index() else {
            return;
        };
        if let Some(command) = command_at_path_mut(&mut self.plan.drones[index].commands, &path) {
            command.params.insert(key.to_string(), value);
            self.mark_dirty();
        }
    }

    pub fn import_code(&mut self, source: &str) -> Result<usize, String> {
        let commands = planner_core::codegen::ScriptParser::new().parse(source);
        let Some(index) = self.active_drone_index() else {
            let err = "No active drone".to_string();
            self.log_level("error", format!("Error applying code: {err}"));
            return Err(err);
        };
        let count = commands.len();
        self.plan.drones[index].commands = commands;
        self.mark_dirty();
        self.refresh_sim();
        self.log_level("success", "Code applied successfully!");
        Ok(count)
    }

    pub fn import_plan_json(&mut self, text: &str) -> Result<(), String> {
        let path = temp_path("plan_in");
        if let Err(error) = std::fs::write(&path, text) {
            return Err(error.to_string());
        }
        let loaded = planner_core::planio::load_plan(path.to_string_lossy().as_ref());
        let _ = std::fs::remove_file(&path);
        match loaded {
            Ok(plan) => {
                self.plan = plan;
                if self.plan.active_drone_id.is_none() && !self.plan.drones.is_empty() {
                    self.plan.active_drone_id = Some(self.plan.drones[0].id.clone());
                }
                self.selection = Selection::default();
                self.log(format!("Loaded flight plan: {}", self.plan.name));
                self.dirty = false;
                self.refresh_sim();
                Ok(())
            }
            Err(error) => {
                let err = error.to_string();
                self.log_level("error", format!("Error loading plan: {err}"));
                Err(err)
            }
        }
    }

    pub fn export_plan_json(&self) -> String {
        let path = temp_path("plan_out");
        if planner_core::planio::save_plan(path.to_string_lossy().as_ref(), &self.plan).is_err() {
            return String::new();
        }
        let text = std::fs::read_to_string(&path).unwrap_or_default();
        let _ = std::fs::remove_file(&path);
        text
    }

    pub fn import_obstacle_text(&mut self, text: &str, filename: &str) -> Result<usize, String> {
        let lower = filename.to_ascii_lowercase();
        let parsed = if lower.ends_with(".json") {
            planner_core::obstacles::import_json(text)
        } else if lower.ends_with(".geojson") {
            planner_core::obstacles::import_geojson(text)
        } else if lower.ends_with(".csv") {
            planner_core::obstacles::import_csv(text)
        } else if lower.ends_with(".obj") {
            planner_core::obstacles::import_obj(text)
        } else {
            Err(format!("Unsupported obstacle file: {filename}"))
        };
        match parsed {
            Ok(accepted) => {
                let rejected = self.obstacles.import(accepted);
                let kept = self.obstacles.obstacles.len();
                self.log(format!("Imported {kept} obstacles from {filename}"));
                if rejected > 0 {
                    self.log_level(
                        "warn",
                        format!("Rejected {rejected} obstacles outside boundary"),
                    );
                }
                self.selection.obstacle_id = None;
                self.mark_dirty();
                self.refresh_sim();
                Ok(kept)
            }
            Err(err) => {
                self.log_level("error", format!("Error importing obstacles: {err}"));
                Err(err)
            }
        }
    }
}

fn temp_path(tag: &str) -> std::path::PathBuf {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    std::env::temp_dir().join(format!("drone_planner_{}_{}_{tag}.json", std::process::id(), n))
}

fn command_at_path_mut<'a>(
    commands: &'a mut [Command],
    path: &[usize],
) -> Option<&'a mut Command> {
    if path.is_empty() {
        return None;
    }
    let mut current = commands;
    for &idx in &path[..path.len() - 1] {
        current = current.get_mut(idx)?.children.as_mut_slice();
    }
    current.get_mut(path[path.len() - 1])
}

fn timestamp_now() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!(
        "{:02}:{:02}:{:02}",
        (secs / 3600) % 24,
        (secs / 60) % 60,
        secs % 60
    )
}
