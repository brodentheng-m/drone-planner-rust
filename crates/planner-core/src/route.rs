use crate::commands::{Command, CommandType, ParamValue};
use crate::obstacles::{COLLISION_DRONE_SIZE, ObstacleSet};
use crate::sim;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::collections::BinaryHeap;
use std::cmp::Ordering;

const DRONE_RADIUS: f64 = COLLISION_DRONE_SIZE / 2.0;
const MAX_TURNS_PER_LEG: usize = 40;
const WAYPOINT_REACH_TOLERANCE: f64 = 0.35;
const BALANCED_WEIGHT: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Optimize {
    Shortest,
    WidestClearance,
    Balanced,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteOptions {
    pub clearance_m: f64,
    pub optimize: Optimize,
    pub cruise_speed: f64,
    pub takeoff_height_m: f64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions {
            clearance_m: 0.05,
            optimize: Optimize::WidestClearance,
            cruise_speed: 50.0,
            takeoff_height_m: 0.8,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteResult {
    pub commands: Vec<Command>,
    pub path: Vec<[f64; 3]>,
    pub waypoints_reached: usize,
    pub feasible: bool,
    pub failure: Option<RouteFailure>,
    pub legs: Vec<LegInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RouteFailure {
    StartInsideObstacle,
    WaypointInsideObstacle(usize),
    NoPath(usize),
    ClearanceUnmet(usize),
    TurnBudgetExceeded(usize),
    EmptyWaypoints,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegInfo {
    pub leg_index: usize,
    pub distance_m: f64,
    pub turn_count: usize,
}

struct InflatedRect {
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
}

fn inflate_obstacles_2d(obstacles: &ObstacleSet, margin: f64, flight_y: f64) -> Vec<InflatedRect> {
    let mut result = Vec::new();
    for obstacle in &obstacles.obstacles {
        if obstacle.is_pad() {
            continue;
        }
        if crate::obstacles::type_dims(&obstacle.obstacle_type).is_none() {
            continue;
        }
        let wb = obstacles.world_box(obstacle);
        if wb.max[1] < flight_y - margin || wb.min[1] > flight_y + margin {
            continue;
        }
        result.push(InflatedRect {
            min_x: wb.min[0] - margin,
            max_x: wb.max[0] + margin,
            min_z: wb.min[2] - margin,
            max_z: wb.max[2] + margin,
        });
    }
    result
}

fn point_inside_inflated_3d(p: [f64; 3], obstacles: &ObstacleSet, margin: f64) -> bool {
    for obstacle in &obstacles.obstacles {
        if obstacle.is_pad() {
            continue;
        }
        if crate::obstacles::type_dims(&obstacle.obstacle_type).is_none() {
            continue;
        }
        let wb = obstacles.world_box(obstacle);
        if p[0] >= wb.min[0] - margin
            && p[0] <= wb.max[0] + margin
            && p[1] >= wb.min[1] - margin
            && p[1] <= wb.max[1] + margin
            && p[2] >= wb.min[2] - margin
            && p[2] <= wb.max[2] + margin
        {
            return true;
        }
    }
    false
}

fn point_inside_rects(x: f64, z: f64, rects: &[InflatedRect]) -> bool {
    for r in rects {
        if x >= r.min_x && x <= r.max_x && z >= r.min_z && z <= r.max_z {
            return true;
        }
    }
    false
}

fn segment_intersects_rect(ax: f64, az: f64, bx: f64, bz: f64, r: &InflatedRect) -> bool {
    let mut tmin = 0.0_f64;
    let mut tmax = 1.0_f64;

    let dx = bx - ax;
    if dx.abs() < 1e-12 {
        if ax < r.min_x || ax > r.max_x {
            return false;
        }
    } else {
        let inv = 1.0 / dx;
        let mut t1 = (r.min_x - ax) * inv;
        let mut t2 = (r.max_x - ax) * inv;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return false;
        }
    }

    let dz = bz - az;
    if dz.abs() < 1e-12 {
        if az < r.min_z || az > r.max_z {
            return false;
        }
    } else {
        let inv = 1.0 / dz;
        let mut t1 = (r.min_z - az) * inv;
        let mut t2 = (r.max_z - az) * inv;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return false;
        }
    }

    true
}

fn segment_blocked_2d(ax: f64, az: f64, bx: f64, bz: f64, rects: &[InflatedRect]) -> bool {
    for r in rects {
        if segment_intersects_rect(ax, az, bx, bz, r) {
            return true;
        }
    }
    false
}

fn min_clearance_segment_2d(
    ax: f64,
    az: f64,
    bx: f64,
    bz: f64,
    obstacles: &ObstacleSet,
    flight_y: f64,
) -> f64 {
    let mut min_dist = f64::INFINITY;
    for obstacle in &obstacles.obstacles {
        if obstacle.is_pad() {
            continue;
        }
        if crate::obstacles::type_dims(&obstacle.obstacle_type).is_none() {
            continue;
        }
        let wb = obstacles.world_box(obstacle);
        if wb.max[1] < flight_y || wb.min[1] > flight_y {
            continue;
        }
        
        let mut d = f64::INFINITY;
        d = d.min(point_to_rect_dist(ax, az, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
        d = d.min(point_to_rect_dist(bx, bz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
        
        let dx = bx - ax;
        let dz = bz - az;
        let len2 = dx * dx + dz * dz;
        if len2 > 1e-12 {
            for &rx in &[wb.min[0], wb.max[0]] {
                let t = (rx - ax) / dx;
                if t >= 0.0 && t <= 1.0 {
                    let pz = az + t * dz;
                    d = d.min(point_to_rect_dist(rx, pz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                }
            }
            for &rz in &[wb.min[2], wb.max[2]] {
                let t = (rz - az) / dz;
                if t >= 0.0 && t <= 1.0 {
                    let px = ax + t * dx;
                    d = d.min(point_to_rect_dist(px, rz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                }
            }
            for &(cx, cz) in &[(wb.min[0], wb.min[2]), (wb.min[0], wb.max[2]), (wb.max[0], wb.min[2]), (wb.max[0], wb.max[2])] {
                let t = ((cx - ax) * dx + (cz - az) * dz) / len2;
                if t >= 0.0 && t <= 1.0 {
                    let px = ax + t * dx;
                    let pz = az + t * dz;
                    d = d.min(point_to_rect_dist(px, pz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                }
            }
        }
        if d < min_dist {
            min_dist = d;
        }
    }
    min_dist
}

fn point_to_rect_dist(px: f64, pz: f64, min_x: f64, max_x: f64, min_z: f64, max_z: f64) -> f64 {
    let dx = if px < min_x {
        min_x - px
    } else if px > max_x {
        px - max_x
    } else {
        0.0
    };
    let dz = if pz < min_z {
        min_z - pz
    } else if pz > max_z {
        pz - max_z
    } else {
        0.0
    };
    (dx * dx + dz * dz).sqrt()
}

fn dist2(ax: f64, az: f64, bx: f64, bz: f64) -> f64 {
    let dx = ax - bx;
    let dz = az - bz;
    (dx * dx + dz * dz).sqrt()
}

#[derive(Clone)]
struct AstarNode {
    cost: f64,
    heuristic: f64,
    index: usize,
}

impl PartialEq for AstarNode {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl Eq for AstarNode {}

impl Ord for AstarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        let a = self.cost + self.heuristic;
        let b = other.cost + other.heuristic;
        b.partial_cmp(&a).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for AstarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn visibility_graph_corners_2d(rects: &[InflatedRect]) -> Vec<(f64, f64)> {
    let eps = 0.001;
    let mut corners = Vec::new();
    for r in rects {
        corners.push((r.min_x - eps, r.min_z - eps));
        corners.push((r.min_x - eps, r.max_z + eps));
        corners.push((r.max_x + eps, r.min_z - eps));
        corners.push((r.max_x + eps, r.max_z + eps));
    }
    corners.retain(|c| !point_inside_rects(c.0, c.1, rects));
    corners
}

fn find_path_shortest_2d(
    sx: f64,
    sz: f64,
    gx: f64,
    gz: f64,
    rects: &[InflatedRect],
) -> Option<Vec<(f64, f64)>> {
    if !segment_blocked_2d(sx, sz, gx, gz, rects) {
        return Some(vec![(sx, sz), (gx, gz)]);
    }
    let mut nodes: Vec<(f64, f64)> = vec![(sx, sz)];
    let graph_corners = visibility_graph_corners_2d(rects);
    nodes.extend_from_slice(&graph_corners);
    let goal_idx = nodes.len();
    nodes.push((gx, gz));

    let n = nodes.len();
    let mut dist_to = vec![f64::INFINITY; n];
    let mut prev = vec![usize::MAX; n];
    let mut visited = vec![false; n];

    dist_to[0] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(AstarNode {
        cost: 0.0,
        heuristic: dist2(sx, sz, gx, gz),
        index: 0,
    });

    while let Some(current) = heap.pop() {
        if current.index == goal_idx {
            break;
        }
        if visited[current.index] {
            continue;
        }
        visited[current.index] = true;

        for j in 0..n {
            if visited[j] {
                continue;
            }
            if segment_blocked_2d(
                nodes[current.index].0,
                nodes[current.index].1,
                nodes[j].0,
                nodes[j].1,
                rects,
            ) {
                continue;
            }
            let edge = dist2(
                nodes[current.index].0,
                nodes[current.index].1,
                nodes[j].0,
                nodes[j].1,
            );
            let new_cost = dist_to[current.index] + edge;
            if new_cost < dist_to[j] {
                dist_to[j] = new_cost;
                prev[j] = current.index;
                heap.push(AstarNode {
                    cost: new_cost,
                    heuristic: dist2(nodes[j].0, nodes[j].1, gx, gz),
                    index: j,
                });
            }
        }
    }

    if dist_to[goal_idx].is_infinite() {
        return None;
    }

    let mut path = Vec::new();
    let mut idx = goal_idx;
    while idx != usize::MAX {
        path.push(nodes[idx]);
        idx = prev[idx];
    }
    path.reverse();
    Some(path)
}

fn find_path_widest_2d(
    sx: f64,
    sz: f64,
    gx: f64,
    gz: f64,
    rects: &[InflatedRect],
    obstacles: &ObstacleSet,
    flight_y: f64,
) -> Option<Vec<(f64, f64)>> {
    if !segment_blocked_2d(sx, sz, gx, gz, rects) {
        return Some(vec![(sx, sz), (gx, gz)]);
    }
    let mut nodes: Vec<(f64, f64)> = vec![(sx, sz)];
    let graph_corners = visibility_graph_corners_2d(rects);
    nodes.extend_from_slice(&graph_corners);
    let goal_idx = nodes.len();
    nodes.push((gx, gz));

    let n = nodes.len();
    let mut edge_data: Vec<Vec<(usize, f64, f64)>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if segment_blocked_2d(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1, rects) {
                continue;
            }
            let edge_len = dist2(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1);
            let clearance =
                min_clearance_segment_2d(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1, obstacles, flight_y);
            edge_data[i].push((j, edge_len, clearance));
            edge_data[j].push((i, edge_len, clearance));
        }
    }

    let mut best_min_clearance = vec![f64::NEG_INFINITY; n];
    best_min_clearance[0] = f64::INFINITY;
    let mut prev = vec![usize::MAX; n];

    #[derive(Clone, PartialEq)]
    struct CNode {
        min_clearance: f64,
        index: usize,
    }
    impl Eq for CNode {}
    impl Ord for CNode {
        fn cmp(&self, other: &Self) -> Ordering {
            self.min_clearance
                .partial_cmp(&other.min_clearance)
                .unwrap_or(Ordering::Equal)
        }
    }
    impl PartialOrd for CNode {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut heap = BinaryHeap::new();
    heap.push(CNode {
        min_clearance: f64::INFINITY,
        index: 0,
    });
    let mut visited = vec![false; n];

    while let Some(current) = heap.pop() {
        if current.index == goal_idx {
            break;
        }
        if visited[current.index] {
            continue;
        }
        visited[current.index] = true;

        for &(j, _edge_len, clearance) in &edge_data[current.index] {
            if visited[j] {
                continue;
            }
            let path_clearance = best_min_clearance[current.index].min(clearance);
            if path_clearance > best_min_clearance[j] {
                best_min_clearance[j] = path_clearance;
                prev[j] = current.index;
                heap.push(CNode {
                    min_clearance: path_clearance,
                    index: j,
                });
            }
        }
    }

    if best_min_clearance[goal_idx] == f64::NEG_INFINITY {
        return None;
    }

    let mut path = Vec::new();
    let mut idx = goal_idx;
    while idx != usize::MAX {
        path.push(nodes[idx]);
        idx = prev[idx];
    }
    path.reverse();
    Some(path)
}

fn find_path_balanced_2d(
    sx: f64,
    sz: f64,
    gx: f64,
    gz: f64,
    rects: &[InflatedRect],
    obstacles: &ObstacleSet,
    flight_y: f64,
) -> Option<Vec<(f64, f64)>> {
    if !segment_blocked_2d(sx, sz, gx, gz, rects) {
        return Some(vec![(sx, sz), (gx, gz)]);
    }
    let mut nodes: Vec<(f64, f64)> = vec![(sx, sz)];
    let graph_corners = visibility_graph_corners_2d(rects);
    nodes.extend_from_slice(&graph_corners);
    let goal_idx = nodes.len();
    nodes.push((gx, gz));

    let n = nodes.len();
    let mut max_edge_len = 0.0_f64;
    let mut max_clearance = 0.0_f64;
    let mut edges: Vec<Vec<(usize, f64, f64)>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if segment_blocked_2d(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1, rects) {
                continue;
            }
            let edge_len = dist2(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1);
            let clearance =
                min_clearance_segment_2d(nodes[i].0, nodes[i].1, nodes[j].0, nodes[j].1, obstacles, flight_y);
            if edge_len > max_edge_len {
                max_edge_len = edge_len;
            }
            if clearance > max_clearance {
                max_clearance = clearance;
            }
            edges[i].push((j, edge_len, clearance));
            edges[j].push((i, edge_len, clearance));
        }
    }

    if max_edge_len < 1e-12 {
        max_edge_len = 1.0;
    }
    if max_clearance < 1e-12 {
        max_clearance = 1.0;
    }

    let mut cost_to = vec![f64::INFINITY; n];
    cost_to[0] = 0.0;
    let mut prev = vec![usize::MAX; n];
    let mut visited = vec![false; n];

    let mut heap = BinaryHeap::new();
    heap.push(AstarNode {
        cost: 0.0,
        heuristic: dist2(sx, sz, gx, gz),
        index: 0,
    });

    while let Some(current) = heap.pop() {
        if current.index == goal_idx {
            break;
        }
        if visited[current.index] {
            continue;
        }
        visited[current.index] = true;

        for &(j, edge_len, clearance) in &edges[current.index] {
            if visited[j] {
                continue;
            }
            let norm_dist = edge_len / max_edge_len;
            let norm_clearance_penalty = 1.0 - (clearance / max_clearance).min(1.0);
            let edge_cost =
                BALANCED_WEIGHT * norm_dist + (1.0 - BALANCED_WEIGHT) * norm_clearance_penalty;
            let new_cost = cost_to[current.index] + edge_cost;
            if new_cost < cost_to[j] {
                cost_to[j] = new_cost;
                prev[j] = current.index;
                heap.push(AstarNode {
                    cost: new_cost,
                    heuristic: dist2(nodes[j].0, nodes[j].1, gx, gz) / max_edge_len
                        * BALANCED_WEIGHT,
                    index: j,
                });
            }
        }
    }

    if cost_to[goal_idx].is_infinite() {
        return None;
    }

    let mut path = Vec::new();
    let mut idx = goal_idx;
    while idx != usize::MAX {
        path.push(nodes[idx]);
        idx = prev[idx];
    }
    path.reverse();
    Some(path)
}

fn simplify_path_2d(path: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut result = vec![path[0]];
    for i in 1..path.len() - 1 {
        let prev = result.last().unwrap();
        let next = &path[i + 1];
        let cur = &path[i];
        let d1x = cur.0 - prev.0;
        let d1z = cur.1 - prev.1;
        let d2x = next.0 - cur.0;
        let d2z = next.1 - cur.1;
        let len1 = (d1x * d1x + d1z * d1z).sqrt();
        let len2 = (d2x * d2x + d2z * d2z).sqrt();
        if len1 < 1e-12 || len2 < 1e-12 {
            continue;
        }
        let dot = (d1x * d2x + d1z * d2z) / (len1 * len2);
        if dot < 0.9999 {
            result.push(*cur);
        }
    }
    result.push(*path.last().unwrap());
    result
}

fn heading_between(from_x: f64, from_z: f64, to_x: f64, to_z: f64) -> f64 {
    let dx = to_x - from_x;
    let dz = to_z - from_z;
    let rad = dz.atan2(dx);
    let mut deg = rad * 180.0 / std::f64::consts::PI;
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

fn normalize_angle(mut deg: f64) -> f64 {
    while deg > 180.0 {
        deg -= 360.0;
    }
    while deg <= -180.0 {
        deg += 360.0;
    }
    deg
}

fn make_command(id: &str, ct: CommandType, params: &[(&str, ParamValue)]) -> Command {
    let mut map = BTreeMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.clone());
    }
    Command {
        id: id.to_string(),
        command_type: ct,
        params: map,
        children: Vec::new(),
    }
}

fn build_commands(
    path_segments: &[Vec<(f64, f64)>],
    _start_heading_deg: f64,
    cruise_speed: f64,
) -> (Vec<Command>, Vec<LegInfo>, Vec<usize>) {
    let mut commands = Vec::new();
    let mut legs = Vec::new();
    let mut cmd_id = 0usize;
    let mut current_heading = 0.0;

    commands.push(make_command(
        &format!("r{cmd_id}"),
        CommandType::Takeoff,
        &[],
    ));
    cmd_id += 1;

    let mut leg_ends = Vec::new();

    for (leg_idx, segment) in path_segments.iter().enumerate() {
        let mut leg_distance = 0.0;
        let mut leg_turns = 0;

        for i in 0..segment.len() - 1 {
            let (fx, fz) = segment[i];
            let (tx, tz) = segment[i + 1];
            let d = dist2(fx, fz, tx, tz);

            if d < 1e-9 {
                continue;
            }

            let target_heading = heading_between(fx, fz, tx, tz);
            let turn = normalize_angle(target_heading - current_heading);

            if turn.abs() > 0.5 {
                commands.push(make_command(
                    &format!("r{cmd_id}"),
                    CommandType::TurnDegree,
                    &[
                        ("deg", ParamValue::Number(turn)),
                        ("timeout", ParamValue::Number(3.0)),
                        ("p_value", ParamValue::Number(10.0)),
                    ],
                ));
                cmd_id += 1;
                leg_turns += 1;
                current_heading = target_heading;
            }

            let dist_cm = d * 100.0;
            commands.push(make_command(
                &format!("r{cmd_id}"),
                CommandType::MoveForward,
                &[
                    ("dist", ParamValue::Number(dist_cm)),
                    ("speed", ParamValue::Number(cruise_speed)),
                ],
            ));
            cmd_id += 1;
            leg_distance += d;
        }

        legs.push(LegInfo {
            leg_index: leg_idx,
            distance_m: leg_distance,
            turn_count: leg_turns,
        });
        leg_ends.push(cmd_id - 1);
    }

    commands.push(make_command(
        &format!("r{cmd_id}"),
        CommandType::Land,
        &[],
    ));

    (commands, legs, leg_ends)
}

pub fn verify_route(
    commands: &[Command],
    waypoints: &[[f64; 3]],
    obstacles: &ObstacleSet,
    start: [f64; 3],
    leg_ends: &[usize],
    path: &[[f64; 3]],
    margin: f64,
) -> (bool, usize) {
    let result = sim::simulate_commands(commands, Some(obstacles));
    if !result.collisions.is_empty() {
        return (false, 0);
    }
    
    let mut min_c = f64::INFINITY;
    for i in 0..path.len().saturating_sub(1) {
        let ax = path[i][0];
        let az = path[i][2];
        let bx = path[i + 1][0];
        let bz = path[i + 1][2];
        let flight_y = path[i][1];
        for obstacle in &obstacles.obstacles {
            if obstacle.is_pad() { continue; }
            if crate::obstacles::type_dims(&obstacle.obstacle_type).is_none() { continue; }
            let wb = obstacles.world_box(obstacle);
            if wb.max[1] < flight_y || wb.min[1] > flight_y { continue; }
            
            let mut d = f64::INFINITY;
            d = d.min(point_to_rect_dist(ax, az, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
            d = d.min(point_to_rect_dist(bx, bz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
            
            let dx = bx - ax;
            let dz = bz - az;
            let len2 = dx * dx + dz * dz;
            if len2 > 1e-12 {
                for &rx in &[wb.min[0], wb.max[0]] {
                    let t = (rx - ax) / dx;
                    if t >= 0.0 && t <= 1.0 {
                        let pz = az + t * dz;
                        d = d.min(point_to_rect_dist(rx, pz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                    }
                }
                for &rz in &[wb.min[2], wb.max[2]] {
                    let t = (rz - az) / dz;
                    if t >= 0.0 && t <= 1.0 {
                        let px = ax + t * dx;
                        d = d.min(point_to_rect_dist(px, rz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                    }
                }
                for &(cx, cz) in &[(wb.min[0], wb.min[2]), (wb.min[0], wb.max[2]), (wb.max[0], wb.min[2]), (wb.max[0], wb.max[2])] {
                    let t = ((cx - ax) * dx + (cz - az) * dz) / len2;
                    if t >= 0.0 && t <= 1.0 {
                        let px = ax + t * dx;
                        let pz = az + t * dz;
                        d = d.min(point_to_rect_dist(px, pz, wb.min[0], wb.max[0], wb.min[2], wb.max[2]));
                    }
                }
            }
            if d < min_c { min_c = d; }
        }
    }
    if min_c < margin - 1e-9 {
        return (false, 0);
    }

    let mut reached = 0usize;
    for (i, wp) in waypoints.iter().enumerate() {
        let sim_x = wp[0] - start[0];
        let sim_y = wp[2] - start[2];
        let end_idx = *leg_ends.get(i).unwrap_or(&0);
        if end_idx >= commands.len() { continue; }
        let res = sim::simulate_commands(&commands[0..=end_idx], Some(obstacles));
        if let Some(pt) = res.positions.last() {
            let dx = pt.x - sim_x;
            let dy = pt.y - sim_y;
            let d = (dx * dx + dy * dy).sqrt();
            if d < WAYPOINT_REACH_TOLERANCE {
                reached += 1;
            }
        }
    }
    let last_cmd = commands.last();
    let ends_with_land = last_cmd
        .map(|c| c.command_type == CommandType::Land)
        .unwrap_or(false);
    let ok = reached == waypoints.len() && ends_with_land;
    (ok, reached)
}

pub fn route_through(
    waypoints: &[[f64; 3]],
    start: [f64; 3],
    start_heading_deg: f64,
    obstacles: &ObstacleSet,
    opts: &RouteOptions,
) -> RouteResult {
    if waypoints.is_empty() {
        return RouteResult {
            commands: Vec::new(),
            path: Vec::new(),
            waypoints_reached: 0,
            feasible: false,
            failure: Some(RouteFailure::EmptyWaypoints),
            legs: Vec::new(),
        };
    }

    let margin = DRONE_RADIUS + opts.clearance_m;
    let flight_y = start[1];

    if point_inside_inflated_3d(start, obstacles, margin) {
        return RouteResult {
            commands: Vec::new(),
            path: vec![start],
            waypoints_reached: 0,
            feasible: false,
            failure: Some(RouteFailure::StartInsideObstacle),
            legs: Vec::new(),
        };
    }

    for (i, wp) in waypoints.iter().enumerate() {
        if point_inside_inflated_3d(*wp, obstacles, margin) {
            return RouteResult {
                commands: Vec::new(),
                path: vec![start],
                waypoints_reached: 0,
                feasible: false,
                failure: Some(RouteFailure::WaypointInsideObstacle(i)),
                legs: Vec::new(),
            };
        }
    }

    let rects = inflate_obstacles_2d(obstacles, margin, flight_y);

    let mut all_xz: Vec<(f64, f64)> = vec![(start[0], start[2])];
    for wp in waypoints {
        all_xz.push((wp[0], wp[2]));
    }

    let mut path_segments: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut full_path: Vec<[f64; 3]> = vec![start];

    for leg_idx in 0..all_xz.len() - 1 {
        let (sx, sz) = all_xz[leg_idx];
        let (gx, gz) = all_xz[leg_idx + 1];

        let raw_path = match opts.optimize {
            Optimize::Shortest => find_path_shortest_2d(sx, sz, gx, gz, &rects),
            Optimize::WidestClearance => {
                find_path_widest_2d(sx, sz, gx, gz, &rects, obstacles, flight_y)
            }
            Optimize::Balanced => {
                find_path_balanced_2d(sx, sz, gx, gz, &rects, obstacles, flight_y)
            }
        };

        let raw_path = match raw_path {
            Some(p) => p,
            None => {
                return RouteResult {
                    commands: Vec::new(),
                    path: full_path,
                    waypoints_reached: leg_idx,
                    feasible: false,
                    failure: Some(RouteFailure::NoPath(leg_idx)),
                    legs: Vec::new(),
                };
            }
        };

        let simplified = simplify_path_2d(&raw_path);

        for seg_i in 0..simplified.len() - 1 {
            let mc = min_clearance_segment_2d(
                simplified[seg_i].0,
                simplified[seg_i].1,
                simplified[seg_i + 1].0,
                simplified[seg_i + 1].1,
                obstacles,
                flight_y,
            );
            if mc < DRONE_RADIUS - 1e-9 {
                return RouteResult {
                    commands: Vec::new(),
                    path: full_path,
                    waypoints_reached: leg_idx,
                    feasible: false,
                    failure: Some(RouteFailure::ClearanceUnmet(leg_idx)),
                    legs: Vec::new(),
                };
            }
        }

        let turn_count = simplified.len().saturating_sub(2);
        if turn_count > MAX_TURNS_PER_LEG {
            return RouteResult {
                commands: Vec::new(),
                path: full_path,
                waypoints_reached: leg_idx,
                feasible: false,
                failure: Some(RouteFailure::TurnBudgetExceeded(leg_idx)),
                legs: Vec::new(),
            };
        }

        for pt in simplified.iter().skip(1) {
            full_path.push([pt.0, flight_y, pt.1]);
        }
        path_segments.push(simplified);
    }

    let (commands, legs, leg_ends) =
        build_commands(&path_segments, start_heading_deg, opts.cruise_speed);

    let (sim_ok, reached) = verify_route(&commands, waypoints, obstacles, start, &leg_ends, &full_path, margin);

    if !sim_ok {
        return RouteResult {
            commands: Vec::new(),
            path: full_path,
            waypoints_reached: reached,
            feasible: false,
            failure: Some(RouteFailure::ClearanceUnmet(0)),
            legs,
        };
    }

    RouteResult {
        commands,
        path: full_path,
        waypoints_reached: reached,
        feasible: true,
        failure: None,
        legs,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacles::{Boundary, Obstacle, ObstacleSet};

    fn path_min_clearance_2d(path: &[(f64, f64)], obstacles: &ObstacleSet, flight_y: f64) -> f64 {
        let mut min_c = f64::INFINITY;
        for i in 0..path.len() - 1 {
            let c = min_clearance_segment_2d(
                path[i].0, path[i].1, path[i + 1].0, path[i + 1].1, obstacles, flight_y,
            );
            if c < min_c {
                min_c = c;
            }
        }
        min_c
    }

    fn empty_obstacles() -> ObstacleSet {
        ObstacleSet {
            obstacles: Vec::new(),
            boundary: Boundary {
                min_x: -10.0,
                max_x: 10.0,
                min_z: -10.0,
                max_z: 10.0,
                max_y: 10.0,
            },
        }
    }

    fn wall_obstacle(id: &str, pos: [f64; 3], name: &str) -> Obstacle {
        Obstacle {
            id: id.to_string(),
            obstacle_type: "wall".to_string(),
            position: pos,
            rotation: [0.0, 0.0, 0.0],
            scale: [1.0, 1.0, 1.0],
            name: name.to_string(),
            color: None,
        }
    }

    fn obs_set_with(obstacles: Vec<Obstacle>) -> ObstacleSet {
        ObstacleSet {
            obstacles,
            boundary: Boundary {
                min_x: -10.0,
                max_x: 10.0,
                min_z: -10.0,
                max_z: 10.0,
                max_y: 10.0,
            },
        }
    }

    fn default_opts(optimize: Optimize) -> RouteOptions {
        RouteOptions {
            clearance_m: 0.05,
            optimize,
            cruise_speed: 50.0,
            takeoff_height_m: 0.8,
        }
    }

    #[test]
    fn straight_line_no_obstacles() {
        let obs = empty_obstacles();
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[2.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(result.feasible, "not feasible: {:?}", result.failure);
        assert_eq!(result.waypoints_reached, 1);
        assert!(result.failure.is_none());
        let (sim_ok, _) = verify_route(&result.commands, &wps, &obs, start, &[result.commands.len().saturating_sub(2)], &result.path, DRONE_RADIUS + opts.clearance_m);
        assert!(sim_ok);
    }

    #[test]
    fn routes_around_wall() {
        let wall = wall_obstacle("w1", [1.0, 0.5, 0.0], "Wall");
        let obs = obs_set_with(vec![wall]);
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[2.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(result.feasible, "not feasible: {:?}", result.failure);
        assert_eq!(result.waypoints_reached, 1);
        assert!(result.path.len() > 2);
    }

    #[test]
    fn enclosed_waypoint_infeasible() {
        let big_box = Obstacle {
            id: "box".to_string(),
            obstacle_type: "wall".to_string(),
            position: [2.0, 0.5, 0.0],
            rotation: [0.0, 0.0, 0.0],
            scale: [4.0, 4.0, 4.0],
            name: "Enclosure".to_string(),
            color: None,
        };
        let obs = obs_set_with(vec![big_box]);
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[2.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(!result.feasible);
        assert!(result.failure.is_some());
    }

    #[test]
    fn start_inside_obstacle() {
        let wall = wall_obstacle("w1", [0.0, 0.5, 0.0], "Wall");
        let obs = obs_set_with(vec![wall]);
        let start = [0.0, 0.5, 0.0];
        let wps = vec![[3.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(!result.feasible);
        match result.failure {
            Some(RouteFailure::StartInsideObstacle) => {}
            _ => panic!("expected StartInsideObstacle, got {:?}", result.failure),
        }
    }

    #[test]
    fn multi_leg_obstacle_on_leg2() {
        let wall = wall_obstacle("w1", [3.0, 0.5, 0.0], "Wall");
        let obs = obs_set_with(vec![wall]);
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[1.5, 0.8, 0.0], [5.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(result.feasible, "not feasible: {:?}", result.failure);
        assert_eq!(result.waypoints_reached, 2);
    }

    #[test]
    fn all_three_optimize_modes() {
        let wall = wall_obstacle("w1", [1.0, 0.5, 0.0], "Wall");
        let obs = obs_set_with(vec![wall]);
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[2.0, 0.8, 0.0]];

        let r_short = route_through(&wps, start, 0.0, &obs, &default_opts(Optimize::Shortest));
        let r_wide = route_through(
            &wps,
            start,
            0.0,
            &obs,
            &default_opts(Optimize::WidestClearance),
        );
        let r_bal = route_through(&wps, start, 0.0, &obs, &default_opts(Optimize::Balanced));

        assert!(r_short.feasible, "Shortest not feasible: {:?}", r_short.failure);
        assert!(r_wide.feasible, "WidestClearance not feasible: {:?}", r_wide.failure);
        assert!(r_bal.feasible, "Balanced not feasible: {:?}", r_bal.failure);

        let flight_y = start[1];
        let short_path: Vec<(f64, f64)> =
            r_short.path.iter().map(|p| (p[0], p[2])).collect();
        let wide_path: Vec<(f64, f64)> =
            r_wide.path.iter().map(|p| (p[0], p[2])).collect();

        let c_short = path_min_clearance_2d(&short_path, &obs, flight_y);
        let c_wide = path_min_clearance_2d(&wide_path, &obs, flight_y);

        assert!(
            c_wide >= c_short - 1e-6,
            "WidestClearance ({c_wide}) should have >= clearance than Shortest ({c_short})"
        );
    }

    #[test]
    fn non_origin_start_routes_successfully() {
        let obs = empty_obstacles();
        let opts = default_opts(Optimize::Shortest);
        let start = [2.0, 0.8, 3.0];
        let wps = vec![[4.0, 0.8, 3.0]];
        let res = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(
            res.feasible,
            "a clear straight line from a non-origin start must be routable, got {:?}",
            res.failure
        );
        assert_eq!(res.waypoints_reached, 1);
    }

    #[test]
    fn non_origin_multi_leg_route_succeeds() {
        let obs = empty_obstacles();
        let opts = default_opts(Optimize::WidestClearance);
        let start = [-3.0, 0.8, 1.5];
        let wps = vec![[0.0, 0.8, 1.5], [3.0, 0.8, -2.0]];
        let res = route_through(&wps, start, 45.0, &obs, &opts);
        assert!(
            res.feasible,
            "multi-leg route from a non-origin start must be routable, got {:?}",
            res.failure
        );
        assert_eq!(res.waypoints_reached, 2);
    }

    #[test]
    fn empty_waypoints() {
        let obs = empty_obstacles();
        let start = [0.0, 0.8, 0.0];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&[], start, 0.0, &obs, &opts);
        assert!(!result.feasible);
        match result.failure {
            Some(RouteFailure::EmptyWaypoints) => {}
            _ => panic!("expected EmptyWaypoints"),
        }
    }

    #[test]
    fn commands_start_takeoff_end_land() {
        let obs = empty_obstacles();
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[1.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(result.feasible, "not feasible: {:?}", result.failure);
        assert!(!result.commands.is_empty());
        assert_eq!(result.commands.first().unwrap().command_type, CommandType::Takeoff);
        assert_eq!(result.commands.last().unwrap().command_type, CommandType::Land);
    }

    #[test]
    fn verify_rejects_path_ending_far_from_waypoint() {
        let obs = empty_obstacles();
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[1.0, 0.8, 0.0]];
        let opts = default_opts(Optimize::Shortest);
        let mut result = route_through(&wps, start, 0.0, &obs, &opts);
        assert!(result.feasible);
        result.commands.pop();
        result.commands.push(make_command(
            "r_test",
            CommandType::MoveForward,
            &[("dist", ParamValue::Number(400.0)), ("speed", ParamValue::Number(50.0))],
        ));
        result.commands.push(make_command("r_test2", CommandType::Land, &[]));
        let (ok, _) = verify_route(&result.commands, &wps, &obs, start, &[result.commands.len().saturating_sub(2)], &result.path, DRONE_RADIUS + opts.clearance_m);
        assert!(!ok);
    }

    #[test]
    fn verify_rejects_intermediate_proximity() {
        let obs = empty_obstacles();
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[1.0, 0.8, 0.0], [5.0, 0.8, 0.0]];
        let mut commands = Vec::new();
        commands.push(make_command("c1", CommandType::Takeoff, &[]));
        commands.push(make_command(
            "c2",
            CommandType::MoveForward,
            &[("dist", ParamValue::Number(100.0)), ("speed", ParamValue::Number(50.0))],
        ));
        let leg0_end = commands.len() - 1;
        commands.push(make_command(
            "c3",
            CommandType::MoveForward,
            &[("dist", ParamValue::Number(400.0)), ("speed", ParamValue::Number(50.0))],
        ));
        let leg1_end = commands.len() - 1;
        commands.push(make_command("c4", CommandType::Land, &[]));
        let path = vec![[0.0, 0.8, 0.0], [1.0, 0.8, 0.0], [5.0, 0.8, 0.0]];
        let (ok, reached) = verify_route(
            &commands,
            &wps,
            &obs,
            start,
            &[leg0_end, leg1_end],
            &path,
            DRONE_RADIUS,
        );
        assert!(ok);
        assert_eq!(reached, 2);

        let mut skewed = commands.clone();
        skewed.remove(skewed.len() - 2);
        skewed.insert(
            skewed.len() - 1,
            make_command(
                "c3b",
                CommandType::MoveForward,
                &[("dist", ParamValue::Number(50.0)), ("speed", ParamValue::Number(50.0))],
            ),
        );
        let (ok2, reached2) = verify_route(
            &skewed,
            &wps,
            &obs,
            start,
            &[leg0_end, leg0_end + 1],
            &path,
            DRONE_RADIUS,
        );
        assert!(!ok2);
        assert!(reached2 < 2);
    }

    fn exact_segment_to_rect_dist(ax: f64, az: f64, bx: f64, bz: f64, min_x: f64, max_x: f64, min_z: f64, max_z: f64) -> f64 {
        let mut min_dist = f64::INFINITY;
        min_dist = min_dist.min(point_to_rect_dist(ax, az, min_x, max_x, min_z, max_z));
        min_dist = min_dist.min(point_to_rect_dist(bx, bz, min_x, max_x, min_z, max_z));
        let dx = bx - ax;
        let dz = bz - az;
        let len2 = dx * dx + dz * dz;
        if len2 > 1e-12 {
            for &rx in &[min_x, max_x] {
                let t = (rx - ax) / dx;
                if t >= 0.0 && t <= 1.0 {
                    let pz = az + t * dz;
                    min_dist = min_dist.min(point_to_rect_dist(rx, pz, min_x, max_x, min_z, max_z));
                }
            }
            for &rz in &[min_z, max_z] {
                let t = (rz - az) / dz;
                if t >= 0.0 && t <= 1.0 {
                    let px = ax + t * dx;
                    min_dist = min_dist.min(point_to_rect_dist(px, rz, min_x, max_x, min_z, max_z));
                }
            }
            for &(cx, cz) in &[(min_x, min_z), (min_x, max_z), (max_x, min_z), (max_x, max_z)] {
                let t = ((cx - ax) * dx + (cz - az) * dz) / len2;
                if t >= 0.0 && t <= 1.0 {
                    let px = ax + t * dx;
                    let pz = az + t * dz;
                    min_dist = min_dist.min(point_to_rect_dist(px, pz, min_x, max_x, min_z, max_z));
                }
            }
        }
        min_dist
    }

    fn exact_path_min_clearance_2d(path: &[(f64, f64)], obstacles: &ObstacleSet, flight_y: f64) -> f64 {
        let mut min_c = f64::INFINITY;
        for i in 0..path.len() - 1 {
            for obs in &obstacles.obstacles {
                if obs.is_pad() { continue; }
                let wb = obstacles.world_box(obs);
                if wb.max[1] < flight_y || wb.min[1] > flight_y { continue; }
                let c = exact_segment_to_rect_dist(
                    path[i].0, path[i].1, path[i + 1].0, path[i + 1].1,
                    wb.min[0], wb.max[0], wb.min[2], wb.max[2]
                );
                if c < min_c { min_c = c; }
            }
        }
        min_c
    }

    #[test]
    fn path_clearance_meets_margin_all_modes() {
        let wall = Obstacle {
            id: "w1".to_string(),
            obstacle_type: "wall".to_string(),
            position: [1.0, 0.5, 0.0],
            rotation: [0.0, 0.0, 0.0],
            scale: [3.0, 1.0, 1.0],
            name: "Wall".to_string(),
            color: None,
        };
        let obs = obs_set_with(vec![wall]);
        let start = [0.0, 0.8, 0.0];
        let wps = vec![[4.0, 0.8, 0.0]];
        for opt in [Optimize::Shortest, Optimize::WidestClearance, Optimize::Balanced] {
            let opts = default_opts(opt);
            let mut result = route_through(&wps, start, 0.0, &obs, &opts);
            let flight_y = start[1];
            let path: Vec<(f64, f64)> = result.path.iter().map(|p| (p[0], p[2])).collect();
            let c = exact_path_min_clearance_2d(&path, &obs, flight_y);
            assert!(c >= DRONE_RADIUS + opts.clearance_m - 1e-6, "Mode {:?} clearance {} < margin", opt, c);
        }
    }
}
