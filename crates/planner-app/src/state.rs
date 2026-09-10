use planner_core::golden::SimResult;
use planner_core::obstacles::ObstacleSet;
use planner_core::planio::Plan;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Default)]
pub struct Selection {
    pub drone_index: Option<usize>,
    pub command_path: Vec<usize>,
}

#[derive(Debug, Clone, Default)]
pub struct Playback {
    pub playing: bool,
    pub frame: usize,
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
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        AppState::new()
    }
}
