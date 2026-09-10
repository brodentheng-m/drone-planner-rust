pub mod controlflow;
pub mod drive;
pub mod moves;
pub mod runtime;
pub mod shapes;

use crate::commands::Command;
use crate::golden::SimResult;
use crate::obstacles::ObstacleSet;
use crate::planio::Plan;
use crate::sim::drive::SimState;
use std::collections::BTreeMap;

pub type OnCommandStart<'a> = &'a mut dyn FnMut(&Command, usize, &SimState);

pub fn simulate_commands(commands: &[Command], obstacles: Option<&ObstacleSet>) -> SimResult {
    let _ = (commands, obstacles);
    SimResult::default()
}

pub fn simulate_swarm(
    plan: &Plan,
    obstacles: Option<&ObstacleSet>,
) -> BTreeMap<String, SimResult> {
    simulate_swarm_with(plan, obstacles, None)
}

pub fn simulate_swarm_with(
    plan: &Plan,
    obstacles: Option<&ObstacleSet>,
    on_start: Option<OnCommandStart>,
) -> BTreeMap<String, SimResult> {
    let _ = (plan, obstacles, on_start);
    BTreeMap::new()
}
