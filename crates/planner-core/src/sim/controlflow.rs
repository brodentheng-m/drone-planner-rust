use crate::commands::Command;
use crate::sim::drive::SimState;
use crate::sim::runtime::RuntimeState;
use std::collections::BTreeMap;

pub const MAX_ITER: u64 = 10000;
pub const MAX_WHILE_LOOPS: u64 = 500;

#[derive(Debug, Clone, Default)]
pub struct ControlContext {
    pub functions: BTreeMap<String, Vec<Command>>,
    pub iter_count: u64,
}

impl ControlContext {
    pub fn new() -> ControlContext {
        ControlContext::default()
    }
}

pub fn execute_block(
    commands: &[Command],
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
) {
    let _ = (commands, state, runtime, ctx);
}

pub fn execute_if(
    condition: &str,
    children: &[Command],
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
) {
    let _ = (condition, children, state, runtime, ctx);
}

pub fn execute_while(
    condition: &str,
    children: &[Command],
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
) {
    let _ = (condition, children, state, runtime, ctx);
}

pub fn execute_for(
    var: &str,
    start: f64,
    end_val: f64,
    step: f64,
    children: &[Command],
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
) {
    let _ = (var, start, end_val, step, children, state, runtime, ctx);
}

pub fn define_function(ctx: &mut ControlContext, name: &str, children: Vec<Command>) {
    ctx.functions.insert(name.to_string(), children);
}

pub fn call_function(
    name: &str,
    state: &mut SimState,
    runtime: &mut RuntimeState,
    ctx: &mut ControlContext,
) {
    let _ = (name, state, runtime, ctx);
}
