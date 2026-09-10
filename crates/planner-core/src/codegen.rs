use crate::commands::Command;
use crate::planio::Plan;

pub fn generate_code(plan: &Plan) -> String {
    let _ = plan;
    String::new()
}

pub fn generate_swarm_code(plan: &Plan) -> String {
    let _ = plan;
    String::new()
}

pub fn generate_animation_code(plan: &Plan) -> String {
    let _ = plan;
    String::new()
}

#[derive(Debug, Clone, Default)]
pub struct ScriptParser;

impl ScriptParser {
    pub fn new() -> ScriptParser {
        ScriptParser
    }

    pub fn parse(&self, source: &str) -> Vec<Command> {
        let _ = source;
        Vec::new()
    }
}
