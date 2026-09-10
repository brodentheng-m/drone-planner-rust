use crate::commands::Command;
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

pub fn define_function(ctx: &mut ControlContext, name: &str, children: Vec<Command>) {
    ctx.functions.insert(func_key(name), children);
}

pub fn call_function<'a>(ctx: &'a ControlContext, name: &str) -> Option<&'a Vec<Command>> {
    ctx.functions.get(&func_key(name))
}

fn func_key(name: &str) -> String {
    format!("__func_{name}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::CommandType;

    #[test]
    fn function_storage_uses_func_prefix() {
        let mut ctx = ControlContext::new();
        let body = vec![Command::new("f1", CommandType::Hover)];
        define_function(&mut ctx, "hop", body);
        assert!(ctx.functions.contains_key("__func_hop"));
        assert!(call_function(&ctx, "hop").is_some());
        assert!(call_function(&ctx, "missing").is_none());
        assert_eq!(call_function(&ctx, "hop").unwrap().len(), 1);
    }
}
