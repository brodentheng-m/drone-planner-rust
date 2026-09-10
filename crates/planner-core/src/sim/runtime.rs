use crate::commands::js_num;
use std::collections::BTreeMap;

pub const SENSOR_BATTERY: f64 = 80.0;
pub const SENSOR_FRONT_RANGE: f64 = 100.0;
pub const SENSOR_FRONT_COLOR: &str = "green";
pub const SENSOR_BACK_COLOR: &str = "blue";
pub const SENSOR_TEMPERATURE: f64 = 22.0;

#[derive(Debug, Clone, PartialEq)]
pub enum VarValue {
    Num(f64),
    Str(String),
    List(Vec<f64>),
}

#[derive(Debug, Clone, Default)]
pub struct RuntimeState {
    pub vars: BTreeMap<String, VarValue>,
}

impl RuntimeState {
    pub fn new() -> RuntimeState {
        RuntimeState::default()
    }
}

pub fn eval_expr(expr: &str, vars: &BTreeMap<String, VarValue>) -> f64 {
    let replaced = replace_vars(expr, vars);
    parse_expression(&replaced).unwrap_or(0.0)
}

pub fn replace_vars(expr: &str, vars: &BTreeMap<String, VarValue>) -> String {
    let mut names: Vec<&String> = vars.keys().collect();
    names.sort_by(|a, b| b.len().cmp(&a.len()).then(a.cmp(b)));
    let mut result = expr.to_string();
    for name in names {
        let value = match vars.get(name.as_str()) {
            Some(v) => stringify_value(v),
            None => continue,
        };
        result = replace_word(&result, name, &value);
    }
    result
}

pub fn stringify_value(value: &VarValue) -> String {
    match value {
        VarValue::Num(n) => js_num(*n),
        VarValue::Str(text) => text.clone(),
        VarValue::List(items) => {
            let parts: Vec<String> = items.iter().map(|v| js_num(*v)).collect();
            format!("[{}]", parts.join(", "))
        }
    }
}

pub fn coerce_num(value: &VarValue) -> f64 {
    match value {
        VarValue::Num(n) => *n,
        VarValue::Str(text) => js_number(text),
        VarValue::List(items) => js_number(&list_to_string(items)),
    }
}

pub fn compound_set(current: &VarValue, op: &str, value: f64) -> VarValue {
    if op == "+=" {
        return match current {
            VarValue::Str(text) if !text.is_empty() => VarValue::Str(text.clone() + &js_num(value)),
            VarValue::List(items) => VarValue::Str(list_to_string(items) + &js_num(value)),
            _ => VarValue::Num(coerce_num(current) + value),
        };
    }
    if op == "/=" && value == 0.0 {
        return VarValue::Num(0.0);
    }
    let base = coerce_num(current);
    match op {
        "-=" => VarValue::Num(base - value),
        "*=" => VarValue::Num(base * value),
        "/=" => VarValue::Num(base / value),
        _ => VarValue::Num(value),
    }
}

pub fn get_battery() -> f64 {
    SENSOR_BATTERY
}

pub fn get_height(z: f64) -> f64 {
    z * 100.0
}

pub fn get_front_range() -> f64 {
    SENSOR_FRONT_RANGE
}

pub fn get_bottom_range(z: f64) -> f64 {
    z * 100.0
}

pub fn get_front_color() -> &'static str {
    SENSOR_FRONT_COLOR
}

pub fn get_back_color() -> &'static str {
    SENSOR_BACK_COLOR
}

pub fn get_temperature() -> f64 {
    SENSOR_TEMPERATURE
}

pub fn get_distance() -> f64 {
    SENSOR_FRONT_RANGE
}

pub fn list_append(state: &mut RuntimeState, name: &str, value: f64) {
    let entry = state
        .vars
        .entry(name.to_string())
        .or_insert_with(|| VarValue::List(Vec::new()));
    if !matches!(entry, VarValue::List(_)) {
        *entry = VarValue::List(Vec::new());
    }
    if let VarValue::List(items) = entry {
        items.push(value);
    }
}

pub fn list_get(state: &RuntimeState, name: &str, index: i64) -> Option<f64> {
    match state.vars.get(name) {
        Some(VarValue::List(items)) => {
            if index < 0 || index >= items.len() as i64 {
                return None;
            }
            items.get(index as usize).copied()
        }
        _ => None,
    }
}

pub fn timer_start(state: &mut RuntimeState, name: &str) {
    state.vars.insert(name.to_string(), VarValue::Num(0.0));
}

pub fn timer_elapsed(state: &RuntimeState, name: &str) -> f64 {
    let _ = (state, name);
    0.0
}

pub fn time_sleep(state: &mut RuntimeState, dur: f64) {
    let _ = (state, dur);
}

pub fn drone_sleep(state: &mut RuntimeState, dur: f64) {
    let _ = (state, dur);
}

fn is_word_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

fn replace_word(text: &str, name: &str, value: &str) -> String {
    if name.is_empty() {
        return text.to_string();
    }
    let mut out = String::with_capacity(text.len() + value.len());
    let mut rest = text;
    while let Some(start) = rest.find(name) {
        let before_ok = rest[..start]
            .chars()
            .next_back()
            .map_or(true, |c| !is_word_char(c));
        let after_ok = rest[start + name.len()..]
            .chars()
            .next()
            .map_or(true, |c| !is_word_char(c));
        let (head, tail) = rest.split_at(start);
        out.push_str(head);
        if before_ok && after_ok {
            out.push_str(value);
        } else {
            out.push_str(name);
        }
        rest = &tail[name.len()..];
    }
    out.push_str(rest);
    out
}

fn list_to_string(items: &[f64]) -> String {
    let parts: Vec<String> = items.iter().map(|v| js_num(*v)).collect();
    parts.join(",")
}

fn js_number(text: &str) -> f64 {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return 0.0;
    }
    trimmed.parse::<f64>().unwrap_or(f64::NAN)
}

fn js_truthy(v: f64) -> bool {
    v != 0.0 && !v.is_nan()
}

fn bool_num(v: bool) -> f64 {
    if v { 1.0 } else { 0.0 }
}

#[derive(Debug, Clone, Copy)]
enum CmpOp {
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
}

struct ExprParser<'a> {
    src: &'a [u8],
    pos: usize,
}

impl<'a> ExprParser<'a> {
    fn skip_ws(&mut self) {
        while self.pos < self.src.len() {
            match self.src[self.pos] {
                b' ' | b'\t' | b'\r' | b'\n' => self.pos += 1,
                _ => break,
            }
        }
    }

    fn peek(&self) -> Option<u8> {
        self.src.get(self.pos).copied()
    }

    fn peek_at(&self, offset: usize) -> Option<u8> {
        self.src.get(self.pos + offset).copied()
    }

    fn parse_expression(&mut self) -> Option<f64> {
        self.parse_or()
    }

    fn parse_or(&mut self) -> Option<f64> {
        let mut left = self.parse_and()?;
        loop {
            self.skip_ws();
            if self.peek() == Some(b'|') && self.peek_at(1) == Some(b'|') {
                self.pos += 2;
                let right = self.parse_and()?;
                left = if js_truthy(left) { left } else { right };
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_and(&mut self) -> Option<f64> {
        let mut left = self.parse_cmp()?;
        loop {
            self.skip_ws();
            if self.peek() == Some(b'&') && self.peek_at(1) == Some(b'&') {
                self.pos += 2;
                let right = self.parse_cmp()?;
                left = if js_truthy(left) { right } else { left };
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_cmp(&mut self) -> Option<f64> {
        let mut left = self.parse_add()?;
        loop {
            self.skip_ws();
            let op = match (self.peek(), self.peek_at(1)) {
                (Some(b'='), Some(b'=')) => {
                    self.pos += 2;
                    CmpOp::Eq
                }
                (Some(b'!'), Some(b'=')) => {
                    self.pos += 2;
                    CmpOp::Ne
                }
                (Some(b'<'), Some(b'=')) => {
                    self.pos += 2;
                    CmpOp::Le
                }
                (Some(b'>'), Some(b'=')) => {
                    self.pos += 2;
                    CmpOp::Ge
                }
                (Some(b'<'), _) => {
                    self.pos += 1;
                    CmpOp::Lt
                }
                (Some(b'>'), _) => {
                    self.pos += 1;
                    CmpOp::Gt
                }
                _ => break,
            };
            let right = self.parse_add()?;
            left = match op {
                CmpOp::Eq => bool_num(left == right),
                CmpOp::Ne => bool_num(left != right),
                CmpOp::Lt => bool_num(left < right),
                CmpOp::Le => bool_num(left <= right),
                CmpOp::Gt => bool_num(left > right),
                CmpOp::Ge => bool_num(left >= right),
            };
        }
        Some(left)
    }

    fn parse_add(&mut self) -> Option<f64> {
        let mut left = self.parse_mul()?;
        loop {
            self.skip_ws();
            match self.peek() {
                Some(b'+') => {
                    self.pos += 1;
                    let right = self.parse_mul()?;
                    left += right;
                }
                Some(b'-') => {
                    self.pos += 1;
                    let right = self.parse_mul()?;
                    left -= right;
                }
                _ => break,
            }
        }
        Some(left)
    }

    fn parse_mul(&mut self) -> Option<f64> {
        let mut left = self.parse_unary()?;
        loop {
            self.skip_ws();
            match self.peek() {
                Some(b'*') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left *= right;
                }
                Some(b'/') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left /= right;
                }
                Some(b'%') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left %= right;
                }
                _ => break,
            }
        }
        Some(left)
    }

    fn parse_unary(&mut self) -> Option<f64> {
        self.skip_ws();
        match self.peek() {
            Some(b'+') => {
                self.pos += 1;
                self.parse_unary()
            }
            Some(b'-') => {
                self.pos += 1;
                let v = self.parse_unary()?;
                Some(-v)
            }
            Some(b'!') => {
                self.pos += 1;
                let v = self.parse_unary()?;
                Some(if js_truthy(v) { 0.0 } else { 1.0 })
            }
            _ => self.parse_primary(),
        }
    }

    fn parse_primary(&mut self) -> Option<f64> {
        self.skip_ws();
        let byte = self.peek()?;
        if byte == b'(' {
            self.pos += 1;
            let v = self.parse_or()?;
            self.skip_ws();
            if self.peek() != Some(b')') {
                return None;
            }
            self.pos += 1;
            return Some(v);
        }
        if byte.is_ascii_digit()
            || (byte == b'.' && self.peek_at(1).map_or(false, |b| b.is_ascii_digit()))
        {
            return self.parse_number();
        }
        None
    }

    fn parse_number(&mut self) -> Option<f64> {
        let start = self.pos;
        while self.peek().map_or(false, |b| b.is_ascii_digit()) {
            self.pos += 1;
        }
        if self.peek() == Some(b'.') {
            self.pos += 1;
            while self.peek().map_or(false, |b| b.is_ascii_digit()) {
                self.pos += 1;
            }
        }
        if matches!(self.peek(), Some(b'e') | Some(b'E')) {
            let mut exp = self.pos + 1;
            if matches!(self.src.get(exp), Some(b'+') | Some(b'-')) {
                exp += 1;
            }
            if self.src.get(exp).map_or(false, |b| b.is_ascii_digit()) {
                self.pos = exp;
                while self.peek().map_or(false, |b| b.is_ascii_digit()) {
                    self.pos += 1;
                }
            } else {
                return None;
            }
        }
        let text = std::str::from_utf8(&self.src[start..self.pos]).ok()?;
        text.parse::<f64>().ok()
    }
}

fn parse_expression(src: &str) -> Option<f64> {
    let mut parser = ExprParser {
        src: src.as_bytes(),
        pos: 0,
    };
    let value = parser.parse_expression()?;
    parser.skip_ws();
    if parser.pos != parser.src.len() {
        return None;
    }
    Some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn assert_eval(expr: &str, expected: f64) {
        assert!(
            approx(eval_expr(expr, &BTreeMap::new()), expected),
            "expr {expr} -> {} expected {expected}",
            eval_expr(expr, &BTreeMap::new())
        );
    }

    fn vars_of(pairs: &[(&str, VarValue)]) -> BTreeMap<String, VarValue> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    #[test]
    fn eval_comparison_less() {
        assert_eval("3 > 4", 0.0);
    }

    #[test]
    fn eval_precedence_mul_over_add() {
        assert_eval("2 + 3 * 4", 14.0);
    }

    #[test]
    fn eval_parens_override() {
        assert_eval("(2+3)*4", 20.0);
    }

    #[test]
    fn eval_equality() {
        assert_eval("5 == 5", 1.0);
        assert_eval("5 != 5", 0.0);
        assert_eval("2 < 3", 1.0);
        assert_eval("3 <= 2", 0.0);
        assert_eval("4 >= 4", 1.0);
        assert_eval("1 < 2 < 3", 1.0);
    }

    #[test]
    fn eval_unknown_identifier_is_zero() {
        assert_eval("True", 0.0);
        assert_eval("bogus !!!", 0.0);
    }

    #[test]
    fn eval_unary_ops() {
        assert_eval("!0", 1.0);
        assert_eval("!3", 0.0);
        assert_eval("-5 + 3", -2.0);
        assert_eval("-(2 + 3)", -5.0);
        assert_eval("!!7", 1.0);
    }

    #[test]
    fn eval_arithmetic() {
        assert_eval("10 % 3", 1.0);
        assert_eval("7 / 2", 3.5);
        assert_eval("1e3 + .5", 1000.5);
        assert_eval("2 * -3", -6.0);
    }

    #[test]
    fn eval_logic_returns_operand_values() {
        assert_eval("0 || 5", 5.0);
        assert_eval("3 || 4", 3.0);
        assert_eval("3 && 4", 4.0);
        assert_eval("0 && 4", 0.0);
        assert_eval("3 > 4 || 1", 1.0);
        assert_eval("0/0 || 5", 5.0);
    }

    #[test]
    fn eval_parse_errors_are_zero() {
        assert_eval("", 0.0);
        assert_eval("(", 0.0);
        assert_eval("1 +", 0.0);
        assert_eval("(2+3", 0.0);
        assert_eval("2+3)", 0.0);
        assert_eval("5e", 0.0);
        assert_eval("3x", 0.0);
    }

    #[test]
    fn eval_with_var_lookup() {
        let vars = vars_of(&[("x", VarValue::Num(3.0))]);
        assert!(approx(eval_expr("x > 2", &vars), 1.0));
        assert!(approx(eval_expr("x * 2 + 1", &vars), 7.0));
    }

    #[test]
    fn replace_single_var() {
        let vars = vars_of(&[("x", VarValue::Num(3.0))]);
        assert_eq!(replace_vars("x > 4", &vars), "3 > 4");
    }

    #[test]
    fn replace_prefers_longer_names() {
        let vars = vars_of(&[("ab", VarValue::Num(1.0)), ("a", VarValue::Num(2.0))]);
        assert_eq!(replace_vars("ab", &vars), "1");
        assert_eq!(replace_vars("a + ab", &vars), "2 + 1");
    }

    #[test]
    fn replace_word_boundaries() {
        let vars = vars_of(&[("x", VarValue::Num(3.0))]);
        assert_eq!(replace_vars("x1 + xy + x", &vars), "x1 + xy + 3");
    }

    #[test]
    fn replace_value_formats() {
        let vars = vars_of(&[
            ("f", VarValue::Num(0.5)),
            ("s", VarValue::Str("green".to_string())),
            ("l", VarValue::List(vec![1.0, 2.5])),
        ]);
        assert_eq!(replace_vars("f", &vars), "0.5");
        assert_eq!(replace_vars("s == green", &vars), "green == green");
        assert_eq!(replace_vars("l", &vars), "[1, 2.5]");
    }

    #[test]
    fn sensor_stubs_match_js() {
        assert!(approx(get_battery(), 80.0));
        assert!(approx(get_height(0.8), 80.0));
        assert!(approx(get_front_range(), 100.0));
        assert!(approx(get_bottom_range(1.0), 100.0));
        assert_eq!(get_front_color(), "green");
        assert_eq!(get_back_color(), "blue");
        assert!(approx(get_temperature(), 22.0));
        assert!(approx(get_distance(), 100.0));
    }

    #[test]
    fn list_helpers() {
        let mut state = RuntimeState::new();
        list_append(&mut state, "items", 5.0);
        list_append(&mut state, "items", 7.0);
        assert!(approx(list_get(&state, "items", 1).unwrap(), 7.0));
        assert!(list_get(&state, "items", 2).is_none());
        assert!(list_get(&state, "items", -1).is_none());
        state.vars.insert("n".to_string(), VarValue::Num(1.0));
        assert!(list_get(&state, "n", 0).is_none());
    }

    #[test]
    fn timers_and_sleeps_are_deterministic() {
        let mut state = RuntimeState::new();
        timer_start(&mut state, "t0");
        assert_eq!(state.vars.get("t0"), Some(&VarValue::Num(0.0)));
        assert!(approx(timer_elapsed(&state, "t0"), 0.0));
        time_sleep(&mut state, 2.0);
        drone_sleep(&mut state, 2.0);
    }

    #[test]
    fn compound_set_semantics() {
        assert_eq!(
            compound_set(&VarValue::Num(2.0), "+=", 3.0),
            VarValue::Num(5.0)
        );
        assert_eq!(
            compound_set(&VarValue::Str("a".to_string()), "+=", 0.5),
            VarValue::Str("a0.5".to_string())
        );
        assert_eq!(
            compound_set(&VarValue::Str(String::new()), "+=", 3.0),
            VarValue::Num(3.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(2.0), "-=", 5.0),
            VarValue::Num(-3.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(4.0), "*=", 2.5),
            VarValue::Num(10.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(8.0), "/=", 0.0),
            VarValue::Num(0.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(8.0), "/=", 4.0),
            VarValue::Num(2.0)
        );
        assert_eq!(
            compound_set(&VarValue::Str("12.5".to_string()), "*=", 2.0),
            VarValue::Num(25.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(1.0), "=", 9.0),
            VarValue::Num(9.0)
        );
    }

    #[test]
    fn coerce_num_matches_js_number() {
        assert!(approx(coerce_num(&VarValue::Num(2.5)), 2.5));
        assert!(approx(coerce_num(&VarValue::Str("12.5".to_string())), 12.5));
        assert!(approx(coerce_num(&VarValue::Str(String::new())), 0.0));
        assert!(coerce_num(&VarValue::Str("abc".to_string())).is_nan());
        assert!(approx(coerce_num(&VarValue::List(vec![5.0])), 5.0));
        assert!(coerce_num(&VarValue::List(vec![1.0, 2.0])).is_nan());
    }
}
