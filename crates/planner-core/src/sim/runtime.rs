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
    List(Vec<VarValue>),
    Bool(bool),
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
    match eval_value(expr, vars) {
        Some(value) => js_to_number(&value),
        None => 0.0,
    }
}

pub fn eval_truthy(expr: &str, vars: &BTreeMap<String, VarValue>) -> bool {
    match eval_value(expr, vars) {
        Some(value) => js_truthy_value(&value),
        None => false,
    }
}

pub fn eval_var(expr: &str, vars: &BTreeMap<String, VarValue>) -> VarValue {
    match eval_value(expr, vars) {
        Some(value) => js_value_to_var(value),
        None => VarValue::Num(0.0),
    }
}

fn js_value_to_var(value: JsValue) -> VarValue {
    match value {
        JsValue::Num(n) => VarValue::Num(n),
        JsValue::Str(text) => VarValue::Str(text),
        JsValue::Bool(flag) => VarValue::Bool(flag),
        JsValue::Arr(items) => VarValue::List(items.into_iter().map(js_value_to_var).collect()),
        JsValue::Null | JsValue::Undefined => VarValue::Num(f64::NAN),
    }
}

fn eval_value(expr: &str, vars: &BTreeMap<String, VarValue>) -> Option<JsValue> {
    let replaced = replace_vars(expr, vars);
    parse_expression(&replaced)
}

pub fn replace_vars(expr: &str, vars: &BTreeMap<String, VarValue>) -> String {
    let mut names: Vec<&String> = vars.keys().collect();
    names.sort_by(|a, b| b.len().cmp(&a.len()).then(a.cmp(b)));
    let mut result: Vec<char> = expr.chars().collect();
    for name in names {
        let value = match vars.get(name.as_str()) {
            Some(v) => stringify_value(v),
            None => continue,
        };
        let pattern = match compile_regex(&format!("\\b{name}\\b")) {
            Some(re) => re,
            None => return String::new(),
        };
        result = regex_replace_global(&pattern, &result, &value);
    }
    result.into_iter().collect()
}

pub fn stringify_value(value: &VarValue) -> String {
    match value {
        VarValue::Num(n) => json_number(*n),
        VarValue::Str(text) => json_string(text),
        VarValue::List(items) => {
            let parts: Vec<String> = items.iter().map(stringify_value).collect();
            format!("[{}]", parts.join(","))
        }
        VarValue::Bool(flag) => flag.to_string(),
    }
}

fn json_number(n: f64) -> String {
    if n.is_nan() || n.is_infinite() {
        "null".to_string()
    } else {
        js_num(n)
    }
}

fn json_string(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

pub fn coerce_num(value: &VarValue) -> f64 {
    var_to_number(value)
}

pub fn var_truthy(value: &VarValue) -> bool {
    match value {
        VarValue::Num(n) => *n != 0.0 && !n.is_nan(),
        VarValue::Str(text) => !text.is_empty(),
        VarValue::List(_) => true,
        VarValue::Bool(flag) => *flag,
    }
}

pub fn var_to_number(value: &VarValue) -> f64 {
    match value {
        VarValue::Num(n) => *n,
        VarValue::Str(text) => js_number(text),
        VarValue::List(items) => match items.len() {
            0 => 0.0,
            1 => var_to_number(&items[0]),
            _ => f64::NAN,
        },
        VarValue::Bool(flag) => {
            if *flag {
                1.0
            } else {
                0.0
            }
        }
    }
}

pub fn var_to_string(value: &VarValue) -> String {
    match value {
        VarValue::Num(n) => js_num(*n),
        VarValue::Str(text) => text.clone(),
        VarValue::List(items) => items
            .iter()
            .map(var_to_string)
            .collect::<Vec<String>>()
            .join(","),
        VarValue::Bool(flag) => flag.to_string(),
    }
}

fn or_zero(value: &VarValue) -> VarValue {
    if var_truthy(value) {
        value.clone()
    } else {
        VarValue::Num(0.0)
    }
}

fn var_add(a: &VarValue, b: &VarValue) -> VarValue {
    if matches!(a, VarValue::Str(_) | VarValue::List(_))
        || matches!(b, VarValue::Str(_) | VarValue::List(_))
    {
        VarValue::Str(var_to_string(a) + &var_to_string(b))
    } else {
        VarValue::Num(var_to_number(a) + var_to_number(b))
    }
}

fn is_strict_number_zero(value: &VarValue) -> bool {
    matches!(value, VarValue::Num(n) if *n == 0.0)
}

pub fn compound_set(current: &VarValue, op: &str, value: &VarValue) -> VarValue {
    match op {
        "=" => value.clone(),
        "+=" => var_add(&or_zero(current), value),
        "-=" => VarValue::Num(var_to_number(&or_zero(current)) - var_to_number(value)),
        "*=" => VarValue::Num(var_to_number(&or_zero(current)) * var_to_number(value)),
        "/=" => {
            if is_strict_number_zero(value) {
                VarValue::Num(0.0)
            } else {
                VarValue::Num(var_to_number(&or_zero(current)) / var_to_number(value))
            }
        }
        _ => current.clone(),
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

pub fn list_ensure(state: &mut RuntimeState, name: &str) {
    if !matches!(state.vars.get(name), Some(VarValue::List(_))) {
        state
            .vars
            .insert(name.to_string(), VarValue::List(Vec::new()));
    }
}

pub fn list_push(state: &mut RuntimeState, name: &str, value: VarValue) {
    if !matches!(state.vars.get(name), Some(VarValue::List(_))) {
        state
            .vars
            .insert(name.to_string(), VarValue::List(Vec::new()));
    }
    if let Some(VarValue::List(items)) = state.vars.get_mut(name) {
        items.push(value);
    }
}

pub fn list_get(state: &RuntimeState, name: &str, index: f64) -> VarValue {
    let items = match state.vars.get(name) {
        Some(VarValue::List(items)) => items,
        _ => return VarValue::Num(0.0),
    };
    if !index.is_finite() || index.fract() != 0.0 || index < 0.0 {
        return VarValue::Num(0.0);
    }
    match items.get(index as usize) {
        Some(value) if var_truthy(value) => value.clone(),
        _ => VarValue::Num(0.0),
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
    c.is_ascii_alphanumeric() || c == '_'
}

fn is_js_line_terminator(c: char) -> bool {
    matches!(c, '\n' | '\r' | '\u{2028}' | '\u{2029}')
}

fn is_js_space(c: char) -> bool {
    c.is_whitespace() || c == '\u{feff}'
}

#[derive(Debug, Clone)]
enum ClassItem {
    Char(char),
    Range(char, char),
    Digit(bool),
    Word(bool),
    Space(bool),
}

#[derive(Debug, Clone)]
enum Re {
    Empty,
    Lit(char),
    Any,
    Class(bool, Vec<ClassItem>),
    Start,
    End,
    WordBoundary(bool),
    Concat(Vec<Re>),
    Alt(Vec<Re>),
    Repeat(Box<Re>, u32, Option<u32>, bool),
}

struct RegexParser {
    src: Vec<char>,
    pos: usize,
}

impl RegexParser {
    fn parse(pattern: &str) -> Option<Re> {
        let mut parser = RegexParser {
            src: pattern.chars().collect(),
            pos: 0,
        };
        let node = parser.parse_alt()?;
        if parser.pos != parser.src.len() {
            return None;
        }
        Some(node)
    }

    fn peek(&self) -> Option<char> {
        self.src.get(self.pos).copied()
    }

    fn peek_at(&self, offset: usize) -> Option<char> {
        self.src.get(self.pos + offset).copied()
    }

    fn parse_alt(&mut self) -> Option<Re> {
        let mut alts = vec![self.parse_concat()?];
        while self.peek() == Some('|') {
            self.pos += 1;
            alts.push(self.parse_concat()?);
        }
        if alts.len() == 1 {
            alts.pop()
        } else {
            Some(Re::Alt(alts))
        }
    }

    fn parse_concat(&mut self) -> Option<Re> {
        let mut nodes = Vec::new();
        while let Some(c) = self.peek() {
            if c == '|' || c == ')' {
                break;
            }
            nodes.push(self.parse_repeat()?);
        }
        if nodes.is_empty() {
            Some(Re::Empty)
        } else if nodes.len() == 1 {
            nodes.pop()
        } else {
            Some(Re::Concat(nodes))
        }
    }

    fn parse_repeat(&mut self) -> Option<Re> {
        let atom = self.parse_atom()?;
        let (min, max) = match self.peek() {
            Some('*') => {
                self.pos += 1;
                (0, None)
            }
            Some('+') => {
                self.pos += 1;
                (1, None)
            }
            Some('?') => {
                self.pos += 1;
                (0, Some(1))
            }
            Some('{') => {
                let save = self.pos;
                match self.parse_braces() {
                    Some(bounds) => bounds,
                    None => {
                        self.pos = save;
                        return Some(atom);
                    }
                }
            }
            _ => return Some(atom),
        };
        let greedy = if self.peek() == Some('?') {
            self.pos += 1;
            false
        } else {
            true
        };
        Some(Re::Repeat(Box::new(atom), min, max, greedy))
    }

    fn parse_braces(&mut self) -> Option<(u32, Option<u32>)> {
        self.pos += 1;
        let min = self.parse_int()?;
        match self.peek() {
            Some('}') => {
                self.pos += 1;
                Some((min, Some(min)))
            }
            Some(',') => {
                self.pos += 1;
                if self.peek() == Some('}') {
                    self.pos += 1;
                    Some((min, None))
                } else {
                    let max = self.parse_int()?;
                    if self.peek() != Some('}') {
                        return None;
                    }
                    self.pos += 1;
                    Some((min, Some(max)))
                }
            }
            _ => None,
        }
    }

    fn parse_int(&mut self) -> Option<u32> {
        let start = self.pos;
        while self.peek().map_or(false, |c| c.is_ascii_digit()) {
            self.pos += 1;
        }
        if self.pos == start {
            return None;
        }
        self.src[start..self.pos]
            .iter()
            .collect::<String>()
            .parse()
            .ok()
    }

    fn parse_atom(&mut self) -> Option<Re> {
        let current = self.peek()?;
        self.pos += 1;
        match current {
            '(' => {
                if self.peek() == Some('?') {
                    self.pos += 1;
                    if self.peek() == Some(':') {
                        self.pos += 1;
                    } else {
                        return None;
                    }
                }
                let inner = self.parse_alt()?;
                if self.peek() != Some(')') {
                    return None;
                }
                self.pos += 1;
                Some(inner)
            }
            '[' => self.parse_class(),
            '.' => Some(Re::Any),
            '^' => Some(Re::Start),
            '$' => Some(Re::End),
            '\\' => self.parse_escape(),
            '*' | '+' | '?' => None,
            other => Some(Re::Lit(other)),
        }
    }

    fn parse_escape(&mut self) -> Option<Re> {
        let current = self.peek()?;
        self.pos += 1;
        match current {
            'd' => Some(Re::Class(false, vec![ClassItem::Digit(true)])),
            'D' => Some(Re::Class(false, vec![ClassItem::Digit(false)])),
            'w' => Some(Re::Class(false, vec![ClassItem::Word(true)])),
            'W' => Some(Re::Class(false, vec![ClassItem::Word(false)])),
            's' => Some(Re::Class(false, vec![ClassItem::Space(true)])),
            'S' => Some(Re::Class(false, vec![ClassItem::Space(false)])),
            'b' => Some(Re::WordBoundary(true)),
            'B' => Some(Re::WordBoundary(false)),
            'n' => Some(Re::Lit('\n')),
            't' => Some(Re::Lit('\t')),
            'r' => Some(Re::Lit('\r')),
            'f' => Some(Re::Lit('\u{c}')),
            'v' => Some(Re::Lit('\u{b}')),
            '0' => Some(Re::Lit('\0')),
            other => Some(Re::Lit(other)),
        }
    }

    fn parse_class_escape(&mut self) -> Option<ClassItem> {
        let current = self.peek()?;
        self.pos += 1;
        match current {
            'd' => Some(ClassItem::Digit(true)),
            'D' => Some(ClassItem::Digit(false)),
            'w' => Some(ClassItem::Word(true)),
            'W' => Some(ClassItem::Word(false)),
            's' => Some(ClassItem::Space(true)),
            'S' => Some(ClassItem::Space(false)),
            'n' => Some(ClassItem::Char('\n')),
            't' => Some(ClassItem::Char('\t')),
            'r' => Some(ClassItem::Char('\r')),
            'f' => Some(ClassItem::Char('\u{c}')),
            'v' => Some(ClassItem::Char('\u{b}')),
            'b' => Some(ClassItem::Char('\u{8}')),
            other => Some(ClassItem::Char(other)),
        }
    }

    fn parse_class(&mut self) -> Option<Re> {
        let negated = if self.peek() == Some('^') {
            self.pos += 1;
            true
        } else {
            false
        };
        let mut items = Vec::new();
        let mut first = true;
        loop {
            let current = self.peek()?;
            if current == ']' && !first {
                self.pos += 1;
                break;
            }
            first = false;
            let item = if current == '\\' {
                self.pos += 1;
                self.parse_class_escape()?
            } else {
                self.pos += 1;
                ClassItem::Char(current)
            };
            match item {
                ClassItem::Char(start) if self.peek() == Some('-') && self.peek_at(1) != Some(']') => {
                    self.pos += 1;
                    let end = if self.peek() == Some('\\') {
                        self.pos += 1;
                        match self.parse_class_escape()? {
                            ClassItem::Char(c) => c,
                            _ => return None,
                        }
                    } else {
                        let c = self.peek()?;
                        self.pos += 1;
                        c
                    };
                    items.push(ClassItem::Range(start, end));
                }
                other => items.push(other),
            }
        }
        Some(Re::Class(negated, items))
    }
}

fn compile_regex(pattern: &str) -> Option<Re> {
    RegexParser::parse(pattern)
}

fn class_item_matches(item: &ClassItem, c: char) -> bool {
    match item {
        ClassItem::Char(x) => *x == c,
        ClassItem::Range(a, b) => *a <= c && c <= *b,
        ClassItem::Digit(positive) => c.is_ascii_digit() == *positive,
        ClassItem::Word(positive) => is_word_char(c) == *positive,
        ClassItem::Space(positive) => is_js_space(c) == *positive,
    }
}

fn class_matches(negated: bool, items: &[ClassItem], c: char) -> bool {
    let hit = items.iter().any(|item| class_item_matches(item, c));
    hit != negated
}

fn re_match_seq(
    nodes: &[Re],
    text: &[char],
    pos: usize,
    cont: &mut dyn FnMut(usize) -> bool,
) -> bool {
    match nodes.split_first() {
        None => cont(pos),
        Some((head, tail)) => {
            re_match(head, text, pos, &mut |next| re_match_seq(tail, text, next, cont))
        }
    }
}

fn re_match_repeat(
    inner: &Re,
    min: u32,
    max: Option<u32>,
    greedy: bool,
    count: u32,
    text: &[char],
    pos: usize,
    cont: &mut dyn FnMut(usize) -> bool,
) -> bool {
    let can_more = max.map_or(true, |limit| count < limit);
    if greedy {
        if can_more
            && re_match(inner, text, pos, &mut |next| {
                if next == pos {
                    count + 1 >= min && cont(next)
                } else {
                    re_match_repeat(inner, min, max, greedy, count + 1, text, next, cont)
                }
            })
        {
            return true;
        }
        count >= min && cont(pos)
    } else {
        if count >= min && cont(pos) {
            return true;
        }
        can_more
            && re_match(inner, text, pos, &mut |next| {
                next != pos && re_match_repeat(inner, min, max, greedy, count + 1, text, next, cont)
            })
    }
}

fn re_match(node: &Re, text: &[char], pos: usize, cont: &mut dyn FnMut(usize) -> bool) -> bool {
    match node {
        Re::Empty => cont(pos),
        Re::Lit(c) => pos < text.len() && text[pos] == *c && cont(pos + 1),
        Re::Any => pos < text.len() && !is_js_line_terminator(text[pos]) && cont(pos + 1),
        Re::Class(negated, items) => {
            pos < text.len() && class_matches(*negated, items, text[pos]) && cont(pos + 1)
        }
        Re::Start => pos == 0 && cont(pos),
        Re::End => pos == text.len() && cont(pos),
        Re::WordBoundary(want) => {
            let before = pos > 0 && is_word_char(text[pos - 1]);
            let after = pos < text.len() && is_word_char(text[pos]);
            (before != after) == *want && cont(pos)
        }
        Re::Concat(nodes) => re_match_seq(nodes, text, pos, cont),
        Re::Alt(alts) => {
            for alt in alts {
                if re_match(alt, text, pos, cont) {
                    return true;
                }
            }
            false
        }
        Re::Repeat(inner, min, max, greedy) => {
            re_match_repeat(inner, *min, *max, *greedy, 0, text, pos, cont)
        }
    }
}

fn expand_replacement(replacement: &str, matched: &str, before: &str, after: &str) -> Vec<char> {
    let chars: Vec<char> = replacement.chars().collect();
    let mut out = Vec::new();
    let mut index = 0usize;
    while index < chars.len() {
        if chars[index] == '$' && index + 1 < chars.len() {
            match chars[index + 1] {
                '$' => {
                    out.push('$');
                    index += 2;
                }
                '&' => {
                    out.extend(matched.chars());
                    index += 2;
                }
                '`' => {
                    out.extend(before.chars());
                    index += 2;
                }
                '\'' => {
                    out.extend(after.chars());
                    index += 2;
                }
                _ => {
                    out.push('$');
                    index += 1;
                }
            }
        } else {
            out.push(chars[index]);
            index += 1;
        }
    }
    out
}

fn regex_replace_global(pattern: &Re, text: &[char], replacement: &str) -> Vec<char> {
    let mut out: Vec<char> = Vec::with_capacity(text.len());
    let mut index = 0usize;
    while index <= text.len() {
        let mut found: Option<(usize, usize)> = None;
        let mut start = index;
        while start <= text.len() {
            let mut end = None;
            let matched = re_match(pattern, text, start, &mut |position| {
                end = Some(position);
                true
            });
            if matched {
                found = Some((start, end.unwrap_or(start)));
                break;
            }
            start += 1;
        }
        match found {
            Some((match_start, match_end)) => {
                out.extend_from_slice(&text[index..match_start]);
                let matched: String = text[match_start..match_end].iter().collect();
                let before: String = text[..match_start].iter().collect();
                let after: String = text[match_end..].iter().collect();
                out.extend(expand_replacement(replacement, &matched, &before, &after));
                index = if match_end > match_start {
                    match_end
                } else {
                    match_start + 1
                };
            }
            None => {
                out.extend_from_slice(&text[index..]);
                break;
            }
        }
    }
    out
}

fn js_radix_number(text: &str) -> Option<f64> {
    let (radix, digits) = if let Some(rest) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        (16u32, rest)
    } else if let Some(rest) = text.strip_prefix("0b").or_else(|| text.strip_prefix("0B")) {
        (2u32, rest)
    } else if let Some(rest) = text.strip_prefix("0o").or_else(|| text.strip_prefix("0O")) {
        (8u32, rest)
    } else {
        return None;
    };
    if digits.is_empty() {
        return None;
    }
    let mut value = 0.0f64;
    for c in digits.chars() {
        value = value * radix as f64 + c.to_digit(radix)? as f64;
    }
    Some(value)
}

fn js_decimal_valid(text: &str) -> bool {
    let bytes = text.as_bytes();
    let mut index = 0usize;
    let mut mantissa_digits = 0usize;
    while index < bytes.len() && bytes[index].is_ascii_digit() {
        index += 1;
        mantissa_digits += 1;
    }
    if index < bytes.len() && bytes[index] == b'.' {
        index += 1;
        while index < bytes.len() && bytes[index].is_ascii_digit() {
            index += 1;
            mantissa_digits += 1;
        }
    }
    if mantissa_digits == 0 {
        return false;
    }
    if index < bytes.len() && (bytes[index] == b'e' || bytes[index] == b'E') {
        index += 1;
        if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
            index += 1;
        }
        let mut exponent_digits = 0usize;
        while index < bytes.len() && bytes[index].is_ascii_digit() {
            index += 1;
            exponent_digits += 1;
        }
        if exponent_digits == 0 {
            return false;
        }
    }
    index == bytes.len()
}

fn js_number(text: &str) -> f64 {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return 0.0;
    }
    let (negative, body) = match trimmed.as_bytes()[0] {
        b'+' => (false, &trimmed[1..]),
        b'-' => (true, &trimmed[1..]),
        _ => (false, trimmed),
    };
    if body == "Infinity" {
        return if negative {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
    }
    if !negative && trimmed.as_bytes()[0] != b'+' {
        if let Some(value) = js_radix_number(body) {
            return value;
        }
    }
    if !js_decimal_valid(body) {
        return f64::NAN;
    }
    match body.parse::<f64>() {
        Ok(value) => {
            if negative {
                -value
            } else {
                value
            }
        }
        Err(_) => f64::NAN,
    }
}

#[derive(Debug, Clone, PartialEq)]
enum JsValue {
    Undefined,
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<JsValue>),
}

fn js_truthy_value(value: &JsValue) -> bool {
    match value {
        JsValue::Undefined | JsValue::Null => false,
        JsValue::Bool(flag) => *flag,
        JsValue::Num(n) => *n != 0.0 && !n.is_nan(),
        JsValue::Str(text) => !text.is_empty(),
        JsValue::Arr(_) => true,
    }
}

fn js_to_number(value: &JsValue) -> f64 {
    match value {
        JsValue::Undefined => f64::NAN,
        JsValue::Null => 0.0,
        JsValue::Bool(flag) => {
            if *flag {
                1.0
            } else {
                0.0
            }
        }
        JsValue::Num(n) => *n,
        JsValue::Str(text) => js_number(text),
        JsValue::Arr(items) => {
            if items.is_empty() {
                0.0
            } else if items.len() == 1 {
                js_to_number(&items[0])
            } else {
                f64::NAN
            }
        }
    }
}

fn js_to_string(value: &JsValue) -> String {
    match value {
        JsValue::Undefined => "undefined".to_string(),
        JsValue::Null => "null".to_string(),
        JsValue::Bool(flag) => flag.to_string(),
        JsValue::Num(n) => js_num(*n),
        JsValue::Str(text) => text.clone(),
        JsValue::Arr(items) => items
            .iter()
            .map(js_to_string)
            .collect::<Vec<String>>()
            .join(","),
    }
}

fn js_to_primitive(value: &JsValue) -> JsValue {
    match value {
        JsValue::Arr(items) => JsValue::Str(
            items
                .iter()
                .map(js_to_string)
                .collect::<Vec<String>>()
                .join(","),
        ),
        other => other.clone(),
    }
}

fn num_eq(a: f64, b: f64) -> bool {
    a == b
}

fn js_loose_eq(a: &JsValue, b: &JsValue) -> bool {
    match (a, b) {
        (JsValue::Undefined, JsValue::Undefined)
        | (JsValue::Null, JsValue::Null)
        | (JsValue::Undefined, JsValue::Null)
        | (JsValue::Null, JsValue::Undefined) => true,
        (JsValue::Undefined, _)
        | (JsValue::Null, _)
        | (_, JsValue::Undefined)
        | (_, JsValue::Null) => false,
        (JsValue::Bool(_), _) | (_, JsValue::Bool(_)) => {
            num_eq(js_to_number(a), js_to_number(b))
        }
        (JsValue::Num(x), JsValue::Num(y)) => num_eq(*x, *y),
        (JsValue::Str(x), JsValue::Str(y)) => x == y,
        (JsValue::Num(_), JsValue::Str(_)) | (JsValue::Str(_), JsValue::Num(_)) => {
            num_eq(js_to_number(a), js_to_number(b))
        }
        (JsValue::Arr(_), JsValue::Arr(_)) => false,
        (JsValue::Arr(_), _) => js_loose_eq(&js_to_primitive(a), b),
        (_, JsValue::Arr(_)) => js_loose_eq(a, &js_to_primitive(b)),
    }
}

fn js_strict_eq(a: &JsValue, b: &JsValue) -> bool {
    match (a, b) {
        (JsValue::Undefined, JsValue::Undefined) => true,
        (JsValue::Null, JsValue::Null) => true,
        (JsValue::Bool(x), JsValue::Bool(y)) => x == y,
        (JsValue::Num(x), JsValue::Num(y)) => x == y,
        (JsValue::Str(x), JsValue::Str(y)) => x == y,
        _ => false,
    }
}

fn js_less(a: &JsValue, b: &JsValue) -> bool {
    if let (JsValue::Str(x), JsValue::Str(y)) = (a, b) {
        return x < y;
    }
    js_to_number(a) < js_to_number(b)
}

fn js_less_eq(a: &JsValue, b: &JsValue) -> bool {
    if let (JsValue::Str(x), JsValue::Str(y)) = (a, b) {
        return x <= y;
    }
    js_to_number(a) <= js_to_number(b)
}

fn js_add(a: &JsValue, b: &JsValue) -> JsValue {
    let left = js_to_primitive(a);
    let right = js_to_primitive(b);
    if matches!(left, JsValue::Str(_)) || matches!(right, JsValue::Str(_)) {
        return JsValue::Str(js_to_string(&left) + &js_to_string(&right));
    }
    JsValue::Num(js_to_number(&left) + js_to_number(&right))
}

fn is_ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_' || c == '$'
}

fn is_ident_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_' || c == '$'
}

fn keyword_value(ident: &str) -> Option<JsValue> {
    match ident {
        "true" => Some(JsValue::Bool(true)),
        "false" => Some(JsValue::Bool(false)),
        "null" => Some(JsValue::Null),
        "undefined" => Some(JsValue::Undefined),
        "NaN" => Some(JsValue::Num(f64::NAN)),
        "Infinity" => Some(JsValue::Num(f64::INFINITY)),
        _ => None,
    }
}

fn index_value(base: &JsValue, index: &JsValue) -> JsValue {
    let position = js_to_number(index);
    if !position.is_finite() || position.fract() != 0.0 || position < 0.0 {
        return JsValue::Undefined;
    }
    let position = position as usize;
    match base {
        JsValue::Arr(items) => items
            .get(position)
            .cloned()
            .unwrap_or(JsValue::Undefined),
        JsValue::Str(text) => text
            .chars()
            .nth(position)
            .map(|c| JsValue::Str(c.to_string()))
            .unwrap_or(JsValue::Undefined),
        _ => JsValue::Undefined,
    }
}

struct ExprParser {
    src: Vec<char>,
    pos: usize,
}

impl ExprParser {
    fn skip_ws(&mut self) {
        while self.pos < self.src.len() && self.src[self.pos].is_whitespace() {
            self.pos += 1;
        }
    }

    fn peek(&self) -> Option<char> {
        self.src.get(self.pos).copied()
    }

    fn peek_at(&self, offset: usize) -> Option<char> {
        self.src.get(self.pos + offset).copied()
    }

    fn parse_expression(&mut self) -> Option<JsValue> {
        self.parse_or()
    }

    fn parse_or(&mut self) -> Option<JsValue> {
        let mut left = self.parse_and()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('|') && self.peek_at(1) == Some('|') {
                self.pos += 2;
                let right = self.parse_and()?;
                if !js_truthy_value(&left) {
                    left = right;
                }
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_and(&mut self) -> Option<JsValue> {
        let mut left = self.parse_equality()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('&') && self.peek_at(1) == Some('&') {
                self.pos += 2;
                let right = self.parse_equality()?;
                if js_truthy_value(&left) {
                    left = right;
                }
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_equality(&mut self) -> Option<JsValue> {
        let mut left = self.parse_relational()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('=')
                && self.peek_at(1) == Some('=')
                && self.peek_at(2) == Some('=')
            {
                self.pos += 3;
                let right = self.parse_relational()?;
                left = JsValue::Bool(js_strict_eq(&left, &right));
            } else if self.peek() == Some('!')
                && self.peek_at(1) == Some('=')
                && self.peek_at(2) == Some('=')
            {
                self.pos += 3;
                let right = self.parse_relational()?;
                left = JsValue::Bool(!js_strict_eq(&left, &right));
            } else if self.peek() == Some('=') && self.peek_at(1) == Some('=') {
                self.pos += 2;
                let right = self.parse_relational()?;
                left = JsValue::Bool(js_loose_eq(&left, &right));
            } else if self.peek() == Some('!') && self.peek_at(1) == Some('=') {
                self.pos += 2;
                let right = self.parse_relational()?;
                left = JsValue::Bool(!js_loose_eq(&left, &right));
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_relational(&mut self) -> Option<JsValue> {
        let mut left = self.parse_additive()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('<') && self.peek_at(1) == Some('=') {
                self.pos += 2;
                let right = self.parse_additive()?;
                left = JsValue::Bool(js_less_eq(&left, &right));
            } else if self.peek() == Some('>') && self.peek_at(1) == Some('=') {
                self.pos += 2;
                let right = self.parse_additive()?;
                left = JsValue::Bool(js_less_eq(&right, &left));
            } else if self.peek() == Some('<') {
                self.pos += 1;
                let right = self.parse_additive()?;
                left = JsValue::Bool(js_less(&left, &right));
            } else if self.peek() == Some('>') {
                self.pos += 1;
                let right = self.parse_additive()?;
                left = JsValue::Bool(js_less(&right, &left));
            } else {
                break;
            }
        }
        Some(left)
    }

    fn parse_additive(&mut self) -> Option<JsValue> {
        let mut left = self.parse_multiplicative()?;
        loop {
            self.skip_ws();
            match self.peek() {
                Some('+') => {
                    self.pos += 1;
                    let right = self.parse_multiplicative()?;
                    left = js_add(&left, &right);
                }
                Some('-') => {
                    self.pos += 1;
                    let right = self.parse_multiplicative()?;
                    left = JsValue::Num(js_to_number(&left) - js_to_number(&right));
                }
                _ => break,
            }
        }
        Some(left)
    }

    fn parse_multiplicative(&mut self) -> Option<JsValue> {
        let mut left = self.parse_unary()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('*') && self.peek_at(1) == Some('*') {
                self.pos += 2;
                let right = self.parse_unary()?;
                left = JsValue::Num(js_to_number(&left).powf(js_to_number(&right)));
                continue;
            }
            match self.peek() {
                Some('*') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left = JsValue::Num(js_to_number(&left) * js_to_number(&right));
                }
                Some('/') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left = JsValue::Num(js_to_number(&left) / js_to_number(&right));
                }
                Some('%') => {
                    self.pos += 1;
                    let right = self.parse_unary()?;
                    left = JsValue::Num(js_to_number(&left) % js_to_number(&right));
                }
                _ => break,
            }
        }
        Some(left)
    }

    fn parse_unary(&mut self) -> Option<JsValue> {
        self.skip_ws();
        match self.peek() {
            Some('+') => {
                self.pos += 1;
                let value = self.parse_unary()?;
                Some(JsValue::Num(js_to_number(&value)))
            }
            Some('-') => {
                self.pos += 1;
                let value = self.parse_unary()?;
                Some(JsValue::Num(-js_to_number(&value)))
            }
            Some('!') if self.peek_at(1) != Some('=') => {
                self.pos += 1;
                let value = self.parse_unary()?;
                Some(JsValue::Bool(!js_truthy_value(&value)))
            }
            _ => self.parse_postfix(),
        }
    }

    fn parse_postfix(&mut self) -> Option<JsValue> {
        let mut value = self.parse_primary()?;
        loop {
            self.skip_ws();
            if self.peek() == Some('[') {
                self.pos += 1;
                let index = self.parse_or()?;
                self.skip_ws();
                if self.peek() != Some(']') {
                    return None;
                }
                self.pos += 1;
                value = index_value(&value, &index);
            } else {
                break;
            }
        }
        Some(value)
    }

    fn parse_primary(&mut self) -> Option<JsValue> {
        self.skip_ws();
        let current = self.peek()?;
        if current == '(' {
            self.pos += 1;
            let value = self.parse_or()?;
            self.skip_ws();
            if self.peek() != Some(')') {
                return None;
            }
            self.pos += 1;
            return Some(value);
        }
        if current == '[' {
            self.pos += 1;
            let mut items = Vec::new();
            self.skip_ws();
            if self.peek() == Some(']') {
                self.pos += 1;
                return Some(JsValue::Arr(items));
            }
            loop {
                let item = self.parse_or()?;
                items.push(item);
                self.skip_ws();
                match self.peek() {
                    Some(',') => {
                        self.pos += 1;
                    }
                    Some(']') => {
                        self.pos += 1;
                        break;
                    }
                    _ => return None,
                }
            }
            return Some(JsValue::Arr(items));
        }
        if current == '"' || current == '\'' {
            return self.parse_string(current);
        }
        if current.is_ascii_digit()
            || (current == '.' && self.peek_at(1).map_or(false, |c| c.is_ascii_digit()))
        {
            return self.parse_number();
        }
        if is_ident_start(current) {
            let ident = self.parse_ident();
            return keyword_value(&ident);
        }
        None
    }

    fn parse_string(&mut self, quote: char) -> Option<JsValue> {
        self.pos += 1;
        let mut text = String::new();
        loop {
            let current = self.peek()?;
            if current == quote {
                self.pos += 1;
                return Some(JsValue::Str(text));
            }
            if current == '\\' {
                self.pos += 1;
                let escape = self.peek()?;
                self.pos += 1;
                match escape {
                    'n' => text.push('\n'),
                    't' => text.push('\t'),
                    'r' => text.push('\r'),
                    'b' => text.push('\u{08}'),
                    'f' => text.push('\u{0c}'),
                    'v' => text.push('\u{0b}'),
                    '0' => text.push('\0'),
                    '\\' => text.push('\\'),
                    '\'' => text.push('\''),
                    '"' => text.push('"'),
                    '\n' => {}
                    'u' => text.push(self.parse_unicode_escape()?),
                    'x' => text.push(self.parse_hex_escape(2)?),
                    other => text.push(other),
                }
            } else {
                text.push(current);
                self.pos += 1;
            }
        }
    }

    fn parse_unicode_escape(&mut self) -> Option<char> {
        if self.peek() == Some('{') {
            self.pos += 1;
            let mut code = 0u32;
            let mut digits = 0usize;
            while let Some(c) = self.peek() {
                if c == '}' {
                    self.pos += 1;
                    break;
                }
                code = code * 16 + c.to_digit(16)?;
                digits += 1;
                self.pos += 1;
            }
            if digits == 0 {
                return None;
            }
            char::from_u32(code)
        } else {
            self.parse_hex_escape(4)
        }
    }

    fn parse_hex_escape(&mut self, len: usize) -> Option<char> {
        let mut code = 0u32;
        for _ in 0..len {
            let c = self.peek()?;
            if !c.is_ascii_hexdigit() {
                return None;
            }
            code = code * 16 + c.to_digit(16)?;
            self.pos += 1;
        }
        char::from_u32(code)
    }

    fn read_digits(&mut self, radix: u32) -> (String, bool) {
        let mut digits = String::new();
        let mut saw_separator = false;
        loop {
            match self.peek() {
                Some('_') if !digits.is_empty() && !saw_separator => {
                    self.pos += 1;
                    saw_separator = true;
                }
                Some(c) if c.to_digit(radix).is_some() => {
                    digits.push(c);
                    self.pos += 1;
                    saw_separator = false;
                }
                _ => break,
            }
        }
        let valid = !saw_separator;
        (digits, valid)
    }

    fn parse_number(&mut self) -> Option<JsValue> {
        if self.peek() == Some('0') {
            if let Some(prefix) = self.peek_at(1) {
                let radix = match prefix {
                    'x' | 'X' => Some(16u32),
                    'b' | 'B' => Some(2u32),
                    'o' | 'O' => Some(8u32),
                    _ => None,
                };
                if let Some(radix) = radix {
                    self.pos += 2;
                    let (digits, valid) = self.read_digits(radix);
                    if !valid || digits.is_empty() {
                        return None;
                    }
                    let mut value = 0.0f64;
                    for c in digits.chars() {
                        value = value * radix as f64 + c.to_digit(radix)? as f64;
                    }
                    return Some(JsValue::Num(value));
                }
                if prefix.is_ascii_digit() || prefix == '_' {
                    return None;
                }
            }
        }
        let (int_digits, int_valid) = self.read_digits(10);
        if !int_valid {
            return None;
        }
        let mut has_digit = !int_digits.is_empty();
        let mut text = int_digits;
        if self.peek() == Some('.') {
            self.pos += 1;
            text.push('.');
            let (frac_digits, frac_valid) = self.read_digits(10);
            if !frac_valid {
                return None;
            }
            if !frac_digits.is_empty() {
                has_digit = true;
            }
            text.push_str(&frac_digits);
        }
        if !has_digit {
            return None;
        }
        if matches!(self.peek(), Some('e') | Some('E')) {
            let save = self.pos;
            self.pos += 1;
            let mut exponent = String::new();
            if matches!(self.peek(), Some('+') | Some('-')) {
                exponent.push(self.peek().unwrap());
                self.pos += 1;
            }
            let (exp_digits, exp_valid) = self.read_digits(10);
            if exp_valid && !exp_digits.is_empty() {
                exponent.push_str(&exp_digits);
                text.push('e');
                text.push_str(&exponent);
            } else {
                self.pos = save;
            }
        }
        text.parse::<f64>().ok().map(JsValue::Num)
    }

    fn parse_ident(&mut self) -> String {
        let start = self.pos;
        while self.peek().map_or(false, is_ident_char) {
            self.pos += 1;
        }
        self.src[start..self.pos].iter().collect()
    }
}

fn parse_expression(src: &str) -> Option<JsValue> {
    let mut parser = ExprParser {
        src: src.chars().collect(),
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
            ("l", VarValue::List(vec![VarValue::Num(1.0), VarValue::Num(2.5)])),
        ]);
        assert_eq!(replace_vars("f", &vars), "0.5");
        assert_eq!(replace_vars("s == green", &vars), "\"green\" == green");
        assert_eq!(replace_vars("l", &vars), "[1,2.5]");
    }

    #[test]
    fn replace_non_finite_becomes_null() {
        let vars = vars_of(&[
            ("n", VarValue::Num(f64::NAN)),
            ("i", VarValue::Num(f64::INFINITY)),
            (
                "l",
                VarValue::List(vec![
                    VarValue::Num(f64::NAN),
                    VarValue::Num(f64::INFINITY),
                ]),
            ),
        ]);
        assert_eq!(replace_vars("n + i", &vars), "null + null");
        assert_eq!(replace_vars("l", &vars), "[null,null]");
    }

    #[test]
    fn replace_escapes_strings() {
        let vars = vars_of(&[("s", VarValue::Str("a\"b\\c\n".to_string()))]);
        assert_eq!(replace_vars("s", &vars), "\"a\\\"b\\\\c\\n\"");
    }

    #[test]
    fn eval_string_comparisons() {
        let vars = vars_of(&[("flag", VarValue::Str("yes".to_string()))]);
        assert!(approx(eval_expr("flag == \"yes\"", &vars), 1.0));
        assert!(approx(eval_expr("flag != \"no\"", &vars), 1.0));
        assert!(approx(eval_expr("flag == \"no\"", &vars), 0.0));
        assert!(eval_truthy("flag", &vars));
        assert!(!eval_truthy("missing", &vars));
    }

    #[test]
    fn eval_literals_and_coercion() {
        assert!(approx(eval_expr("true", &BTreeMap::new()), 1.0));
        assert!(approx(eval_expr("false", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("null", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("\"5\" == 5", &BTreeMap::new()), 1.0));
        assert!(approx(eval_expr("\"5\" === 5", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("0x10", &BTreeMap::new()), 16.0));
        assert!(approx(eval_expr("2 ** 3", &BTreeMap::new()), 8.0));
        assert!(approx(eval_expr("\"a\" + \"b\" == \"ab\"", &BTreeMap::new()), 1.0));
        assert!(approx(eval_expr("[1,2][1]", &BTreeMap::new()), 2.0));
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
        list_ensure(&mut state, "items");
        list_push(&mut state, "items", VarValue::Num(5.0));
        list_push(&mut state, "items", VarValue::Num(7.0));
        assert_eq!(list_get(&state, "items", 1.0), VarValue::Num(7.0));
        assert_eq!(list_get(&state, "items", 2.0), VarValue::Num(0.0));
        assert_eq!(list_get(&state, "items", -1.0), VarValue::Num(0.0));
        state.vars.insert("n".to_string(), VarValue::Num(1.0));
        assert_eq!(list_get(&state, "n", 0.0), VarValue::Num(0.0));
    }

    #[test]
    fn list_helpers_preserve_strings() {
        let mut state = RuntimeState::new();
        list_ensure(&mut state, "items");
        list_push(&mut state, "items", VarValue::Str("a".to_string()));
        assert_eq!(list_get(&state, "items", 0.0), VarValue::Str("a".to_string()));
        list_push(&mut state, "items", VarValue::Num(f64::NAN));
        assert_eq!(list_get(&state, "items", 1.0), VarValue::Num(0.0));
        assert_eq!(list_get(&state, "items", f64::NAN), VarValue::Num(0.0));
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
        let three = VarValue::Num(3.0);
        assert_eq!(
            compound_set(&VarValue::Num(2.0), "+=", &three),
            VarValue::Num(5.0)
        );
        assert_eq!(
            compound_set(
                &VarValue::Str("a".to_string()),
                "+=",
                &VarValue::Num(0.5)
            ),
            VarValue::Str("a0.5".to_string())
        );
        assert_eq!(
            compound_set(&VarValue::Str(String::new()), "+=", &three),
            VarValue::Num(3.0)
        );
        assert_eq!(
            compound_set(
                &VarValue::List(vec![VarValue::Num(1.0), VarValue::Num(2.0)]),
                "+=",
                &three
            ),
            VarValue::Str("1,23".to_string())
        );
        assert_eq!(
            compound_set(&VarValue::Num(2.0), "-=", &VarValue::Num(5.0)),
            VarValue::Num(-3.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(4.0), "*=", &VarValue::Num(2.5)),
            VarValue::Num(10.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(8.0), "/=", &VarValue::Num(0.0)),
            VarValue::Num(0.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(8.0), "/=", &VarValue::Num(4.0)),
            VarValue::Num(2.0)
        );
        assert_eq!(
            compound_set(&VarValue::Str("12.5".to_string()), "*=", &VarValue::Num(2.0)),
            VarValue::Num(25.0)
        );
        assert_eq!(
            compound_set(&VarValue::Num(1.0), "=", &VarValue::Str("green".to_string())),
            VarValue::Str("green".to_string())
        );
        assert_eq!(
            compound_set(&VarValue::Num(1.0), "??", &VarValue::Num(9.0)),
            VarValue::Num(1.0)
        );
    }

    #[test]
    fn coerce_num_matches_js_number() {
        assert!(approx(coerce_num(&VarValue::Num(2.5)), 2.5));
        assert!(approx(coerce_num(&VarValue::Str("12.5".to_string())), 12.5));
        assert!(approx(coerce_num(&VarValue::Str(String::new())), 0.0));
        assert!(coerce_num(&VarValue::Str("abc".to_string())).is_nan());
        assert!(approx(coerce_num(&VarValue::List(vec![VarValue::Num(5.0)])), 5.0));
        assert!(coerce_num(&VarValue::List(vec![VarValue::Num(1.0), VarValue::Num(2.0)])).is_nan());
    }

    #[test]
    fn eval_var_preserves_types() {
        assert_eq!(
            eval_var("\"green\"", &BTreeMap::new()),
            VarValue::Str("green".to_string())
        );
        assert_eq!(eval_var("2 + 3", &BTreeMap::new()), VarValue::Num(5.0));
        assert_eq!(eval_var("bogus", &BTreeMap::new()), VarValue::Num(0.0));
        assert_eq!(
            eval_var("[1, 2]", &BTreeMap::new()),
            VarValue::List(vec![VarValue::Num(1.0), VarValue::Num(2.0)])
        );
        assert!(eval_truthy("\"green\"", &BTreeMap::new()));
        assert!(matches!(
            eval_var("null", &BTreeMap::new()),
            VarValue::Num(n) if n.is_nan()
        ));
    }

    #[test]
    fn js_number_accepts_radix_prefixes() {
        assert!(approx(coerce_num(&VarValue::Str("0x10".to_string())), 16.0));
        assert!(approx(coerce_num(&VarValue::Str("0X10".to_string())), 16.0));
        assert!(approx(coerce_num(&VarValue::Str("0b101".to_string())), 5.0));
        assert!(approx(coerce_num(&VarValue::Str("0o17".to_string())), 15.0));
        assert!(approx(coerce_num(&VarValue::Str(" 0x1F ".to_string())), 31.0));
        assert!(approx(coerce_num(&VarValue::Str("010".to_string())), 10.0));
        assert_eq!(
            coerce_num(&VarValue::Str("Infinity".to_string())),
            f64::INFINITY
        );
        assert!(coerce_num(&VarValue::Str("inf".to_string())).is_nan());
        assert!(coerce_num(&VarValue::Str("infinity".to_string())).is_nan());
        assert!(coerce_num(&VarValue::Str("-0x10".to_string())).is_nan());
        assert!(coerce_num(&VarValue::Str("1_000".to_string())).is_nan());
    }

    #[test]
    fn boolean_values_stay_booleans() {
        assert_eq!(eval_var("true", &BTreeMap::new()), VarValue::Bool(true));
        assert_eq!(eval_var("false", &BTreeMap::new()), VarValue::Bool(false));
        assert_eq!(stringify_value(&VarValue::Bool(true)), "true");
        assert_eq!(stringify_value(&VarValue::Bool(false)), "false");
        assert!(approx(var_to_number(&VarValue::Bool(true)), 1.0));
        assert!(approx(var_to_number(&VarValue::Bool(false)), 0.0));
        assert!(var_truthy(&VarValue::Bool(true)));
        assert!(!var_truthy(&VarValue::Bool(false)));
        assert_eq!(
            compound_set(&VarValue::Bool(true), "+=", &VarValue::Str("x".to_string())),
            VarValue::Str("truex".to_string())
        );
        let vars = vars_of(&[("s", VarValue::Bool(true))]);
        assert!(approx(eval_expr("s === true", &vars), 1.0));
        assert!(approx(eval_expr("s == 1", &vars), 1.0));
    }

    #[test]
    fn numeric_literals_match_strict_mode() {
        assert!(approx(eval_expr("010", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("00", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("08", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("0_1", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("1_000", &BTreeMap::new()), 1000.0));
        assert!(approx(eval_expr("1_000.5", &BTreeMap::new()), 1000.5));
        assert!(approx(eval_expr("1_000e2", &BTreeMap::new()), 100000.0));
        assert!(approx(eval_expr("0b1_0", &BTreeMap::new()), 2.0));
        assert!(approx(eval_expr("0o1_7", &BTreeMap::new()), 15.0));
        assert!(approx(eval_expr("0xFF", &BTreeMap::new()), 255.0));
        assert!(approx(eval_expr("1__0", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("0", &BTreeMap::new()), 0.0));
        assert!(approx(eval_expr("0.5", &BTreeMap::new()), 0.5));
        assert!(approx(eval_expr("0e1", &BTreeMap::new()), 0.0));
    }

    #[test]
    fn replace_vars_interprets_replacement_patterns() {
        let vars = vars_of(&[("s", VarValue::Str("$&".to_string()))]);
        assert_eq!(replace_vars("s == \"$&\"", &vars), "\"s\" == \"$&\"");
        let vars = vars_of(&[("s", VarValue::Str("$$".to_string()))]);
        assert_eq!(replace_vars("s == \"$$\"", &vars), "\"$\" == \"$$\"");
        let vars = vars_of(&[("s", VarValue::Str("$`".to_string()))]);
        assert_eq!(replace_vars("s == \"$`\"", &vars), "\"\" == \"$`\"");
    }

    #[test]
    fn replace_vars_treats_name_as_regex() {
        let vars = vars_of(&[("a.b", VarValue::Num(5.0)), ("axb", VarValue::Num(9.0))]);
        assert_eq!(replace_vars("axb == 5", &vars), "5 == 5");
    }
}
