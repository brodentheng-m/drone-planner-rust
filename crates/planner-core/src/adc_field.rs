use crate::obstacles::{type_dims, Boundary, Obstacle};
use crate::planio::ObstacleFile;

const IN: f64 = 0.0254;
const FT: f64 = 0.3048;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Confidence {
    Verified,
    OcrUncertain,
    Unreadable,
}

#[derive(Debug, Clone, Copy)]
pub struct Dimension {
    pub name: &'static str,
    pub value_m: f64,
    pub source_page: u32,
    pub confidence: Confidence,
}

static DIMENSIONS: &[Dimension] = &[
    Dimension {
        name: "field_width",
        value_m: 20.0 * FT,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "field_depth",
        value_m: 20.0 * FT,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_top_stub",
        value_m: 19.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_top_gap_1",
        value_m: 60.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_top_gap_2",
        value_m: 42.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_top_gap_3",
        value_m: 65.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_left_inset",
        value_m: 42.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_mid_inset",
        value_m: 61.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_bottom_inset",
        value_m: 26.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_bottom_gap_1",
        value_m: 58.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_bottom_gap_2",
        value_m: 48.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_bottom_gap_3",
        value_m: 51.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "track_bottom_gap_4",
        value_m: 52.0 * IN,
        source_page: 40,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "arch_tension_wire",
        value_m: 102.0 * IN,
        source_page: 41,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "arch_track_mark",
        value_m: 1.0 * FT,
        source_page: 41,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "pillar_mark_near",
        value_m: 2.0 * FT,
        source_page: 43,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "pillar_mark_far",
        value_m: 5.0 * FT,
        source_page: 43,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "cube_corner_offset",
        value_m: 3.0 * IN,
        source_page: 44,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_keyhole_height",
        value_m: 64.0 * IN,
        source_page: 45,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "landing_pad_gap_min",
        value_m: 1.0 * IN,
        source_page: 46,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "landing_pad_gap_max",
        value_m: 2.0 * IN,
        source_page: 46,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_offset_82",
        value_m: 82.0 * IN,
        source_page: 47,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_offset_43",
        value_m: 43.0 * IN,
        source_page: 47,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_offset_38",
        value_m: 38.0 * IN,
        source_page: 47,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_offset_29",
        value_m: 29.0 * IN,
        source_page: 47,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_offset_28",
        value_m: 28.0 * IN,
        source_page: 47,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_track_overlap",
        value_m: 5.0 * IN,
        source_page: 50,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "loading_station_ball_count",
        value_m: 20.0,
        source_page: 51,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_length",
        value_m: 24.0 * IN,
        source_page: 37,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_thickness",
        value_m: 0.125 * IN,
        source_page: 37,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_flat_width",
        value_m: 7.5 * IN,
        source_page: 37,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_flap_1",
        value_m: 1.75 * IN,
        source_page: 37,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_flap_2",
        value_m: 2.0 * IN,
        source_page: 37,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_height",
        value_m: 2.285 * IN,
        source_page: 38,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "goal_segment_base",
        value_m: 1.674 * IN,
        source_page: 38,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "switch_plate_length",
        value_m: 20.079 * IN,
        source_page: 36,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "switch_plate_width",
        value_m: 9.055 * IN,
        source_page: 36,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "switch_plate_notch",
        value_m: 8.071 * IN,
        source_page: 36,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "switch_plate_span",
        value_m: 4.941 * IN,
        source_page: 36,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "switch_plate_tab",
        value_m: 1.378 * IN,
        source_page: 36,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "low_switch_plate_length",
        value_m: 20.079 * IN,
        source_page: 23,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "low_switch_plate_width",
        value_m: 9.055 * IN,
        source_page: 23,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_pvc_span",
        value_m: 0.15,
        source_page: 34,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "high_switch_pvc_length",
        value_m: 0.18,
        source_page: 34,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "low_switch_pvc_span",
        value_m: 5.126 * IN,
        source_page: 21,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "low_switch_pvc_length",
        value_m: 11.0 * IN,
        source_page: 21,
        confidence: Confidence::Verified,
    },
    Dimension {
        name: "cube_size",
        value_m: 0.0,
        source_page: 44,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "bean_bag_size",
        value_m: 0.0,
        source_page: 48,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "block_size",
        value_m: 0.0,
        source_page: 49,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "loading_station_size",
        value_m: 0.0,
        source_page: 51,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "keyhole_gate_diameter",
        value_m: 0.0,
        source_page: 45,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "track_segment_width",
        value_m: 0.0,
        source_page: 40,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "high_switch_pole_height",
        value_m: 0.0,
        source_page: 47,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "low_switch_pole_height",
        value_m: 0.0,
        source_page: 42,
        confidence: Confidence::Unreadable,
    },
    Dimension {
        name: "arch_gate_size",
        value_m: 0.0,
        source_page: 41,
        confidence: Confidence::Unreadable,
    },
];

pub fn dimensions() -> &'static [Dimension] {
    DIMENSIONS
}

pub fn uncertain_dimensions() -> Vec<&'static str> {
    DIMENSIONS
        .iter()
        .filter(|dimension| dimension.confidence != Confidence::Verified)
        .map(|dimension| dimension.name)
        .collect()
}

fn dim(name: &str) -> f64 {
    DIMENSIONS
        .iter()
        .find(|dimension| dimension.name == name)
        .map(|dimension| dimension.value_m)
        .unwrap_or(0.0)
}

pub fn field_bounds() -> Boundary {
    let hoop = type_dims("hoop").unwrap();
    Boundary {
        min_x: 0.0,
        max_x: dim("field_width"),
        min_z: 0.0,
        max_z: dim("field_depth"),
        max_y: dim("high_switch_keyhole_height") + hoop.outer_radius,
    }
}

fn obstacle(
    id: &str,
    obstacle_type: &str,
    name: &str,
    position: [f64; 3],
    rotation: [f64; 3],
    scale: [f64; 3],
) -> Obstacle {
    Obstacle {
        id: id.to_string(),
        obstacle_type: obstacle_type.to_string(),
        position,
        rotation,
        scale,
        name: name.to_string(),
        color: None,
    }
}

fn track_columns() -> [f64; 4] {
    let first = dim("track_bottom_inset");
    let second = first + dim("track_bottom_gap_1");
    let third = second + dim("track_bottom_gap_2");
    let fourth = third + dim("track_bottom_gap_3");
    [first, second, third, fourth]
}

fn push_perimeter(obstacles: &mut Vec<Obstacle>) {
    let width = dim("field_width");
    let depth = dim("field_depth");
    let dims = type_dims("wall").unwrap();
    let height = dims.height;
    let thickness = 0.05;
    let turn = std::f64::consts::FRAC_PI_2;
    obstacles.push(obstacle(
        "adc_wall_south",
        "wall",
        "South Perimeter Wall",
        [width / 2.0, height / 2.0, 0.0],
        [0.0, 0.0, 0.0],
        [width / dims.width, 1.0, thickness / dims.depth],
    ));
    obstacles.push(obstacle(
        "adc_wall_north",
        "wall",
        "North Perimeter Wall",
        [width / 2.0, height / 2.0, depth],
        [0.0, 0.0, 0.0],
        [width / dims.width, 1.0, thickness / dims.depth],
    ));
    obstacles.push(obstacle(
        "adc_wall_west",
        "wall",
        "West Perimeter Wall",
        [0.0, height / 2.0, depth / 2.0],
        [0.0, turn, 0.0],
        [depth / dims.width, 1.0, thickness / dims.depth],
    ));
    obstacles.push(obstacle(
        "adc_wall_east",
        "wall",
        "East Perimeter Wall",
        [width, height / 2.0, depth / 2.0],
        [0.0, turn, 0.0],
        [depth / dims.width, 1.0, thickness / dims.depth],
    ));
}

fn push_tracks(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let depth = dim("field_depth");
    let run = depth - dim("track_top_stub");
    let dims = type_dims("square").unwrap();
    for (index, column) in columns.iter().enumerate() {
        obstacles.push(obstacle(
            &format!("adc_track_{}", index + 1),
            "square",
            &format!("Track {}", index + 1),
            [*column, 0.0025, run / 2.0],
            [0.0, 0.0, 0.0],
            [0.05 / dims.width, 0.005 / dims.height, run / dims.depth],
        ));
    }
}

fn push_goals(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let end = dim("field_depth") - dim("track_top_stub");
    let side = dim("goal_segment_length");
    let height = dim("goal_segment_height");
    let base = dim("goal_segment_base");
    let dims = type_dims("square").unwrap();
    let radius = side / 3.0_f64.sqrt();
    for (track_index, column) in columns.iter().enumerate() {
        for corner in 0..3 {
            let angle =
                std::f64::consts::FRAC_PI_2 + corner as f64 * std::f64::consts::TAU / 3.0;
            let position = [
                column + radius * angle.cos(),
                height / 2.0,
                end + radius * angle.sin(),
            ];
            obstacles.push(obstacle(
                &format!("adc_goal_{}_{}", track_index + 1, corner + 1),
                "square",
                &format!("Goal Segment {}-{}", track_index + 1, corner + 1),
                position,
                [0.0, angle, 0.0],
                [
                    side / dims.width,
                    height / dims.height,
                    base / dims.depth,
                ],
            ));
        }
    }
}

fn push_low_switches(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let depth = dim("field_depth");
    let length = dim("switch_plate_length");
    let width = dim("switch_plate_width");
    let dims = type_dims("square").unwrap();
    let marks = [depth * 0.40, depth * 0.55];
    let mut count = 0;
    for column in [columns[0], columns[2]] {
        for mark in marks {
            count += 1;
            obstacles.push(obstacle(
                &format!("adc_low_switch_{count}"),
                "square",
                &format!("Low Switch {count}"),
                [column, width / 2.0, mark],
                [0.0, 0.0, 0.0],
                [
                    length / dims.width,
                    width / dims.height,
                    0.02 / dims.depth,
                ],
            ));
        }
    }
}

fn push_pillars(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let dims = type_dims("tower").unwrap();
    let marks = [dim("pillar_mark_near"), dim("pillar_mark_far")];
    for (index, mark) in marks.iter().enumerate() {
        obstacles.push(obstacle(
            &format!("adc_pillar_{}", index + 1),
            "tower",
            &format!("Pillar {}", index + 1),
            [columns[1], dims.height / 2.0, *mark],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ));
    }
}

fn push_high_switches(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let depth = dim("field_depth");
    let tower = type_dims("tower").unwrap();
    let hoop = type_dims("hoop").unwrap();
    let keyhole = dim("high_switch_keyhole_height");
    let mark = depth * 0.72;
    let assemblies = [
        (
            columns[1],
            "adc_high_switch_1",
            "High Switch Assembly 1",
            "adc_keyhole_1",
            "Keyhole Gate 1",
        ),
        (
            columns[2],
            "adc_high_switch_2",
            "High Switch Assembly 2",
            "adc_keyhole_2",
            "Keyhole Gate 2",
        ),
    ];
    for (column, pole_id, pole_name, gate_id, gate_name) in assemblies {
        obstacles.push(obstacle(
            pole_id,
            "tower",
            pole_name,
            [column, tower.height / 2.0, mark],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ));
        obstacles.push(obstacle(
            gate_id,
            "hoop",
            gate_name,
            [column, keyhole + hoop.outer_radius, mark],
            [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ));
    }
}

fn push_arch_gates(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let hoop = type_dims("hoop").unwrap();
    let span = dim("arch_tension_wire");
    let natural = 2.0 * (hoop.outer_radius + hoop.inner_radius);
    let scale = span / natural;
    let mark = dim("field_depth") * 0.12;
    obstacles.push(obstacle(
        "adc_arch_red",
        "hoop",
        "Red Arch Gate",
        [columns[0], 1.2, mark],
        [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
        [scale, scale, scale],
    ));
    obstacles.push(obstacle(
        "adc_arch_blue",
        "hoop",
        "Blue Arch Gate",
        [columns[3], 1.2, mark],
        [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
        [scale, scale, scale],
    ));
}

fn push_loading_stations(obstacles: &mut Vec<Obstacle>) {
    let columns = track_columns();
    let mark = dim("field_depth") - dim("arch_track_mark");
    let dims = type_dims("square").unwrap();
    obstacles.push(obstacle(
        "adc_loading_station_1",
        "square",
        "Loading Station Pad 1",
        [columns[0], 0.01, mark],
        [0.0, 0.0, 0.0],
        [0.6 / dims.width, 0.02 / dims.height, 0.6 / dims.depth],
    ));
    obstacles.push(obstacle(
        "adc_loading_station_2",
        "square",
        "Loading Station Pad 2",
        [columns[3], 0.01, mark],
        [0.0, 0.0, 0.0],
        [0.6 / dims.width, 0.02 / dims.height, 0.6 / dims.depth],
    ));
}

pub fn build_fast_track() -> ObstacleFile {
    let mut obstacles = Vec::new();
    push_perimeter(&mut obstacles);
    push_tracks(&mut obstacles);
    push_goals(&mut obstacles);
    push_low_switches(&mut obstacles);
    push_pillars(&mut obstacles);
    push_high_switches(&mut obstacles);
    push_arch_gates(&mut obstacles);
    push_loading_stations(&mut obstacles);
    ObstacleFile {
        obstacles,
        boundary: Some(field_bounds()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_returns_non_empty_finite_set() {
        let file = build_fast_track();
        assert!(!file.obstacles.is_empty());
        for obstacle in &file.obstacles {
            for value in obstacle
                .position
                .iter()
                .chain(obstacle.rotation.iter())
                .chain(obstacle.scale.iter())
            {
                assert!(value.is_finite(), "non-finite value on {}", obstacle.id);
            }
        }
    }

    #[test]
    fn obstacles_have_identity_and_finite_transforms() {
        for obstacle in build_fast_track().obstacles {
            assert!(!obstacle.id.is_empty());
            assert!(!obstacle.name.is_empty());
            for value in obstacle
                .position
                .iter()
                .chain(obstacle.rotation.iter())
                .chain(obstacle.scale.iter())
            {
                assert!(value.is_finite(), "non-finite value on {}", obstacle.id);
                assert!(!value.is_nan(), "NaN on {}", obstacle.id);
            }
        }
    }

    #[test]
    fn obstacles_within_field_bounds() {
        let file = build_fast_track();
        let bounds = field_bounds();
        for obstacle in &file.obstacles {
            assert!(obstacle.position[0] >= bounds.min_x, "{}", obstacle.id);
            assert!(obstacle.position[0] <= bounds.max_x, "{}", obstacle.id);
            assert!(obstacle.position[2] >= bounds.min_z, "{}", obstacle.id);
            assert!(obstacle.position[2] <= bounds.max_z, "{}", obstacle.id);
            assert!(obstacle.position[1] >= 0.0, "{}", obstacle.id);
            assert!(obstacle.position[1] <= bounds.max_y, "{}", obstacle.id);
        }
    }

    #[test]
    fn geometry_matches_dimension_table() {
        let file = build_fast_track();
        let track_one = file
            .obstacles
            .iter()
            .find(|obstacle| obstacle.name == "Track 1")
            .expect("Track 1");
        assert!((track_one.position[0] - dim("track_bottom_inset")).abs() < 1e-9);
        let track_three = file
            .obstacles
            .iter()
            .find(|obstacle| obstacle.name == "Track 3")
            .expect("Track 3");
        let expected = dim("track_bottom_inset")
            + dim("track_bottom_gap_1")
            + dim("track_bottom_gap_2");
        assert!((track_three.position[0] - expected).abs() < 1e-9);
        let south = file
            .obstacles
            .iter()
            .find(|obstacle| obstacle.name == "South Perimeter Wall")
            .expect("South Perimeter Wall");
        let wall = type_dims("wall").unwrap();
        let span = wall.width * south.scale[0];
        assert!((span - dim("field_width")).abs() < 1e-6);
    }

    #[test]
    fn every_dimension_page_is_in_range() {
        for dimension in dimensions() {
            assert!(dimension.source_page > 0, "{}", dimension.name);
            assert!(dimension.source_page <= 51, "{}", dimension.name);
        }
    }

    #[test]
    fn uncertain_dimensions_are_the_unverified_entries() {
        let names = uncertain_dimensions();
        for dimension in dimensions() {
            if dimension.confidence == Confidence::Verified {
                assert!(!names.contains(&dimension.name));
            } else {
                assert!(names.contains(&dimension.name));
            }
        }
    }
}
