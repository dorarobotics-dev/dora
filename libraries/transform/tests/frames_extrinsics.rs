use dora_transform::{Extrinsics, FrameDict, FrameError, Kind};

const FRAMES: &str = include_str!("fixtures/frames.yml");
const EXTRINSICS: &str = include_str!("fixtures/extrinsics.yml");

fn frames() -> FrameDict {
    FrameDict::parse(FRAMES).unwrap()
}
fn extrinsics() -> Extrinsics {
    Extrinsics::parse(EXTRINSICS, &frames(), None).unwrap()
}

#[test]
fn reference_dictionary_loads() {
    let f = frames();
    assert_eq!(f.require("rslidar").unwrap().kind, Kind::Mounted);
    assert_eq!(
        f.chain("rslidar").unwrap(),
        vec!["rslidar", "base_link", "base_footprint", "odom", "map"]
    );
}

#[test]
fn rejected_name_says_what_to_use() {
    let e = frames().require("lidar_link").unwrap_err();
    assert!(matches!(e, FrameError::UnknownFrame(_)));
    assert!(e.to_string().contains("'rslidar'"), "{e}");
}

#[test]
fn unknown_frame_is_refused() {
    assert!(matches!(
        frames().require("nowhere"),
        Err(FrameError::UnknownFrame(_))
    ));
}

#[test]
fn dictionary_shape_errors() {
    for bad in [
        "frames: [{name: a, kind: fixed}, {name: a, kind: fixed}]",
        "frames: [{name: a, kind: spinning}]",
        "frames: [{name: a, parent: b, kind: mounted}]",
        "frames: [{name: a, kind: mounted}]",
        "frames: [{name: a, parent: a, kind: fixed}]",
        "frames: [{name: [a], kind: fixed}]",
        "frames: [{name: a, kind: fixed}]\nrejected_names: {a: a}",
        "frames: [{name: a, kind: fixed}]\nrejected_names: {b: c}",
        "nodes: []",
    ] {
        assert!(
            matches!(FrameDict::parse(bad), Err(FrameError::Invalid(_))),
            "accepted or wrong variant: {bad}"
        );
    }
}

#[test]
fn cycle_is_refused() {
    let doc = "frames: [{name: a, parent: b, kind: dynamic}, {name: b, parent: a, kind: dynamic}]";
    assert!(
        FrameDict::parse(doc)
            .unwrap_err()
            .to_string()
            .contains("cycle")
    );
}

#[test]
fn static_route_up_and_down() {
    let r = extrinsics().route("rslidar", "camera_front").unwrap();
    assert_eq!(
        r.up.iter().map(|e| e.child.as_str()).collect::<Vec<_>>(),
        vec!["rslidar"]
    );
    assert_eq!(
        r.down.iter().map(|e| e.child.as_str()).collect::<Vec<_>>(),
        vec!["camera_front"]
    );
}

#[test]
fn route_through_one_dynamic_edge_names_it() {
    match extrinsics().route("rslidar", "odom") {
        Err(FrameError::DynamicEdge { edges, .. }) => {
            assert_eq!(
                edges,
                vec![("base_footprint".to_string(), "odom".to_string())]
            );
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn route_through_two_dynamic_edges_lists_both() {
    match extrinsics().route("rslidar", "map") {
        Err(FrameError::DynamicEdge { edges, .. }) => assert_eq!(edges.len(), 2),
        other => panic!("{other:?}"),
    }
}

#[test]
fn missing_extrinsic_wins_over_dynamic_edge() {
    let doc = EXTRINSICS.replace(
        "  - parent: base_footprint\n    child: base_link\n    translation_m: [0.0, 0.0, 0.1]\n    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]\n", "");
    let e = Extrinsics::parse(&doc, &frames(), None).unwrap();
    let err = e.route("rslidar", "odom").unwrap_err();
    assert!(matches!(err, FrameError::MissingExtrinsic(_)), "{err:?}");
    assert!(err.to_string().contains("never assumed to be identity"));
}

#[test]
fn none_frame_is_never_routed() {
    assert!(matches!(
        extrinsics().route("none", "map"),
        Err(FrameError::UnknownFrame(_))
    ));
}

#[test]
fn calibration_shape_errors() {
    let f = frames();
    let edge = |body: &str| {
        format!(
            "contract_version: 1.0.0\nrobot: r\ncalibrated: \"2026-10-20\"\nsource: s\ntransforms:\n{body}"
        )
    };
    for bad in [
        edge(
            "  - {parent: odom, child: base_footprint, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}",
        ),
        edge(
            "  - {parent: map, child: base_link, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}",
        ),
        edge(
            "  - {parent: base_link, child: rslidar, translation_m: [0,0], rotation_xyzw: [0,0,0,1]}",
        ),
        edge(
            "  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,2]}",
        ),
        edge(
            "  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}\n  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}",
        ),
        edge(
            "  - {parent: base_link, child: rslidar, translation_m: [0,0,.nan], rotation_xyzw: [0,0,0,1]}",
        ),
        edge(
            "  - {parent: base_link, child: rslidar, translation_m: [0,0,true], rotation_xyzw: [0,0,0,1]}",
        ),
        "robot: r\ntransforms: []".to_string(),
    ] {
        assert!(
            Extrinsics::parse(&bad, &f, None).is_err(),
            "accepted: {bad}"
        );
    }
}

fn temp_file(name: &str, body: Option<&str>) -> std::path::PathBuf {
    let path = std::env::temp_dir().join(format!("dora-transform-{}-{name}", std::process::id()));
    match body {
        Some(b) => std::fs::write(&path, b).unwrap(),
        None => {
            let _ = std::fs::remove_file(&path);
        }
    }
    path
}

fn edge_names(edges: &[dora_transform::Edge]) -> Vec<&str> {
    edges.iter().map(|e| e.child.as_str()).collect()
}

#[test]
fn route_to_self_is_empty() {
    let r = extrinsics().route("rslidar", "rslidar").unwrap();
    assert!(r.up.is_empty() && r.down.is_empty());
}

#[test]
fn route_to_parent_is_up_only() {
    let r = extrinsics().route("rslidar", "base_link").unwrap();
    assert_eq!(edge_names(&r.up), vec!["rslidar"]);
    assert!(r.down.is_empty());
}

#[test]
fn route_to_child_is_down_only() {
    let r = extrinsics().route("base_link", "camera_front").unwrap();
    assert!(r.up.is_empty());
    assert_eq!(edge_names(&r.down), vec!["camera_front"]);
}

#[test]
fn downward_dynamic_edges_are_listed_from_dst_upward() {
    match extrinsics().route("map", "rslidar") {
        Err(FrameError::DynamicEdge { edges, .. }) => assert_eq!(
            edges,
            vec![
                ("base_footprint".to_string(), "odom".to_string()),
                ("odom".to_string(), "map".to_string()),
            ]
        ),
        other => panic!("{other:?}"),
    }
}

#[test]
fn tool0_route_crosses_exactly_one_dynamic_edge() {
    match extrinsics().route("tool0", "camera_front") {
        Err(FrameError::DynamicEdge { edges, .. }) => {
            assert_eq!(edges, vec![("tool0".to_string(), "base_link".to_string())]);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn route_with_rejected_name_keeps_the_hint() {
    let e = extrinsics().route("lidar_link", "base_link").unwrap_err();
    assert!(matches!(e, FrameError::UnknownFrame(_)));
    assert!(e.to_string().contains("use 'rslidar'"), "{e}");
}

#[test]
fn load_errors_name_the_path() {
    let missing = temp_file("missing.yml", None);
    let bad = temp_file("bad.yml", Some("frames: [unclosed"));
    let shape = temp_file("shape.yml", Some("nodes: []"));
    for path in [&missing, &bad, &shape] {
        let e = FrameDict::load(path).unwrap_err();
        assert!(matches!(e, FrameError::Invalid(_)));
        assert!(e.to_string().contains(&path.display().to_string()), "{e}");
    }
    let f = frames();
    for path in [&missing, &bad] {
        let e = Extrinsics::load(path, &f).unwrap_err();
        assert!(e.to_string().contains(&path.display().to_string()), "{e}");
    }
    for path in [&bad, &shape] {
        let _ = std::fs::remove_file(path);
    }
}

#[test]
fn load_reads_a_valid_file_and_exposes_accessors() {
    let path = temp_file("ok-extrinsics.yml", Some(EXTRINSICS));
    let f = frames();
    let e = Extrinsics::load(&path, &f).unwrap();
    assert_eq!(e.path(), Some(path.as_path()));
    assert_eq!(e.robot(), "simulated-mobile-robot");
    assert!(!e.source().is_empty());
    assert_eq!(e.edges().len(), 4);
    assert_eq!(e.frames().get("map").map(|x| x.kind), Some(Kind::Fixed));
    assert!(f.rejected().contains_key("lidar_link"));
    assert!(f.frames().contains_key("tool0"));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn integer_translations_are_accepted() {
    let doc = "contract_version: 1.0.0\nrobot: r\ncalibrated: \"2026-10-20\"\nsource: s\ntransforms:\n  - {parent: base_link, child: rslidar, translation_m: [0, 0, 0], rotation_xyzw: [0, 0, 0, 1]}";
    let e = Extrinsics::parse(doc, &frames(), None).unwrap();
    assert_eq!(e.edges()["rslidar"].translation_m, [0.0, 0.0, 0.0]);
}

#[test]
fn empty_transforms_list_is_refused() {
    let doc =
        "contract_version: 1.0.0\nrobot: r\ncalibrated: \"2026-10-20\"\nsource: s\ntransforms: []";
    let e = Extrinsics::parse(doc, &frames(), None).unwrap_err();
    assert!(e.to_string().contains("non-empty list"), "{e}");
}

#[test]
fn non_string_robot_or_source_is_refused() {
    for (robot, source, key) in [("[r]", "s", "robot"), ("r", "5", "source")] {
        let doc = format!(
            "contract_version: 1.0.0\nrobot: {robot}\ncalibrated: \"2026-10-20\"\nsource: {source}\ntransforms:\n  - {{parent: base_link, child: rslidar, translation_m: [0, 0, 0], rotation_xyzw: [0, 0, 0, 1]}}"
        );
        let e = Extrinsics::parse(&doc, &frames(), None).unwrap_err();
        assert!(matches!(e, FrameError::Invalid(_)));
        assert!(
            e.to_string().contains(&format!("{key} must be a string")),
            "{e}"
        );
    }
}
