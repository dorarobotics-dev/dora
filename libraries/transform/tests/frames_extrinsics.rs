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

// Parity with the reference suite (docs/contract-core/test-mapping.md).

fn message(r: Result<impl std::fmt::Debug, FrameError>) -> String {
    r.unwrap_err().to_string()
}

#[test]
fn malformed_dictionary_messages() {
    for (doc, want) in [
        (
            "frames: [{name: a, parent: null, kind: floating}]",
            "kind \"floating\"",
        ),
        (
            "frames: [{name: a, parent: null, kind: mounted}]",
            "needs a parent",
        ),
        (
            "frames: [{name: a, parent: null, kind: dynamic}]",
            "needs a parent",
        ),
        (
            "frames: [{name: m, kind: fixed}, {name: a, parent: m, kind: fixed}]",
            "cannot have a parent",
        ),
        (
            "frames: [{name: a, parent: ghost, kind: mounted}]",
            "unknown parent 'ghost'",
        ),
        (
            "frames: [{name: a, kind: fixed}, {name: a, kind: fixed}]",
            "more than once",
        ),
        ("frames: [{name: '', kind: fixed}]", "non-empty name"),
        (
            "frames: [{name: a, parent: b, kind: dynamic}, {name: b, parent: a, kind: dynamic}]",
            "cycle",
        ),
        (
            "frames: [{name: map, kind: fixed}]\nrejected_names: {map: map}",
            "both a frame and a rejected name",
        ),
        (
            "frames: [{name: map, kind: fixed}]\nrejected_names: {world: globe}",
            "unknown frame 'globe'",
        ),
        ("contract_version: 1.1.0", "'frames' list"),
        ("frames: [{name: a, parent: [x], kind: mounted}]", "parent"),
        (
            "frames: [{name: map, kind: fixed}]\nrejected_names: {world: [map]}",
            "rejected",
        ),
    ] {
        let e = FrameDict::parse(doc).unwrap_err();
        assert!(matches!(e, FrameError::Invalid(_)), "{doc}: {e:?}");
        assert!(e.to_string().contains(want), "{doc}: {e}");
    }
}

#[test]
fn chain_of_a_root_and_of_an_unknown_name() {
    assert_eq!(frames().chain("map").unwrap(), vec!["map"]);
    let e = frames().chain("velodyne").unwrap_err();
    assert!(matches!(e, FrameError::UnknownFrame(_)));
    assert!(e.to_string().contains("not in the frame dictionary"), "{e}");
}

#[test]
fn known_frame_has_its_kind_and_parent() {
    let f = frames();
    let r = f.require("rslidar").unwrap();
    assert_eq!(
        (r.name.as_str(), r.parent.as_deref(), r.kind),
        ("rslidar", Some("base_link"), Kind::Mounted)
    );
    assert_eq!(f.require("tool0").unwrap().kind, Kind::Dynamic);
    assert_eq!(f.rejected()["livox_frame"], "rslidar");
}

fn calibration(entries: &str) -> String {
    format!(
        "contract_version: 1.0.0\nrobot: test-bot\ncalibrated: \"2026-10-20\"\nsource: unit-test fixture\ntransforms:\n{entries}"
    )
}

fn entry(parent: &str, child: &str, t: &str, q: &str) -> String {
    format!("  - {{parent: {parent}, child: {child}, translation_m: {t}, rotation_xyzw: {q}}}\n")
}

#[test]
fn bad_extrinsic_messages() {
    let (t0, q0) = ("[0.0, 0.0, 0.0]", "[0.0, 0.0, 0.0, 1.0]");
    for (e, want) in [
        (
            entry("odom", "base_footprint", t0, q0),
            "only mounted frames are calibrated",
        ),
        (
            entry("map", "odom", t0, q0),
            "only mounted frames are calibrated",
        ),
        (
            entry("map", "rslidar", t0, q0),
            "hangs from Some(\"base_link\")",
        ),
        (entry("base_link", "lidar_link", t0, q0), "use 'rslidar'"),
        (
            entry("base_link", "velodyne", t0, q0),
            "not in the frame dictionary",
        ),
        (
            entry("base_link", "rslidar", t0, "[0.0, 0.0, 0.0, 0.9]"),
            "unit quaternion",
        ),
        (
            entry("base_link", "rslidar", t0, "[0.0, 0.0, 0.0, 1.000002]"),
            "unit quaternion",
        ),
        (
            entry("base_link", "rslidar", "[0.0, .nan, 0.0]", q0),
            "not a finite number",
        ),
        (
            entry("base_link", "rslidar", "[0.0, 0.0]", q0),
            "list of 3 numbers",
        ),
        (
            entry("base_link", "rslidar", t0, "[0.0, 0.0, 1.0]"),
            "list of 4 numbers",
        ),
        (
            entry("base_link", "rslidar", "[true, 0.0, 0.0]", q0),
            "not a finite number",
        ),
        (
            "  - base_link -> rslidar\n".to_string(),
            "parent and child must be frame names",
        ),
        (entry("[base_link]", "rslidar", t0, q0), "parent and child"),
        (entry("base_link", "[rslidar]", t0, q0), "parent and child"),
    ] {
        let doc = calibration(&e);
        let err = message(Extrinsics::parse(&doc, &frames(), None));
        assert!(err.contains(want), "{e}: {err}");
    }
}

#[test]
fn a_quaternion_within_tolerance_is_kept_not_normalised() {
    let doc = calibration(&entry(
        "base_link",
        "rslidar",
        "[0.0, 0.0, 0.0]",
        "[0.0, 0.0, 0.0, 1.0000005]",
    ));
    let e = Extrinsics::parse(&doc, &frames(), None).unwrap();
    assert_eq!(
        e.edges()["rslidar"].rotation_xyzw,
        [0.0, 0.0, 0.0, 1.0000005]
    );
}

#[test]
fn every_top_level_key_is_required() {
    let full = calibration(&entry("base_link", "rslidar", "[0, 0, 0]", "[0, 0, 0, 1]"));
    for key in [
        "contract_version",
        "robot",
        "calibrated",
        "source",
        "transforms",
    ] {
        let doc: String = full
            .lines()
            .filter(|l| !l.starts_with(&format!("{key}:")))
            .map(|l| format!("{l}\n"))
            .collect();
        let doc = if key == "transforms" {
            doc.lines()
                .filter(|l| !l.starts_with("  -"))
                .map(|l| format!("{l}\n"))
                .collect()
        } else {
            doc
        };
        let err = message(Extrinsics::parse(&doc, &frames(), None));
        assert!(
            err.contains(&format!("missing the required key '{key}'")),
            "{key}: {err}"
        );
    }
}

#[test]
fn a_route_descends_two_edges_from_the_ancestor() {
    let r = extrinsics()
        .route("base_footprint", "camera_front")
        .unwrap();
    assert!(r.up.is_empty());
    assert_eq!(edge_names(&r.down), vec!["base_link", "camera_front"]);
}

#[test]
fn a_missing_extrinsic_anywhere_on_the_route_outranks_a_dynamic_edge() {
    // tool0 is dynamic (a pose can stand in for it); camera_front is not calibrated.
    let doc = calibration(&entry(
        "base_footprint",
        "base_link",
        "[0.0, 0.0, 0.1]",
        "[0, 0, 0, 1]",
    ));
    let e = Extrinsics::parse(&doc, &frames(), None).unwrap();
    for (src, dst) in [("tool0", "camera_front"), ("camera_front", "tool0")] {
        let err = e.route(src, dst).unwrap_err();
        assert!(
            matches!(err, FrameError::MissingExtrinsic(_)),
            "{src}->{dst}: {err:?}"
        );
        assert!(
            err.to_string().contains("'base_link' -> 'camera_front'"),
            "{err}"
        );
    }
}

#[test]
fn a_missing_extrinsic_on_a_static_route_is_never_identity() {
    let doc = calibration(&entry(
        "base_link",
        "rslidar",
        "[0.25, 0.0, 0.25]",
        "[0, 0, 0, 1]",
    ));
    let e = Extrinsics::parse(&doc, &frames(), None).unwrap();
    let err = e.route("imu_link", "base_link").unwrap_err();
    assert!(matches!(err, FrameError::MissingExtrinsic(_)));
    assert!(
        err.to_string().contains("never assumed to be identity"),
        "{err}"
    );
}

#[test]
fn a_dynamic_edge_error_names_every_edge_it_crosses() {
    let err = extrinsics().route("rslidar", "map").unwrap_err();
    assert!(
        err.to_string().contains("'odom' -> 'base_footprint'"),
        "{err}"
    );
    match err {
        FrameError::DynamicEdge { edges, .. } => assert_eq!(
            edges,
            vec![
                ("base_footprint".to_string(), "odom".to_string()),
                ("odom".to_string(), "map".to_string()),
            ]
        ),
        other => panic!("{other:?}"),
    }
    let none = extrinsics().route("none", "base_link").unwrap_err();
    assert!(
        none.to_string().contains("never part of a transform"),
        "{none}"
    );
}
