use dora_transform::{Extrinsics, FrameDict, FrameError, Kind};

const FRAMES: &str = include_str!("fixtures/frames.yml");
const EXTRINSICS: &str = include_str!("fixtures/extrinsics.yml");

fn frames() -> FrameDict { FrameDict::parse(FRAMES).unwrap() }
fn extrinsics() -> Extrinsics { Extrinsics::parse(EXTRINSICS, &frames(), None).unwrap() }

#[test]
fn reference_dictionary_loads() {
    let f = frames();
    assert_eq!(f.require("rslidar").unwrap().kind, Kind::Mounted);
    assert_eq!(f.chain("rslidar").unwrap(), vec!["rslidar", "base_link", "base_footprint", "odom", "map"]);
}

#[test]
fn rejected_name_says_what_to_use() {
    let e = frames().require("lidar_link").unwrap_err();
    assert!(matches!(e, FrameError::UnknownFrame(_)));
    assert!(e.to_string().contains("'rslidar'"), "{e}");
}

#[test]
fn unknown_frame_is_refused() {
    assert!(matches!(frames().require("nowhere"), Err(FrameError::UnknownFrame(_))));
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
        assert!(FrameDict::parse(bad).is_err(), "accepted: {bad}");
    }
}

#[test]
fn cycle_is_refused() {
    let doc = "frames: [{name: a, parent: b, kind: dynamic}, {name: b, parent: a, kind: dynamic}]";
    assert!(FrameDict::parse(doc).unwrap_err().to_string().contains("cycle"));
}

#[test]
fn static_route_up_and_down() {
    let r = extrinsics().route("rslidar", "camera_front").unwrap();
    assert_eq!(r.up.iter().map(|e| e.child.as_str()).collect::<Vec<_>>(), vec!["rslidar"]);
    assert_eq!(r.down.iter().map(|e| e.child.as_str()).collect::<Vec<_>>(), vec!["camera_front"]);
}

#[test]
fn route_through_one_dynamic_edge_names_it() {
    match extrinsics().route("rslidar", "odom") {
        Err(FrameError::DynamicEdge { edges, .. }) => {
            assert_eq!(edges, vec![("base_footprint".to_string(), "odom".to_string())]);
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
    assert!(matches!(extrinsics().route("none", "map"), Err(FrameError::UnknownFrame(_))));
}

#[test]
fn calibration_shape_errors() {
    let f = frames();
    let edge = |body: &str| format!(
        "contract_version: 1.0.0\nrobot: r\ncalibrated: \"2026-10-20\"\nsource: s\ntransforms:\n{body}");
    for bad in [
        edge("  - {parent: odom, child: base_footprint, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}"),
        edge("  - {parent: map, child: base_link, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}"),
        edge("  - {parent: base_link, child: rslidar, translation_m: [0,0], rotation_xyzw: [0,0,0,1]}"),
        edge("  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,2]}"),
        edge("  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}\n  - {parent: base_link, child: rslidar, translation_m: [0,0,0], rotation_xyzw: [0,0,0,1]}"),
        edge("  - {parent: base_link, child: rslidar, translation_m: [0,0,.nan], rotation_xyzw: [0,0,0,1]}"),
        edge("  - {parent: base_link, child: rslidar, translation_m: [0,0,true], rotation_xyzw: [0,0,0,1]}"),
        "robot: r\ntransforms: []".to_string(),
    ] {
        assert!(Extrinsics::parse(&bad, &f, None).is_err(), "accepted: {bad}");
    }
}
