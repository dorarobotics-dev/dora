use dora_core::descriptor::Descriptor;
use dora_core::descriptor::frames_check::{FrameViolation, check_frames};
use std::path::Path;

fn run(yaml: &str) -> Vec<String> {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/frames_check");
    let d: Descriptor = serde_yaml::from_str(yaml).unwrap();
    check_frames(&d, &base)
        .into_iter()
        .map(|v| format!("{}:{}:{}: {}", v.check, v.node, v.port, v.message))
        .collect()
}

const HEAD: &str = "frames: {dictionary: frames.yml, extrinsics: extrinsics.yml}\nnodes:\n";

const SIM: &str = r#"
  - id: sim
    path: sim.py
    outputs: [cloud, odom_pose]
    output_types: {cloud: std/media/v1/PointCloud, odom_pose: std/math/v1/Pose}
    output_frames: {cloud: rslidar, odom_pose: odom}
"#;

fn transform_node(target: &str, with_pose: bool, extrinsics: &str) -> String {
    let pose_in = if with_pose {
        "      pose: sim/odom_pose\n"
    } else {
        ""
    };
    format!(
        r#"
  - id: ft
    path: ft.py
    inputs:
      cloud: sim/cloud
{pose_in}    input_types: {{cloud: std/media/v1/PointCloud, pose: std/math/v1/Pose}}
    input_frames: {{cloud: any}}
    outputs: [cloud]
    output_frames: {{cloud: env:TARGET_FRAME}}
    env: {{TARGET_FRAME: {target}, DORA_FRAMES: frames.yml, DORA_EXTRINSICS: {extrinsics}}}
"#
    )
}

#[test]
fn good_chain_passes() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics.yml")
    ));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn no_frames_key_means_no_checks() {
    let yaml = format!("nodes:\n{SIM}").replace("rslidar", "nowhere");
    assert!(run(&yaml).is_empty());
}

#[test]
fn unknown_and_rejected_frames() {
    let v = run(&format!(
        "{HEAD}{}",
        SIM.replace("cloud: rslidar", "cloud: lidar_link")
    ));
    assert!(
        v.iter()
            .any(|m| m.starts_with("unknown_frame:sim:cloud") && m.contains("'rslidar'")),
        "{v:?}"
    );

    // Reported once, by the producer, not again by each consumer's route.
    let v = run(&format!(
        "{HEAD}{}{}",
        SIM.replace("cloud: rslidar", "cloud: lidar_link"),
        transform_node("odom", true, "extrinsics.yml")
    ));
    assert_eq!(v.len(), 1, "{v:?}");
}

#[test]
fn edge_frame_mismatch() {
    let sink = "\n  - id: sink\n    path: s.py\n    inputs: {cloud: sim/cloud}\n    input_frames: {cloud: base_link}\n";
    let v = run(&format!("{HEAD}{SIM}{sink}"));
    assert!(
        v.iter().any(|m| m.starts_with("frame:sink:cloud")
            && m.contains("'base_link'")
            && m.contains("'rslidar'")),
        "{v:?}"
    );
}

#[test]
fn env_target_must_be_a_real_frame() {
    for bad in ["any", "none", "same_as:cloud", "env:X"] {
        let v = run(&format!(
            "{HEAD}{SIM}{}",
            transform_node(&format!("\"{bad}\""), true, "extrinsics.yml")
        ));
        assert!(
            v.iter().any(
                |m| m.starts_with("frame:ft:cloud") || m.starts_with("unknown_frame:ft:cloud")
            ),
            "{bad}: {v:?}"
        );
    }
}

#[test]
fn dynamic_edge_without_pose_input() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", false, "extrinsics.yml")
    ));
    assert!(
        v.iter()
            .any(|m| m.starts_with("transform:ft:cloud") && m.contains("std/math/v1/Pose")),
        "{v:?}"
    );
}

#[test]
fn two_dynamic_edges_refused_even_with_pose() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("map", true, "extrinsics.yml")
    ));
    assert!(
        v.iter().any(|m| m.contains("crosses 2 dynamic edges")),
        "{v:?}"
    );
}

#[test]
fn missing_calibration_refused() {
    let yaml = format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics-no-base.yml")
    )
    .replace(
        "extrinsics: extrinsics.yml",
        "extrinsics: extrinsics-no-base.yml",
    );
    let v = run(&yaml);
    assert!(
        v.iter().any(|m| m.contains("never assumed to be identity")),
        "{v:?}"
    );
}

#[test]
fn launch_calibration_must_match_checked_one() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics-no-base.yml")
    ));
    assert!(
        v.iter()
            .any(|m| m.contains("a robot must run the calibration it was checked with")),
        "{v:?}"
    );
}

#[test]
fn launch_calibration_that_does_not_exist() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "other.yml")
    ));
    assert!(
        v.iter()
            .any(|m| m.starts_with("transform:ft:DORA_EXTRINSICS")
                && m.ends_with("frames_check/other.yml does not exist")),
        "{v:?}"
    );
}

#[test]
fn frame_change_without_extrinsics_env() {
    let yaml = format!("{HEAD}{SIM}{}", transform_node("odom", true, "x"))
        .replace(", DORA_EXTRINSICS: x", "");
    let v = run(&yaml);
    assert!(
        v.iter().any(|m| m.contains("sets no DORA_EXTRINSICS")),
        "{v:?}"
    );
}

// Beyond the task's minimum list.

#[test]
fn same_as_follows_upstream_and_cycles_are_errors() {
    let relay = "\n  - id: relay\n    path: r.py\n    inputs: {cloud: sim/cloud}\n    outputs: [cloud]\n    output_frames: {cloud: same_as:cloud}\n  - id: sink\n    path: s.py\n    inputs: {cloud: relay/cloud}\n    input_frames: {cloud: rslidar}\n";
    assert!(run(&format!("{HEAD}{SIM}{relay}")).is_empty());

    let cycle = "\n  - id: a\n    path: a.py\n    inputs: {x: b/y}\n    outputs: [y]\n    output_frames: {y: same_as:x}\n  - id: b\n    path: b.py\n    inputs: {x: a/y}\n    outputs: [y]\n    output_frames: {y: same_as:x}\n  - id: sink\n    path: s.py\n    inputs: {y: a/y}\n    input_frames: {y: rslidar}\n";
    let v = run(&format!("{HEAD}{cycle}"));
    assert!(
        v.iter()
            .any(|m| m.starts_with("frame:sink:y") && m.contains("frame resolution cycle")),
        "{v:?}"
    );
}

#[test]
fn upstream_without_a_frame_cannot_be_checked() {
    let sink = "\n  - id: sink\n    path: s.py\n    inputs: {pose: sim/odom_pose, cloud: sim/cloud}\n    input_frames: {cloud: rslidar}\n";
    let yaml = format!("{HEAD}{SIM}{sink}").replace(", odom_pose: odom}", "}");
    assert!(
        run(&yaml).is_empty(),
        "an undeclared frame on an unchecked input is fine"
    );
    let yaml = yaml.replace(
        "input_frames: {cloud: rslidar}",
        "input_frames: {cloud: rslidar, pose: odom}",
    );
    let v = run(&yaml);
    assert!(
        v.iter()
            .any(|m| m.starts_with("frame:sink:pose")
                && m.contains("cannot be checked before launch")),
        "{v:?}"
    );
}

#[test]
fn dataflow_env_reaches_the_node() {
    // dora launches every node with the dataflow-level `env:` under its own.
    let yaml = format!(
        "env: {{DORA_EXTRINSICS: extrinsics.yml}}\n{HEAD}{SIM}{}",
        transform_node("odom", true, "x")
    )
    .replace(", DORA_EXTRINSICS: x", "");
    let v = run(&yaml);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn timer_inputs_are_not_data_inputs() {
    let yaml = format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics.yml")
    )
    .replace(
        "      cloud: sim/cloud\n",
        "      cloud: sim/cloud\n      tick: dora/timer/millis/100\n",
    );
    assert!(run(&yaml).is_empty());
}

#[test]
fn unreadable_calibration_is_reported_once() {
    let yaml = format!("{HEAD}{SIM}{}", transform_node("odom", true, "missing.yml"))
        .replace("extrinsics: extrinsics.yml", "extrinsics: missing.yml");
    let v = run(&yaml);
    assert_eq!(
        v.iter()
            .filter(|m| m.starts_with("transform:-:frames.extrinsics"))
            .count(),
        1,
        "{v:?}"
    );
}

fn base() -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/frames_check")
        .display()
        .to_string()
}

#[test]
fn extrinsics_resolve_against_deploy_working_dir() {
    let frames = format!("DORA_FRAMES: {}/frames.yml", base());
    let with_wd = |extrinsics: &str, wd: &str| {
        format!(
            "{HEAD}{SIM}{}    deploy: {{working_dir: {wd}}}\n",
            transform_node("odom", true, extrinsics).replace("DORA_FRAMES: frames.yml", &frames)
        )
    };
    // The node runs from <dataflow dir>/sub, so ../extrinsics.yml is the checked file.
    assert!(run(&with_wd("../extrinsics.yml", "sub")).is_empty());
    let v = run(&with_wd("extrinsics.yml", "sub"));
    assert!(
        v.iter()
            .any(|m| m.ends_with("frames_check/sub/extrinsics.yml does not exist")),
        "{v:?}"
    );
    // An absolute working_dir is used as is.
    assert!(run(&with_wd("extrinsics.yml", &base())).is_empty());
}

#[test]
fn git_node_with_relative_extrinsics_cannot_be_checked() {
    let frames = format!("DORA_FRAMES: {}/frames.yml", base());
    let git = |extrinsics: &str| {
        format!(
            "{HEAD}{SIM}{}    git: https://example.com/ft.git\n",
            transform_node("odom", true, extrinsics).replace("DORA_FRAMES: frames.yml", &frames)
        )
    };
    let v = run(&git("extrinsics.yml"));
    assert!(
        v.iter()
            .any(|m| m.starts_with("transform:ft:DORA_EXTRINSICS")
                && m.contains("is relative and the node runs from its git checkout")
                && m.contains("use an absolute path")),
        "{v:?}"
    );
    let absolute = format!("{}/extrinsics.yml", base());
    assert!(run(&git(&absolute)).is_empty());
}

#[test]
fn absolute_extrinsics_path() {
    let good = format!("{}/extrinsics.yml", base());
    assert!(
        run(&format!(
            "{HEAD}{SIM}{}",
            transform_node("odom", true, &good)
        ))
        .is_empty()
    );
    let other = format!("{}/extrinsics-no-base.yml", base());
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, &other)
    ));
    assert!(
        v.iter()
            .any(|m| m.contains("a robot must run the calibration it was checked with")),
        "{v:?}"
    );
}

#[test]
fn pose_input_typed_by_its_upstream_output() {
    // `pose` has no input_types entry; sim/odom_pose is a Pose by output_types.
    let untyped = |target: &str| {
        transform_node(target, true, "extrinsics.yml").replace(
            "input_types: {cloud: std/media/v1/PointCloud, pose: std/math/v1/Pose}",
            "input_types: {cloud: std/media/v1/PointCloud}",
        )
    };
    // It satisfies the one-dynamic-edge check ...
    let v = run(&format!("{HEAD}{SIM}{}", untyped("odom")));
    assert!(v.is_empty(), "{v:?}");
    // ... and is not data: a pose in `map` would otherwise need
    // map -> base_link, which crosses two dynamic edges.
    let sim_in_map = SIM.replace("odom_pose: odom}", "odom_pose: map}");
    let v = run(&format!("{HEAD}{sim_in_map}{}", untyped("base_link")));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn two_inputs_in_one_frame_report_one_route() {
    let ft = transform_node("odom", false, "extrinsics.yml").replace(
        "      cloud: sim/cloud\n",
        "      cloud: sim/cloud\n      cloud2: sim/cloud\n",
    );
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    let routes: Vec<_> = v
        .iter()
        .filter(|m| m.starts_with("transform:ft:cloud"))
        .collect();
    assert_eq!(routes.len(), 1, "{v:?}");
}

#[test]
fn none_frames_are_skipped_by_the_route_check() {
    // An input in `none`: nothing to move, so no route and no calibration needed.
    let ft = transform_node("odom", false, "x").replace(", DORA_EXTRINSICS: x", "");
    let sim_none = SIM.replace("cloud: rslidar", "cloud: none");
    assert_eq!(run(&format!("{HEAD}{sim_none}{ft}")), Vec::<String>::new());
    // An output in `none`.
    let ft = ft.replace(
        "output_frames: {cloud: env:TARGET_FRAME}",
        "output_frames: {cloud: none}",
    );
    assert_eq!(run(&format!("{HEAD}{SIM}{ft}")), Vec::<String>::new());
}

#[test]
fn same_as_an_unwired_input() {
    let relay = "\n  - id: relay\n    path: r.py\n    inputs: {cloud: sim/cloud}\n    outputs: [cloud]\n    output_frames: {cloud: same_as:missing}\n  - id: sink\n    path: s.py\n    inputs: {cloud: relay/cloud}\n    input_frames: {cloud: rslidar}\n";
    let v = run(&format!("{HEAD}{SIM}{relay}"));
    assert!(
        v.iter().any(|m| m.starts_with("frame:sink:cloud")
            && m.contains(
                "relay.missing is unwired, so relay.cloud cannot resolve same_as:missing"
            )),
        "{v:?}"
    );
}

#[test]
fn empty_node_extrinsics_overrides_dataflow_env() {
    let yaml = format!(
        "env: {{DORA_EXTRINSICS: extrinsics.yml}}\n{HEAD}{SIM}{}",
        transform_node("odom", true, "\"\"")
    );
    let v = run(&yaml);
    assert!(
        v.iter().any(|m| m.contains("sets no DORA_EXTRINSICS")),
        "{v:?}"
    );
}

#[test]
fn env_frames_are_not_trimmed() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("\" odom\"", true, "extrinsics.yml")
    ));
    assert!(
        v.iter()
            .any(|m| m.starts_with("unknown_frame:ft:cloud") && m.contains("' odom'")),
        "{v:?}"
    );
}

// Parity with the reference suite (docs/contract-core/test-mapping.md).

#[test]
fn a_calibrated_static_frame_change_passes() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("camera_front", false, "extrinsics.yml")
    ));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn an_env_frame_with_its_variable_unset_is_reported() {
    let unset = transform_node("x", true, "extrinsics.yml").replace("TARGET_FRAME: x, ", "");
    let v = run(&format!("{HEAD}{SIM}{unset}"));
    assert_eq!(v.len(), 1, "{v:?}");
    assert!(
        v[0].starts_with("frame:ft:cloud") && v[0].contains("sets no TARGET_FRAME"),
        "{v:?}"
    );
    // A value-less entry is unset, never the string "None"/"null".
    let empty = transform_node("\"\"", true, "extrinsics.yml");
    let v = run(&format!("{HEAD}{SIM}{empty}"));
    assert!(
        v.iter().any(|m| m.contains("sets no TARGET_FRAME")),
        "{v:?}"
    );
}

#[test]
fn a_rejected_env_target_names_its_replacement() {
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("lidar_link", true, "extrinsics.yml")
    ));
    assert_eq!(v.len(), 1, "{v:?}");
    assert!(
        v[0].starts_with("unknown_frame:ft:cloud") && v[0].contains("use 'rslidar'"),
        "{v:?}"
    );
}

#[test]
fn a_literal_output_that_changes_frame_is_checked() {
    let literal = |frame: &str| {
        transform_node("x", false, "x")
            .replace(", DORA_EXTRINSICS: x", "")
            .replace("{cloud: env:TARGET_FRAME}", &format!("{{cloud: {frame}}}"))
    };
    // rslidar in, base_link out: a frame change with no calibration named.
    let v = run(&format!("{HEAD}{SIM}{}", literal("base_link")));
    assert!(
        v.iter()
            .any(|m| m.starts_with("transform:ft:DORA_EXTRINSICS")
                && m.contains("sets no DORA_EXTRINSICS")),
        "{v:?}"
    );
    // rslidar in, rslidar out: no frame change, nothing to calibrate.
    assert!(run(&format!("{HEAD}{SIM}{}", literal("rslidar"))).is_empty());
}

#[test]
fn same_as_follows_a_three_hop_chain_and_a_literal_stops_it() {
    let relay = |id: &str, from: &str| {
        format!(
            "\n  - id: {id}\n    path: r.py\n    inputs: {{cloud: {from}/cloud}}\n    outputs: [cloud]\n    output_frames: {{cloud: same_as:cloud}}\n"
        )
    };
    let sink = |from: &str, frame: &str| {
        format!(
            "\n  - id: sink\n    path: s.py\n    inputs: {{cloud: {from}/cloud}}\n    input_frames: {{cloud: {frame}}}\n"
        )
    };
    let chain = format!(
        "{}{}{}",
        relay("r1", "sim"),
        relay("r2", "r1"),
        relay("r3", "r2")
    );
    assert!(run(&format!("{HEAD}{SIM}{chain}{}", sink("r3", "rslidar"))).is_empty());
    let v = run(&format!("{HEAD}{SIM}{chain}{}", sink("r3", "base_link")));
    assert!(
        v.iter()
            .any(|m| m.starts_with("frame:sink:cloud") && m.contains("'rslidar'")),
        "{v:?}"
    );

    // A literal output ends the chain: what comes after is in its frame.
    let lit = "\n  - id: tf\n    path: t.py\n    inputs: {cloud: r3/cloud}\n    outputs: [cloud]\n    output_frames: {cloud: base_link}\n    env: {DORA_FRAMES: frames.yml, DORA_EXTRINSICS: extrinsics.yml}\n";
    let after = relay("r4", "tf");
    assert!(
        run(&format!(
            "{HEAD}{SIM}{chain}{lit}{after}{}",
            sink("r4", "base_link")
        ))
        .is_empty()
    );
}

#[test]
fn a_same_as_self_loop_terminates() {
    let looped = "\n  - id: a\n    path: a.py\n    inputs: {x: a/y}\n    outputs: [y]\n    output_frames: {y: same_as:x}\n  - id: sink\n    path: s.py\n    inputs: {y: a/y}\n    input_frames: {y: rslidar}\n";
    let v = run(&format!("{HEAD}{looped}"));
    assert!(
        v.iter()
            .any(|m| m.contains("frame resolution cycle: a.y -> a.y")),
        "{v:?}"
    );
}

#[test]
fn an_unreadable_frame_dictionary_is_reported_by_name() {
    for dictionary in ["absent.yml", "extrinsics-no-base.yml"] {
        let yaml = format!("{HEAD}{SIM}").replace(
            "dictionary: frames.yml",
            &format!("dictionary: {dictionary}"),
        );
        let v = run(&yaml);
        assert_eq!(v.len(), 1, "{v:?}");
        assert!(
            v[0].starts_with("transform:-:frames.dictionary") && v[0].contains(dictionary),
            "{v:?}"
        );
    }
}

#[test]
fn a_violation_renders_check_node_and_port() {
    let v = FrameViolation {
        check: "frame",
        node: "sink".into(),
        port: "cloud".into(),
        message: "boom".into(),
    };
    assert_eq!(v.to_string(), "[frame] sink/cloud: boom");
}

#[test]
fn a_value_less_env_entry_is_refused_not_read_as_none() {
    // dora refuses a null env value when it parses the dataflow, so a
    // value-less TARGET_FRAME can never reach a node as "None" or "null".
    let yaml = format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics.yml")
    )
    .replace("TARGET_FRAME: odom,", "TARGET_FRAME: ,");
    let err = serde_yaml::from_str::<Descriptor>(&yaml)
        .unwrap_err()
        .to_string();
    assert!(err.contains("env"), "{err}");
}

// Check 4, partly: the wired pose input covers the dynamic edge.

#[test]
fn a_pose_in_the_dynamic_edges_parent_frame_is_accepted() {
    // rslidar -> odom crosses odom -> base_footprint; sim/odom_pose is in odom.
    let v = run(&format!(
        "{HEAD}{SIM}{}",
        transform_node("odom", true, "extrinsics.yml")
    ));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn a_pose_in_another_frame_than_the_dynamic_edges_parent_is_refused() {
    // A camera pose in `map` cannot stand in for odom -> base_footprint.
    let sim = SIM.replace("odom_pose: odom}", "odom_pose: map}");
    let v = run(&format!(
        "{HEAD}{sim}{}",
        transform_node("odom", true, "extrinsics.yml")
    ));
    assert_eq!(v.len(), 1, "{v:?}");
    assert!(
        v[0].starts_with("transform:ft:cloud")
            && v[0].contains("'odom' -> 'base_footprint'")
            && v[0].contains("pose input 'pose' (sim/odom_pose) is in 'map'"),
        "{v:?}"
    );
}

#[test]
fn a_pose_frame_resolved_through_same_as_is_checked() {
    let relay = "\n  - id: relay\n    path: r.py\n    inputs: {pose: sim/odom_pose}\n    outputs: [pose]\n    output_types: {pose: std/math/v1/Pose}\n    output_frames: {pose: same_as:pose}\n";
    let ft = transform_node("odom", true, "extrinsics.yml")
        .replace("pose: sim/odom_pose", "pose: relay/pose");
    assert!(run(&format!("{HEAD}{SIM}{relay}{ft}")).is_empty());
    let sim = SIM.replace("odom_pose: odom}", "odom_pose: map}");
    let v = run(&format!("{HEAD}{sim}{relay}{ft}"));
    assert!(
        v.iter()
            .any(|m| m.contains("pose input 'pose' (relay/pose) is in 'map'")),
        "{v:?}"
    );
}

#[test]
fn a_pose_with_no_declared_frame_is_accepted() {
    // No descriptor field declares a pose's frame or child_frame, so an
    // undeclared one cannot be checked before launch.
    let sim = SIM.replace(", odom_pose: odom}", "}");
    let v = run(&format!(
        "{HEAD}{sim}{}",
        transform_node("odom", true, "extrinsics.yml")
    ));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn one_matching_pose_input_covers_the_edge() {
    let sim = SIM
        .replace(
            "outputs: [cloud, odom_pose]",
            "outputs: [cloud, odom_pose, map_pose]",
        )
        .replace(
            "odom_pose: std/math/v1/Pose}",
            "odom_pose: std/math/v1/Pose, map_pose: std/math/v1/Pose}",
        )
        .replace("odom_pose: odom}", "odom_pose: odom, map_pose: map}");
    let ft = transform_node("odom", true, "extrinsics.yml").replace(
        "      pose: sim/odom_pose\n",
        "      pose: sim/odom_pose\n      pose2: sim/map_pose\n",
    );
    assert!(run(&format!("{HEAD}{sim}{ft}")).is_empty());
}

// Route reporting: the first failing route per node.

#[test]
fn only_the_first_failing_route_of_a_node_is_reported() {
    let ft = transform_node("odom", false, "extrinsics.yml")
        .replace("outputs: [cloud]", "outputs: [cloud, cloud2]")
        .replace(
            "output_frames: {cloud: env:TARGET_FRAME}",
            "output_frames: {cloud: env:TARGET_FRAME, cloud2: map}",
        );
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    let routes: Vec<_> = v
        .iter()
        .filter(|m| m.starts_with("transform:ft:cloud"))
        .collect();
    assert_eq!(routes.len(), 1, "{v:?}");
    // Another node still gets its own report.
    let ft2 = transform_node("odom", false, "extrinsics.yml").replace("id: ft", "id: ft2");
    let v = run(&format!("{HEAD}{SIM}{ft}{ft2}"));
    assert_eq!(
        v.iter()
            .filter(|m| m.starts_with("transform:ft2:cloud"))
            .count(),
        1,
        "{v:?}"
    );
}

// DORA_FRAMES is pinned to frames.dictionary as DORA_EXTRINSICS is to frames.extrinsics.

#[test]
fn launch_dictionary_must_match_checked_one() {
    let ft = transform_node("odom", true, "extrinsics.yml")
        .replace("DORA_FRAMES: frames.yml", "DORA_FRAMES: extrinsics.yml");
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    assert!(
        v.iter().any(|m| m.starts_with("transform:ft:DORA_FRAMES")
            && m.contains("a robot must run the frame dictionary it was checked with")),
        "{v:?}"
    );
}

#[test]
fn launch_dictionary_that_does_not_exist() {
    let ft = transform_node("odom", true, "extrinsics.yml")
        .replace("DORA_FRAMES: frames.yml", "DORA_FRAMES: other.yml");
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    assert!(
        v.iter().any(|m| m.starts_with("transform:ft:DORA_FRAMES")
            && m.ends_with("frames_check/other.yml does not exist")),
        "{v:?}"
    );
}

#[test]
fn frame_change_without_frames_env() {
    let ft =
        transform_node("odom", true, "extrinsics.yml").replace("DORA_FRAMES: frames.yml, ", "");
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    assert_eq!(v.len(), 1, "{v:?}");
    assert!(
        v[0].starts_with("transform:ft:DORA_FRAMES") && v[0].contains("sets no DORA_FRAMES"),
        "{v:?}"
    );
    // The dataflow-level env reaches the node.
    let yaml = format!("env: {{DORA_FRAMES: frames.yml}}\n{HEAD}{SIM}{ft}");
    assert!(run(&yaml).is_empty());
    // An empty node value overrides it.
    let empty = transform_node("odom", true, "extrinsics.yml")
        .replace("DORA_FRAMES: frames.yml", "DORA_FRAMES: \"\"");
    let v = run(&format!(
        "env: {{DORA_FRAMES: frames.yml}}\n{HEAD}{SIM}{empty}"
    ));
    assert!(v.iter().any(|m| m.contains("sets no DORA_FRAMES")), "{v:?}");
}

#[test]
fn dictionary_resolves_against_deploy_working_dir() {
    let with_wd = |frames: &str, wd: &str| {
        format!(
            "{HEAD}{SIM}{}    deploy: {{working_dir: {wd}}}\n",
            transform_node("odom", true, "../extrinsics.yml")
                .replace("DORA_FRAMES: frames.yml", &format!("DORA_FRAMES: {frames}"))
        )
    };
    assert!(run(&with_wd("../frames.yml", "sub")).is_empty());
    let v = run(&with_wd("frames.yml", "sub"));
    assert!(
        v.iter().any(|m| m.starts_with("transform:ft:DORA_FRAMES")
            && m.ends_with("frames_check/sub/frames.yml does not exist")),
        "{v:?}"
    );
}

#[test]
fn git_node_with_relative_dictionary_cannot_be_checked() {
    let absolute_ex = format!("{}/extrinsics.yml", base());
    let git = |frames: &str| {
        format!(
            "{HEAD}{SIM}{}    git: https://example.com/ft.git\n",
            transform_node("odom", true, &absolute_ex)
                .replace("DORA_FRAMES: frames.yml", &format!("DORA_FRAMES: {frames}"))
        )
    };
    let v = run(&git("frames.yml"));
    assert!(
        v.iter().any(|m| m.starts_with("transform:ft:DORA_FRAMES")
            && m.contains("DORA_FRAMES 'frames.yml' is relative")
            && m.contains("use an absolute path")),
        "{v:?}"
    );
    assert!(run(&git(&format!("{}/frames.yml", base()))).is_empty());
}

#[test]
fn dictionary_paths_are_not_trimmed() {
    let ft = transform_node("odom", true, "extrinsics.yml")
        .replace("DORA_FRAMES: frames.yml", "DORA_FRAMES: \" frames.yml\"");
    let v = run(&format!("{HEAD}{SIM}{ft}"));
    assert!(
        v.iter().any(|m| m.starts_with("transform:ft:DORA_FRAMES")
            && m.ends_with("frames_check/ frames.yml does not exist")),
        "{v:?}"
    );
}
