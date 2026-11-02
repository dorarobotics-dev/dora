use dora_core::descriptor::Descriptor;
use dora_core::descriptor::frames_check::check_frames;
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
    env: {{TARGET_FRAME: {target}, DORA_EXTRINSICS: {extrinsics}}}
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
    let with_wd = |extrinsics: &str, wd: &str| {
        format!(
            "{HEAD}{SIM}{}    deploy: {{working_dir: {wd}}}\n",
            transform_node("odom", true, extrinsics)
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
    let git = |extrinsics: &str| {
        format!(
            "{HEAD}{SIM}{}    git: https://example.com/ft.git\n",
            transform_node("odom", true, extrinsics)
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
