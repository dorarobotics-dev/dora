use dora_transform::rigid::{Transform, lookup, pose_transform, resolve};
use dora_transform::{Extrinsics, FrameDict, FrameError};

const FRAMES: &str = include_str!("fixtures/frames.yml");
const EXTRINSICS: &str = include_str!("fixtures/extrinsics.yml");
fn ex() -> Extrinsics {
    Extrinsics::parse(EXTRINSICS, &FrameDict::parse(FRAMES).unwrap(), None).unwrap()
}

fn close(a: [f64; 3], b: [f64; 3]) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
}

#[test]
fn non_rigid_matrix_is_refused() {
    let mut m = *Transform::identity("a").matrix();
    m[0][0] = 2.0; // scale
    assert!(Transform::new("a", "b", m).is_err());
    let mut r = *Transform::identity("a").matrix();
    r[0][0] = -1.0; // reflection
    assert!(Transform::new("a", "b", r).is_err());
    let mut last = *Transform::identity("a").matrix();
    last[3][0] = 1.0;
    assert!(Transform::new("a", "b", last).is_err());
}

#[test]
fn then_requires_frames_to_meet() {
    let a = Transform::identity("a");
    let b = Transform::from_parts("b", "c", [0.0; 3], [0.0, 0.0, 0.0, 1.0]).unwrap();
    assert!(matches!(a.then(&b), Err(FrameError::FrameOrder(_))));
}

#[test]
fn rotation_is_read_xyzw() {
    // 90 degrees about z: x axis goes to y.
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let t = Transform::from_parts("a", "b", [0.0; 3], [0.0, 0.0, s, s]).unwrap();
    assert!(close(t.apply([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]));
}

#[test]
fn inverse_undoes() {
    let t = Transform::from_parts(
        "a",
        "b",
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.3826834323650898, 0.9238795325112867],
    )
    .unwrap();
    let back = t.then(&t.inverse()).unwrap();
    assert!(close(back.apply([4.0, 5.0, 6.0]), [4.0, 5.0, 6.0]));
}

#[test]
fn lookup_lidar_to_base_footprint() {
    let t = lookup(&ex(), "rslidar", "base_footprint").unwrap();
    assert!(close(t.apply([0.0, 0.0, 0.0]), [0.25, 0.0, 0.35]));
}

#[test]
fn lookup_through_dynamic_edge_is_refused() {
    assert!(matches!(
        lookup(&ex(), "rslidar", "odom"),
        Err(FrameError::DynamicEdge { .. })
    ));
}

#[test]
fn pose_transform_checks_frames_and_values() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let pose = [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let t = pose_transform(&pose, "odom", "base_footprint", &f).unwrap();
    assert_eq!((t.src(), t.dst()), ("base_footprint", "odom"));
    assert!(pose_transform(&pose, "base_link", "rslidar", &f).is_err()); // mounted parent
    assert!(pose_transform(&pose, "odom", "none", &f).is_err());
    assert!(
        pose_transform(
            &[1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            "odom",
            "base_footprint",
            &f
        )
        .is_err()
    );
    assert!(pose_transform(&[1.0, 2.0, 0.0], "odom", "base_footprint", &f).is_err());
    assert!(
        pose_transform(
            &[f64::NAN, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "odom",
            "base_footprint",
            &f
        )
        .is_err()
    );
}

#[test]
fn resolve_upwards_with_pose() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let pose = pose_transform(
        &[1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "odom",
        "base_footprint",
        &f,
    )
    .unwrap();
    let (t, used) = resolve(&ex(), "rslidar", "odom", Some(&pose)).unwrap();
    assert!(used);
    assert!(close(t.unwrap().apply([0.0, 0.0, 0.0]), [1.25, 2.0, 0.35]));
}

#[test]
fn resolve_downwards_uses_inverse_pose() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let pose = pose_transform(
        &[1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "odom",
        "base_footprint",
        &f,
    )
    .unwrap();
    let (t, used) = resolve(&ex(), "odom", "base_link", Some(&pose)).unwrap();
    assert!(used);
    assert!(close(t.unwrap().apply([1.0, 2.0, 0.1]), [0.0, 0.0, 0.0]));
}

#[test]
fn resolve_static_ignores_pose_and_none_without_pose() {
    let (t, used) = resolve(&ex(), "rslidar", "base_link", None).unwrap();
    assert!(!used && t.is_some());
    let (t, used) = resolve(&ex(), "rslidar", "odom", None).unwrap();
    assert!(!used && t.is_none());
}

#[test]
fn resolve_refuses_a_second_dynamic_edge() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let pose = pose_transform(
        &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "odom",
        "base_footprint",
        &f,
    )
    .unwrap();
    assert!(matches!(
        resolve(&ex(), "rslidar", "map", Some(&pose)),
        Err(FrameError::DynamicEdge { .. })
    ));
}

#[test]
fn apply_packed_moves_xyz_and_keeps_intensity() {
    let t = Transform::from_parts("a", "b", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
    let pts: Vec<f32> = vec![0.0, 0.0, 0.0, 7.0, 1.0, 1.0, 1.0, 9.0];
    let out = t.apply_packed(&pts, 4).unwrap();
    assert_eq!(out, vec![1.0, 0.0, 0.0, 7.0, 2.0, 1.0, 1.0, 9.0]);
    assert!(t.apply_packed(&pts[..7], 4).is_err());
    assert!(t.apply_packed(&pts, 2).is_err());
}

fn rz90(src: &str, dst: &str, t: [f64; 3]) -> Transform {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    Transform::from_parts(src, dst, t, [0.0, 0.0, s, s]).unwrap()
}

#[test]
fn lookup_pins_camera_geometry() {
    let t = lookup(&ex(), "rslidar", "camera_front").unwrap();
    assert!(close(t.apply([0.0; 3]), [0.0, 0.1, -0.15]));
    let t = lookup(&ex(), "camera_front", "base_link").unwrap();
    assert!(close(t.apply([0.0, 0.0, 1.0]), [1.4, 0.0, 0.35]));
}

#[test]
fn then_order_is_self_first() {
    let a = rz90("p", "q", [0.0; 3]);
    let b = Transform::from_parts("q", "q", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
    // rotate (1,0,0) to (0,1,0), then shift +x
    assert!(close(
        a.then(&b).unwrap().apply([1.0, 0.0, 0.0]),
        [1.0, 1.0, 0.0]
    ));
    // shift first, then rotate: (2,0,0) -> (0,2,0)
    let b2 = Transform::from_parts("p", "p", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
    let a2 = rz90("p", "q", [0.0; 3]);
    assert!(close(
        b2.then(&a2).unwrap().apply([1.0, 0.0, 0.0]),
        [0.0, 2.0, 0.0]
    ));
}

#[test]
fn resolve_with_rotated_pose() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let pose = pose_transform(
        &[1.0, 2.0, 0.0, 0.0, 0.0, s, s],
        "odom",
        "base_footprint",
        &f,
    )
    .unwrap();
    let (t, used) = resolve(&ex(), "rslidar", "odom", Some(&pose)).unwrap();
    assert!(used);
    assert!(close(t.unwrap().apply([0.0; 3]), [1.0, 2.25, 0.35]));
    let (t, used) = resolve(&ex(), "odom", "base_link", Some(&pose)).unwrap();
    assert!(used);
    assert!(close(t.unwrap().apply([1.0, 3.0, 0.1]), [1.0, 0.0, 0.0]));
}

#[test]
fn inverse_of_rotated_translated() {
    let t = rz90("a", "b", [1.0, 2.0, 3.0]);
    let inv = t.inverse();
    let p = [0.3, -0.7, 2.0];
    assert!(close(inv.apply(t.apply(p)), p));
    let m = t.matrix();
    let n = inv.matrix();
    for i in 0..3 {
        for j in 0..3 {
            assert!((n[i][j] - m[j][i]).abs() < 1e-12);
        }
        let want = -(0..3).map(|k| m[k][i] * m[k][3]).sum::<f64>();
        assert!((n[i][3] - want).abs() < 1e-12);
    }
}

#[test]
fn resolve_propagates_other_errors() {
    assert!(matches!(
        resolve(&ex(), "nope", "odom", None),
        Err(FrameError::UnknownFrame(_))
    ));
    // drop the rslidar edge from the calibration
    let text = EXTRINSICS.replace(
        "  - parent: base_link\n    child: rslidar\n    translation_m: [0.25, 0.0, 0.25]\n    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]\n",
        "",
    );
    assert_ne!(text, EXTRINSICS);
    let frames = FrameDict::parse(FRAMES).unwrap();
    {
        let ex2 = Extrinsics::parse(&text, &frames, None).expect("parse");
        let r = resolve(&ex2, "rslidar", "base_link", None);
        assert!(matches!(r, Err(FrameError::MissingExtrinsic(_))), "{r:?}");
    }
}

#[test]
fn near_unit_quaternions_still_resolve() {
    let k = 1.0 + 9.9e-7;
    let text = EXTRINSICS.replace(
        "[-0.5, 0.5, -0.5, 0.5]",
        &format!(
            "[{:?}, {:?}, {:?}, {:?}]",
            -0.5 * k,
            0.5 * k,
            -0.5 * k,
            0.5 * k
        ),
    );
    assert_ne!(text, EXTRINSICS);
    let f = FrameDict::parse(FRAMES).unwrap();
    let ex = Extrinsics::parse(&text, &f, None).unwrap();
    let q = [0.04502, -0.33413, -0.89433, -0.29411];
    let n = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    let sc = k / n;
    let v = [0.5, 0.5, 0.0, q[0] * sc, q[1] * sc, q[2] * sc, q[3] * sc];
    let pose = pose_transform(&v, "odom", "base_footprint", &f).unwrap();
    let (t, used) = resolve(&ex, "camera_front", "odom", Some(&pose)).unwrap();
    assert!(used && t.is_some());
}

#[test]
fn degenerate_inputs_are_refused() {
    assert!(Transform::from_parts("a", "b", [0.0; 3], [0.0; 4]).is_err());
    let f = FrameDict::parse(FRAMES).unwrap();
    let id = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    assert!(pose_transform(&id, "odom", "odom", &f).is_err());
}

// Parity with the reference suite (docs/contract-core/test-mapping.md).
// Expected numbers are worked by hand, as in the reference.

use dora_transform::rigid::rotation_matrix;

const CAMERA_Q: [f64; 4] = [-0.5, 0.5, -0.5, 0.5];

fn mat_close(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> bool {
    a.iter()
        .flatten()
        .zip(b.iter().flatten())
        .all(|(x, y)| (x - y).abs() < 1e-12)
}

#[test]
fn camera_quaternion_gives_the_optical_axes_as_columns() {
    let want = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]];
    assert!(mat_close(rotation_matrix(CAMERA_Q), want));
}

#[test]
fn identity_quaternion_is_xyzw_not_wxyz() {
    let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(mat_close(rotation_matrix([0.0, 0.0, 0.0, 1.0]), eye));
}

#[test]
fn yaw_plus_translation_moves_a_point_where_the_hand_says() {
    let t = rz90("a", "b", [1.0, 2.0, 3.0]);
    assert!(close(t.apply([1.0, 0.0, 0.0]), [1.0, 3.0, 3.0]));
}

#[test]
fn a_transform_and_its_inverse_compose_to_identity_frames() {
    let t = Transform::from_parts("a", "b", [1.0, 2.0, 3.0], CAMERA_Q).unwrap();
    let back = t.then(&t.inverse()).unwrap();
    assert_eq!((back.src(), back.dst()), ("a", "a"));
    let id = Transform::identity("a");
    for (x, y) in back
        .matrix()
        .iter()
        .flatten()
        .zip(id.matrix().iter().flatten())
    {
        assert!((x - y).abs() < 1e-12);
    }
}

#[test]
fn then_refusal_says_the_frames_do_not_meet() {
    let a = Transform::identity("a");
    let c = Transform::from_parts("c", "d", [0.0; 3], [0.0, 0.0, 0.0, 1.0]).unwrap();
    assert!(a.then(&c).unwrap_err().to_string().contains("do not meet"));
}

#[test]
fn a_non_finite_matrix_is_refused() {
    assert!(Transform::new("a", "b", [[f64::NAN; 4]; 4]).is_err());
    let mut m = *Transform::identity("a").matrix();
    m[0][0] = 2.0;
    m[1][1] = 2.0;
    m[2][2] = 2.0;
    assert!(
        Transform::new("a", "b", m)
            .unwrap_err()
            .to_string()
            .contains("rotation")
    );
}

#[test]
fn camera_and_lidar_points_land_where_the_hand_says() {
    let t = lookup(&ex(), "camera_front", "base_link").unwrap();
    assert!(close(t.apply([0.0, 0.0, 1.0]), [1.40, 0.0, 0.35]));
    assert!(close(t.apply([1.0, 0.0, 0.0]), [0.40, -1.0, 0.35]));
    // (1,2,3) in rslidar -> (1.25, 2, 3.25) in base_link -> minus the camera
    // origin -> optical axes x=-y, y=-z, z=x -> (-2, -2.9, 0.85).
    let t = lookup(&ex(), "rslidar", "camera_front").unwrap();
    assert!(close(t.apply([1.0, 2.0, 3.0]), [-2.0, -2.9, 0.85]));
}

#[test]
fn lookup_in_reverse_is_the_inverse() {
    let forward = lookup(&ex(), "rslidar", "camera_front").unwrap();
    let backward = lookup(&ex(), "camera_front", "rslidar").unwrap();
    let inv = backward.inverse();
    for (x, y) in forward
        .matrix()
        .iter()
        .flatten()
        .zip(inv.matrix().iter().flatten())
    {
        assert!((x - y).abs() < 1e-12);
    }
}

#[test]
fn a_pose_maps_its_child_into_its_frame() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let t = pose_transform(
        &[1.0, 2.0, 0.0, 0.0, 0.0, s, s],
        "odom",
        "base_footprint",
        &f,
    )
    .unwrap();
    assert_eq!((t.src(), t.dst()), ("base_footprint", "odom"));
    assert!(close(t.apply([1.0, 0.0, 0.0]), [1.0, 3.0, 0.0]));
}

#[test]
fn bad_pose_messages() {
    let f = FrameDict::parse(FRAMES).unwrap();
    let id = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    for (values, parent, child, want) in [
        (&id[..], "base_link", "rslidar", "fixed or dynamic"),
        (&id[..], "odom", "none", "never part of a transform"),
        (&id[..6], "odom", "base_link", "seven"),
        (
            &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5][..],
            "odom",
            "base_link",
            "unit quaternion",
        ),
        (
            &[0.0, 0.0, f64::INFINITY, 0.0, 0.0, 0.0, 1.0][..],
            "odom",
            "base_link",
            "seven finite",
        ),
        (&id[..], "world", "base_link", "use 'map'"),
    ] {
        let err = pose_transform(values, parent, child, &f)
            .unwrap_err()
            .to_string();
        assert!(err.contains(want), "{parent}/{child}: {err}");
    }
}

#[test]
fn a_forward_pose_route_applies_a_non_trivial_lookup_after_the_pose() {
    // A mounted `dock` under odom, 5 m along x and turned 180 degrees.
    let frames = FrameDict::parse(&format!("{FRAMES}\n").replace(
        "rejected_names:",
        "  - {name: dock, parent: odom, kind: mounted}\nrejected_names:",
    ))
    .unwrap();
    let text = format!(
        "{EXTRINSICS}  - parent: odom\n    child: dock\n    translation_m: [5.0, 0.0, 0.0]\n    rotation_xyzw: [0.0, 0.0, 1.0, 0.0]\n"
    );
    let calib = Extrinsics::parse(&text, &frames, None).unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let pose = pose_transform(
        &[1.0, 2.0, 0.0, 0.0, 0.0, s, s],
        "odom",
        "base_footprint",
        &frames,
    )
    .unwrap();
    let (t, used) = resolve(&calib, "rslidar", "dock", Some(&pose)).unwrap();
    assert!(used);
    // (0,0,0) in rslidar -> (0.25, 0, 0.35) in base_footprint -> (1, 2.25, 0.35)
    // in odom -> R180^T ((1, 2.25, 0.35) - (5, 0, 0)) = (4, -2.25, 0.35) in dock.
    assert!(close(t.unwrap().apply([0.0; 3]), [4.0, -2.25, 0.35]));
}
