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
    let mut m = Transform::identity("a").matrix;
    m[0][0] = 2.0; // scale
    assert!(Transform::new("a", "b", m).is_err());
    let mut r = Transform::identity("a").matrix;
    r[0][0] = -1.0; // reflection
    assert!(Transform::new("a", "b", r).is_err());
    let mut last = Transform::identity("a").matrix;
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
    assert_eq!((t.src.as_str(), t.dst.as_str()), ("base_footprint", "odom"));
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
