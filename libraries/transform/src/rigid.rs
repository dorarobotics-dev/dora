use crate::extrinsics::Edge;
use crate::frames::{FrameDict, Kind};
use crate::{Extrinsics, FrameError, QUATERNION_NORM_TOLERANCE, Result};

pub type Matrix = [[f64; 4]; 4];

fn mul(a: &Matrix, b: &Matrix) -> Matrix {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    out
}

/// A 3x3 rotation from a unit quaternion in x, y, z, w order.
pub fn rotation_matrix(q: [f64; 4]) -> [[f64; 3]; 3] {
    let [x, y, z, w] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Maps coordinates expressed in `src` to coordinates expressed in `dst`.
#[derive(Debug, Clone, PartialEq)]
pub struct Transform {
    pub src: String,
    pub dst: String,
    pub matrix: Matrix,
}

impl Transform {
    pub fn new(src: impl Into<String>, dst: impl Into<String>, m: Matrix) -> Result<Self> {
        if m.iter().flatten().any(|v| !v.is_finite()) {
            return Err(FrameError::Invalid(
                "a transform is a finite 4x4 matrix".into(),
            ));
        }
        let last = [0.0, 0.0, 0.0, 1.0];
        if m[3].iter().zip(last).any(|(a, b)| (a - b).abs() > 1e-8) {
            return Err(FrameError::Invalid(
                "the last row of a rigid transform must be 0 0 0 1".into(),
            ));
        }
        // R^T R == I and det(R) > 0
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3).map(|k| m[k][i] * m[k][j]).sum();
                let want = if i == j { 1.0 } else { 0.0 };
                if (dot - want).abs() > 1e-5 {
                    return Err(FrameError::Invalid("the rotation block of a rigid transform must be orthonormal with a positive determinant (no scale, shear or reflection)".into()));
                }
            }
        }
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if det <= 0.0 {
            return Err(FrameError::Invalid("the rotation block of a rigid transform must be orthonormal with a positive determinant (no scale, shear or reflection)".into()));
        }
        Ok(Transform {
            src: src.into(),
            dst: dst.into(),
            matrix: m,
        })
    }

    pub fn identity(frame: &str) -> Self {
        let mut m = [[0.0; 4]; 4];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        Transform {
            src: frame.into(),
            dst: frame.into(),
            matrix: m,
        }
    }

    pub fn from_parts(src: &str, dst: &str, t: [f64; 3], q_xyzw: [f64; 4]) -> Result<Self> {
        let r = rotation_matrix(q_xyzw);
        let mut m = [[0.0; 4]; 4];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = r[i][j];
            }
            m[i][3] = t[i];
        }
        m[3][3] = 1.0;
        Self::new(src, dst, m)
    }

    /// Apply `self`, then `other`.
    pub fn then(&self, other: &Transform) -> Result<Transform> {
        if self.dst != other.src {
            return Err(FrameError::FrameOrder(format!(
                "cannot apply '{}' -> '{}' after '{}' -> '{}'; the frames do not meet",
                other.src, other.dst, self.src, self.dst
            )));
        }
        Transform::new(
            self.src.clone(),
            other.dst.clone(),
            mul(&other.matrix, &self.matrix),
        )
    }

    pub fn inverse(&self) -> Transform {
        let m = &self.matrix;
        let mut out = [[0.0; 4]; 4];
        for i in 0..3 {
            for j in 0..3 {
                out[i][j] = m[j][i];
            }
            out[i][3] = -(0..3).map(|k| m[k][i] * m[k][3]).sum::<f64>();
        }
        out[3][3] = 1.0;
        Transform {
            src: self.dst.clone(),
            dst: self.src.clone(),
            matrix: out,
        }
    }

    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let m = &self.matrix;
        let mut out = [0.0; 3];
        for i in 0..3 {
            out[i] = m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3];
        }
        out
    }

    /// Transform packed float32 points of `item_step` floats each; only the
    /// first three (x, y, z) move, the rest pass through.
    pub fn apply_packed(&self, points: &[f32], item_step: usize) -> Result<Vec<f32>> {
        if item_step < 3 {
            return Err(FrameError::Invalid(format!(
                "item_step must be >= 3, got {item_step}"
            )));
        }
        if !points.len().is_multiple_of(item_step) {
            return Err(FrameError::Invalid(format!(
                "{} floats is not a whole number of {item_step}-float points",
                points.len()
            )));
        }
        let mut out = points.to_vec();
        for chunk in out.chunks_exact_mut(item_step) {
            let p = self.apply([chunk[0] as f64, chunk[1] as f64, chunk[2] as f64]);
            chunk[0] = p[0] as f32;
            chunk[1] = p[1] as f32;
            chunk[2] = p[2] as f32;
        }
        Ok(out)
    }
}

/// child -> parent: an edge stores the child's pose in its parent.
pub fn edge_transform(edge: &Edge) -> Result<Transform> {
    Transform::from_parts(
        &edge.child,
        &edge.parent,
        edge.translation_m,
        edge.rotation_xyzw,
    )
}

/// The static transform from `src` to `dst`. Errors as `Extrinsics::route` does.
pub fn lookup(ex: &Extrinsics, src: &str, dst: &str) -> Result<Transform> {
    let route = ex.route(src, dst)?;
    let mut result = Transform::identity(src);
    for edge in &route.up {
        result = result.then(&edge_transform(edge)?)?;
    }
    for edge in &route.down {
        result = result.then(&edge_transform(edge)?.inverse())?;
    }
    Ok(result)
}

/// `child_frame -> parent_frame` from one pose: px py pz qx qy qz qw.
pub fn pose_transform(
    values: &[f64],
    parent_frame: &str,
    child_frame: &str,
    frames: &FrameDict,
) -> Result<Transform> {
    let parent = frames.require(parent_frame)?;
    if !matches!(parent.kind, Kind::Fixed | Kind::Dynamic) {
        return Err(FrameError::Invalid(format!(
            "a pose is expressed in a fixed or dynamic frame, not the {} frame '{}'",
            parent.kind.as_str(),
            parent.name
        )));
    }
    if frames.require(child_frame)?.kind == Kind::None {
        return Err(FrameError::UnknownFrame(
            "frame 'none' marks a payload with no spatial meaning and is never part of a transform"
                .into(),
        ));
    }
    if values.len() != 7 || values.iter().any(|v| !v.is_finite()) {
        return Err(FrameError::Invalid(format!(
            "a pose payload is seven finite values px py pz qx qy qz qw, got {values:?}"
        )));
    }
    let q = [values[3], values[4], values[5], values[6]];
    let norm = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    if (norm - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
        return Err(FrameError::Invalid(format!(
            "the pose orientation has norm {norm}; it must be a unit quaternion"
        )));
    }
    Transform::from_parts(
        child_frame,
        parent_frame,
        [values[0], values[1], values[2]],
        q,
    )
}

fn through(ex: &Extrinsics, src: &str, dst: &str, moving: &Transform) -> Result<Option<Transform>> {
    let legs = lookup(ex, src, &moving.src)
        .and_then(|a| a.then(moving))
        .and_then(|a| Ok((a, lookup(ex, &moving.dst, dst)?)))
        .and_then(|(a, b)| a.then(&b));
    match legs {
        Ok(t) => Ok(Some(t)),
        Err(FrameError::DynamicEdge { .. }) => Ok(None),
        Err(e) => Err(e),
    }
}

/// The transform for one cloud, and whether it used the pose. A static path
/// never consults the pose; otherwise exactly one dynamic edge, the pose's, is
/// crossed in either direction. With no pose yet there is no transform.
pub fn resolve(
    ex: &Extrinsics,
    src: &str,
    dst: &str,
    pose: Option<&Transform>,
) -> Result<(Option<Transform>, bool)> {
    match lookup(ex, src, dst) {
        Ok(t) => return Ok((Some(t), false)),
        Err(FrameError::DynamicEdge { .. }) => {}
        Err(e) => return Err(e),
    }
    let Some(moving) = pose else {
        return Ok((None, false));
    };
    for candidate in [moving.clone(), moving.inverse()] {
        if let Some(t) = through(ex, src, dst, &candidate)? {
            return Ok((Some(t), true));
        }
    }
    Err(FrameError::DynamicEdge {
        message: format!(
            "'{src}' -> '{dst}' cannot be composed with the pose of '{}' in '{}': the route needs a dynamic edge that pose does not give, and one pose covers one dynamic edge",
            moving.src, moving.dst
        ),
        edges: vec![],
    })
}
