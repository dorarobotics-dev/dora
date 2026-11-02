use crate::frames::{FrameDict, Kind};
use crate::{FrameError, QUATERNION_NORM_TOLERANCE, Result};
use serde_yaml::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// One calibrated mounting: the pose of `child` expressed in `parent`.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub parent: String,
    pub child: String,
    pub translation_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

/// `up` climbs from `src` to the common ancestor (child -> parent);
/// `down` descends from it to `dst` (parent -> child, applied inverted).
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    pub src: String,
    pub dst: String,
    pub up: Vec<Edge>,
    pub down: Vec<Edge>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Extrinsics {
    robot: String,
    source: String,
    frames: FrameDict,
    edges: BTreeMap<String, Edge>,
    path: Option<PathBuf>,
}

const REQUIRED: [&str; 5] = [
    "contract_version",
    "robot",
    "calibrated",
    "source",
    "transforms",
];

fn numbers<const N: usize>(
    value: Option<&Value>,
    label: &str,
    where_: &str,
) -> Result<[f64; N]> {
    let Some(seq) = value.and_then(Value::as_sequence).filter(|s| s.len() == N) else {
        return Err(FrameError::Invalid(format!(
            "{where_}: {label} must be a list of {N} numbers, got {value:?}"
        )));
    };
    let mut out = [0.0; N];
    for (i, item) in seq.iter().enumerate() {
        let v = match item {
            Value::Number(n) => n.as_f64(),
            _ => None,
        };
        match v {
            Some(v) if v.is_finite() => out[i] = v,
            _ => {
                return Err(FrameError::Invalid(format!(
                    "{where_}: {label} holds {item:?}, which is not a finite number"
                )));
            }
        }
    }
    Ok(out)
}

impl Extrinsics {
    pub fn load(path: &Path, frames: &FrameDict) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| FrameError::Invalid(format!("{}: {e}", path.display())))?;
        Self::parse(&text, frames, Some(path.to_path_buf()))
    }

    pub fn parse(text: &str, frames: &FrameDict, path: Option<PathBuf>) -> Result<Self> {
        let where_ = path
            .as_ref()
            .map_or("extrinsics".to_string(), |p| p.display().to_string());
        let doc: Value = serde_yaml::from_str(text)
            .map_err(|e| FrameError::Invalid(format!("{where_}: {e}")))?;
        if !doc.is_mapping() {
            return Err(FrameError::Invalid(format!("{where_}: must be a mapping")));
        }
        for key in REQUIRED {
            if doc.get(key).is_none() {
                return Err(FrameError::Invalid(format!(
                    "{where_}: missing the required key '{key}'"
                )));
            }
        }
        let Some(entries) = doc["transforms"].as_sequence().filter(|s| !s.is_empty()) else {
            return Err(FrameError::Invalid(format!(
                "{where_}: 'transforms' must be a non-empty list"
            )));
        };
        let mut edges = BTreeMap::new();
        for entry in entries {
            let parent = entry.get("parent").and_then(Value::as_str);
            let child = entry.get("child").and_then(Value::as_str);
            let (Some(parent), Some(child)) = (parent, child) else {
                return Err(FrameError::Invalid(format!(
                    "{where_}: parent and child must be frame names, got {entry:?}"
                )));
            };
            let label = format!("{where_}: {parent} -> {child}");
            frames.require(parent)?;
            let frame = frames.require(child)?;
            if frame.kind != Kind::Mounted {
                return Err(FrameError::Invalid(format!(
                    "{label}: '{child}' is {}, and only mounted frames are calibrated; a dynamic edge comes from a published pose",
                    frame.kind.as_str()
                )));
            }
            if frame.parent.as_deref() != Some(parent) {
                return Err(FrameError::Invalid(format!(
                    "{label}: the frame dictionary says '{child}' hangs from {:?}",
                    frame.parent
                )));
            }
            let translation_m = numbers::<3>(entry.get("translation_m"), "translation_m", &label)?;
            let rotation_xyzw = numbers::<4>(entry.get("rotation_xyzw"), "rotation_xyzw", &label)?;
            let norm = rotation_xyzw.iter().map(|c| c * c).sum::<f64>().sqrt();
            if (norm - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
                return Err(FrameError::Invalid(format!(
                    "{label}: rotation_xyzw has norm {norm}; it must be a unit quaternion, and it is not normalised for you"
                )));
            }
            if edges.contains_key(child) {
                return Err(FrameError::Invalid(format!(
                    "{where_}: '{child}' is calibrated more than once"
                )));
            }
            edges.insert(
                child.to_string(),
                Edge {
                    parent: parent.to_string(),
                    child: child.to_string(),
                    translation_m,
                    rotation_xyzw,
                },
            );
        }
        let text_of = |key: &str| {
            doc[key]
                .as_str()
                .map(str::to_string)
                .ok_or_else(|| FrameError::Invalid(format!("{where_}: {key} must be a string")))
        };
        Ok(Extrinsics {
            robot: text_of("robot")?,
            source: text_of("source")?,
            frames: frames.clone(),
            edges,
            path,
        })
    }

    pub fn frames(&self) -> &FrameDict {
        &self.frames
    }

    pub fn edges(&self) -> &BTreeMap<String, Edge> {
        &self.edges
    }

    pub fn robot(&self) -> &str {
        &self.robot
    }

    pub fn source(&self) -> &str {
        &self.source
    }

    pub fn path(&self) -> Option<&Path> {
        self.path.as_deref()
    }

    pub fn route(&self, src: &str, dst: &str) -> Result<Route> {
        for name in [src, dst] {
            if self.frames.require(name)?.kind == Kind::None {
                return Err(FrameError::UnknownFrame(format!(
                    "frame '{name}' marks a payload with no spatial meaning and is never part of a transform"
                )));
            }
        }
        let up_chain = self.frames.chain(src)?;
        let down_chain = self.frames.chain(dst)?;
        let Some(common) = up_chain.iter().find(|n| down_chain.contains(n)).cloned() else {
            return Err(FrameError::MissingExtrinsic(format!(
                "'{src}' and '{dst}' share no ancestor in the frame tree"
            )));
        };
        // Walk the whole route before refusing it: a wired pose can stand in for
        // a dynamic edge but never for a missing calibration, so a missing one wins.
        let mut dynamic = Vec::new();
        let up_children = &up_chain[..up_chain.iter().position(|n| *n == common).unwrap()];
        let down_children = &down_chain[..down_chain.iter().position(|n| *n == common).unwrap()];
        let up = self.edges_above(up_children, src, dst, &mut dynamic)?;
        let mut down = self.edges_above(down_children, src, dst, &mut dynamic)?;
        if let Some(first) = dynamic.first() {
            let parent = self.frames.frames()[first].parent.clone().unwrap_or_default();
            return Err(FrameError::DynamicEdge {
                message: format!(
                    "'{src}' -> '{dst}' crosses the dynamic edge '{parent}' -> '{first}'; compose with a published pose instead of a static lookup"
                ),
                edges: dynamic
                    .iter()
                    .map(|c| {
                        (
                            c.clone(),
                            self.frames.frames()[c].parent.clone().unwrap_or_default(),
                        )
                    })
                    .collect(),
            });
        }
        down.reverse();
        Ok(Route {
            src: src.into(),
            dst: dst.into(),
            up,
            down,
        })
    }

    fn edges_above(
        &self,
        children: &[String],
        src: &str,
        dst: &str,
        dynamic: &mut Vec<String>,
    ) -> Result<Vec<Edge>> {
        let mut out = Vec::new();
        for child in children {
            let frame = &self.frames.frames()[child];
            if frame.kind == Kind::Dynamic {
                dynamic.push(child.clone());
                continue;
            }
            match self.edges.get(child) {
                Some(edge) => out.push(edge.clone()),
                None => {
                    return Err(FrameError::MissingExtrinsic(format!(
                        "'{src}' -> '{dst}' needs the extrinsic '{}' -> '{child}', which '{}' does not calibrate; a missing extrinsic is never assumed to be identity",
                        frame.parent.as_deref().unwrap_or(""),
                        self.robot
                    )));
                }
            }
        }
        Ok(out)
    }
}
