//! Pre-launch frame checks, run by `dora validate` when the dataflow has a
//! `frames:` key.
//!
//! Ported from the operator contract's validator: the per-edge frame check
//! and check 6 (a frame change needs a calibration that supports it), with
//! frame resolution as in its `resolve_frame`. The declarations come from
//! the nodes' `output_frames`/`input_frames` instead of an operators file.

use crate::descriptor::{Descriptor, normalize_path};
use dora_message::{
    config::{InputMapping, UserInputMapping},
    descriptor::{EnvValue, Node},
};
use dora_transform::{Extrinsics, FrameDict, FrameError, Kind};
use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

/// The input type a frame-changing node composes with across a dynamic edge.
pub const POSE_TYPE: &str = "std/math/v1/Pose";
/// The env var a node loads its calibration from.
pub const EXTRINSICS_ENV: &str = "DORA_EXTRINSICS";

/// Frames that carry no spatial meaning, so no transform starts or ends there.
const SPATIALLESS: [&str; 2] = ["any", "none"];

#[derive(Debug, Clone, PartialEq)]
pub struct FrameViolation {
    /// `unknown_frame`, `frame` or `transform`.
    pub check: &'static str,
    pub node: String,
    pub port: String,
    pub message: String,
}

impl fmt::Display for FrameViolation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}] {}/{}: {}",
            self.check, self.node, self.port, self.message
        )
    }
}

struct Ctx<'a> {
    nodes: BTreeMap<String, &'a Node>,
    global_env: Option<&'a BTreeMap<String, EnvValue>>,
}

impl<'a> Ctx<'a> {
    /// A variable as the node would see it at launch: its own `env` over the
    /// dataflow-level `env`. Trimmed; an empty value counts as unset.
    fn env(&self, node: &Node, var: &str) -> Option<String> {
        node.env
            .as_ref()
            .and_then(|e| e.get(var))
            .or_else(|| self.global_env?.get(var))
            .map(|v| v.to_string().trim().to_string())
            .filter(|v| !v.is_empty())
    }

    /// The frame `node_id`'s output `port` resolves to, following `same_as:`
    /// upstream and `env:` from the node's env. An output with no
    /// `output_frames` entry is `any`.
    fn resolve(&self, node_id: &str, port: &str) -> Result<String, String> {
        self.resolve_seen(node_id, port, &mut Vec::new())
    }

    fn resolve_seen(
        &self,
        node_id: &str,
        port: &str,
        seen: &mut Vec<(String, String)>,
    ) -> Result<String, String> {
        let here = (node_id.to_string(), port.to_string());
        if seen.contains(&here) {
            let path: Vec<String> = seen
                .iter()
                .chain([&here])
                .map(|(n, p)| format!("{n}.{p}"))
                .collect();
            return Err(format!("frame resolution cycle: {}", path.join(" -> ")));
        }
        seen.push(here);
        let node = self
            .nodes
            .get(node_id)
            .ok_or_else(|| format!("node '{node_id}' is not in the dataflow"))?;
        let Some(declared) = node.output_frames.get(port).map(String::as_str) else {
            return Ok("any".into());
        };
        if let Some(target) = declared.strip_prefix("same_as:") {
            let up = upstream(node, target).ok_or_else(|| {
                format!(
                    "{node_id}.{target} is unwired, so {node_id}.{port} cannot resolve same_as:{target}"
                )
            })?;
            return self.resolve_seen(up.source.as_ref(), up.output.as_str(), seen);
        }
        if let Some(var) = declared.strip_prefix("env:") {
            let value = self.env(node, var).ok_or_else(|| {
                format!(
                    "{node_id}.{port} declares env:{var}, but node '{node_id}' sets no {var} in its env"
                )
            })?;
            if !is_frame_name(&value) {
                return Err(format!(
                    "{node_id}.{port} declares env:{var}, and node '{node_id}' sets {var} to '{value}', which is not a frame; an env frame must name one frame, never 'any', 'none', same_as: or env:"
                ));
            }
            return Ok(value);
        }
        Ok(declared.to_string())
    }
}

fn is_frame_name(value: &str) -> bool {
    !SPATIALLESS.contains(&value) && !value.starts_with("same_as:") && !value.starts_with("env:")
}

/// The node output that feeds `node`'s input `input`, if another node
/// produces it. Timers and log subscriptions are not node edges.
fn upstream<'n>(node: &'n Node, input: &str) -> Option<&'n UserInputMapping> {
    let (_, input) = node
        .inputs
        .iter()
        .find(|(k, _)| k.as_str() as &str == input)?;
    match &input.mapping {
        InputMapping::User(m) => Some(m),
        InputMapping::Timer { .. } | InputMapping::Logs(_) => None,
    }
}

/// Run the frame checks. A dataflow without a `frames:` key has none.
/// `base_dir` is the dataflow file's directory; `frames:` paths and a node's
/// `DORA_EXTRINSICS` resolve against it, as dora runs nodes from there.
pub fn check_frames(dataflow: &Descriptor, base_dir: &Path) -> Vec<FrameViolation> {
    let Some(cfg) = &dataflow.frames else {
        return vec![];
    };
    let mut out = Vec::new();
    let mut push = |check: &'static str, node: &str, port: &str, message: String| {
        out.push(FrameViolation {
            check,
            node: node.into(),
            port: port.into(),
            message,
        });
    };

    let frames = match FrameDict::load(&base_dir.join(&cfg.dictionary)) {
        Ok(f) => f,
        Err(e) => {
            push("transform", "-", "frames.dictionary", e.to_string());
            return out;
        }
    };
    let ex_path = base_dir.join(&cfg.extrinsics);
    let extrinsics = match Extrinsics::load(&ex_path, &frames) {
        Ok(e) => Some(e),
        Err(e) => {
            push("transform", "-", "frames.extrinsics", e.to_string());
            None
        }
    };
    let ctx = Ctx {
        nodes: dataflow
            .nodes
            .iter()
            .map(|n| (n.id.to_string(), n))
            .collect(),
        global_env: dataflow.env.as_ref(),
    };
    let spatialless = |frame: &str| {
        SPATIALLESS.contains(&frame) || frames.get(frame).is_some_and(|f| f.kind == Kind::None)
    };

    // 1. unknown_frame: every literal frame is in the dictionary. Env-resolved
    //    output frames are checked with the node's outputs below.
    for node in &dataflow.nodes {
        let nid = node.id.to_string();
        let outputs = node.output_frames.iter().filter(|(_, f)| {
            f.as_str() != "any" && !f.starts_with("same_as:") && !f.starts_with("env:")
        });
        let inputs = node
            .input_frames
            .iter()
            .filter(|(_, f)| f.as_str() != "any");
        for (port, f) in outputs.chain(inputs) {
            if let Err(e) = frames.require(f) {
                push("unknown_frame", &nid, port.as_str(), e.to_string());
            }
        }
    }

    // 2. frame: an input that requires a frame gets it from its upstream.
    for node in &dataflow.nodes {
        let nid = node.id.to_string();
        for (port, want) in &node.input_frames {
            if want == "any" {
                continue;
            }
            let Some(up) = upstream(node, port.as_str()) else {
                continue;
            };
            let (un, uo) = (up.source.to_string(), up.output.to_string());
            match ctx.resolve(&un, &uo) {
                Err(e) => push("frame", &nid, port.as_str(), e),
                Ok(actual) if actual == "any" => push(
                    "frame",
                    &nid,
                    port.as_str(),
                    format!(
                        "requires frame '{want}' but {un}/{uo} declares 'any', which cannot be checked before launch"
                    ),
                ),
                Ok(actual) if &actual != want => push(
                    "frame",
                    &nid,
                    port.as_str(),
                    format!("requires frame '{want}' but {un}/{uo} resolves to '{actual}'"),
                ),
                Ok(_) => {}
            }
        }
    }

    // 3-5. transform: a frame change needs a calibration that supports it.
    for node in &dataflow.nodes {
        let nid = node.id.to_string();
        let is_pose = |port: &str| {
            node.input_types
                .iter()
                .any(|(k, t)| k.as_str() == port && t == POSE_TYPE)
        };

        // The frames this node's data inputs resolve to. A pose input is what
        // a transform composes with, not data it moves, so it is skipped.
        let mut inputs = Vec::new();
        for port in node.inputs.keys() {
            let p = port.as_str();
            if is_pose(p) {
                continue;
            }
            let Some(up) = upstream(node, p) else {
                continue;
            };
            // An unresolvable frame is reported by check 2 or below.
            let Ok(frame) = ctx.resolve(up.source.as_ref(), up.output.as_str()) else {
                continue;
            };
            // An unknown frame is check 1's violation (or the upstream's env
            // check below), not a route to check.
            if spatialless(&frame) || frames.require(&frame).is_err() {
                continue;
            }
            let want = node.input_frames.get(port).map_or("any", String::as_str);
            if want != "any" && want != frame {
                continue; // a frame the input refuses is check 2's violation, not a transform
            }
            inputs.push(frame);
        }

        let mut changes = Vec::new();
        for (port, declared) in &node.output_frames {
            if declared.starts_with("same_as:") {
                continue;
            }
            let p = port.as_str();
            let out_frame = match ctx.resolve(&nid, p) {
                Ok(f) => f,
                Err(e) => {
                    if declared.starts_with("env:") {
                        push("frame", &nid, p, e);
                    }
                    continue;
                }
            };
            if spatialless(&out_frame) {
                continue;
            }
            if let Err(e) = frames.require(&out_frame) {
                // A literal frame was reported by check 1.
                if declared.starts_with("env:") {
                    push("unknown_frame", &nid, p, e.to_string());
                }
                continue;
            }
            for src in &inputs {
                if *src != out_frame {
                    changes.push((src.clone(), out_frame.clone(), p.to_string()));
                }
            }
        }

        if let Some(ex) = &extrinsics {
            for (src, dst, port) in &changes {
                match ex.route(src, dst) {
                    Ok(_) => {}
                    Err(FrameError::DynamicEdge { message, edges }) => {
                        let wired_pose = node
                            .inputs
                            .keys()
                            .any(|k| is_pose(k.as_str()) && upstream(node, k.as_str()).is_some());
                        if edges.len() > 1 {
                            // The node composes with one pose, standing in for
                            // one dynamic edge; a second would drop every
                            // message at runtime.
                            push(
                                "transform",
                                &nid,
                                port,
                                format!(
                                    "'{src}' -> '{dst}' crosses {} dynamic edges; one pose input covers one",
                                    edges.len()
                                ),
                            );
                        } else if !wired_pose {
                            push(
                                "transform",
                                &nid,
                                port,
                                format!(
                                    "{message}; node '{nid}' has no wired {POSE_TYPE} input to compose with"
                                ),
                            );
                        }
                    }
                    Err(e) => push("transform", &nid, port, e.to_string()),
                }
            }
        }

        match ctx.env(node, EXTRINSICS_ENV) {
            Some(named) => {
                let launched = resolve_path(&base_dir.join(&named));
                let checked = resolve_path(&ex_path);
                if launched != checked {
                    push(
                        "transform",
                        &nid,
                        EXTRINSICS_ENV,
                        format!(
                            "launches with {} but was validated against {}; a robot must run the calibration it was checked with",
                            launched.display(),
                            checked.display()
                        ),
                    );
                }
            }
            None if !changes.is_empty() => push(
                "transform",
                &nid,
                EXTRINSICS_ENV,
                format!(
                    "changes frames but sets no {EXTRINSICS_ENV}, so the node cannot load a calibration"
                ),
            ),
            None => {}
        }
    }
    out
}

/// Like Python's non-strict `Path.resolve()`: absolute, `..` collapsed, and
/// symlinks followed when the file exists.
fn resolve_path(p: &Path) -> PathBuf {
    let absolute = std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf());
    dunce::canonicalize(&absolute).unwrap_or_else(|_| normalize_path(&absolute))
}
