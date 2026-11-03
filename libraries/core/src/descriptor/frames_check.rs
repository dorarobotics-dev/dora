//! Pre-launch frame checks, run by `dora validate` when the dataflow has a
//! `frames:` key.
//!
//! Ported from the operator contract's validator: the per-edge frame check
//! and check 6 (a frame change needs a calibration that supports it), with
//! frame resolution as in its `resolve_frame`. The declarations come from
//! the nodes' `output_frames`/`input_frames` instead of an operators file.
//!
//! Check 4 is partial: a route that crosses one dynamic edge needs a wired
//! pose input, and a pose input whose upstream declares a frame must be in the
//! edge's parent frame (a pose's `frame_id` is its parent). No descriptor field
//! declares a pose's `child_frame`, so the child side of the edge is not
//! checked, and an undeclared pose frame is accepted.
//!
//! Each node reports its first failing route only (routes ordered by source
//! frame, target frame, output port).
//!
//! A node's `DORA_FRAMES` and `DORA_EXTRINSICS` must name the files the
//! dataflow's `frames.dictionary` and `frames.extrinsics` hold, and a node
//! that changes frames must set both.
//!
//! An unknown frame name is reported once, at the producer that declares it.
//! The reference also reports the route of every consumer of that output; here
//! a consumer's route check skips an input whose frame is not in the
//! dictionary, so one bad name yields one violation.

use crate::descriptor::{Descriptor, normalize_path};
use dora_message::{
    config::{InputMapping, UserInputMapping},
    descriptor::{EnvValue, Node},
};
use dora_transform::{Extrinsics, FrameDict, FrameError, Kind};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::path::{Path, PathBuf};

/// The input type a frame-changing node composes with across a dynamic edge.
pub const POSE_TYPE: &str = "std/math/v1/Pose";
/// The env var a node loads its calibration from.
pub const EXTRINSICS_ENV: &str = "DORA_EXTRINSICS";
/// The env var a node loads its frame dictionary from.
pub const FRAMES_ENV: &str = "DORA_FRAMES";

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
    /// dataflow-level `env`. The raw value, untrimmed; an empty value counts
    /// as unset (also when it overrides a non-empty dataflow-level value).
    fn env(&self, node: &Node, var: &str) -> Option<String> {
        node.env
            .as_ref()
            .and_then(|e| e.get(var))
            .or_else(|| self.global_env?.get(var))
            .map(|v| v.to_string())
            .filter(|v| !v.is_empty())
    }

    /// The type URN of `node`'s input `port`: its own `input_types` entry,
    /// else the `output_types` entry of the node output wired to it.
    fn port_type(&self, node: &'a Node, port: &str) -> Option<&'a str> {
        if let Some(t) = node.input_types.get(port) {
            return Some(t);
        }
        let up = upstream(node, port)?;
        self.nodes
            .get(up.source.as_ref() as &str)?
            .output_types
            .get(up.output.as_str())
            .map(String::as_str)
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
    match &node.inputs.get(input)?.mapping {
        InputMapping::User(m) => Some(m),
        InputMapping::Timer { .. } | InputMapping::Logs(_) => None,
    }
}

/// Run the frame checks. A dataflow without a `frames:` key has none.
/// `base_dir` is the dataflow file's directory; `frames:` paths resolve
/// against it, and a node's relative `DORA_FRAMES`/`DORA_EXTRINSICS` against the directory
/// the node is launched from (see [`launch_dir`]).
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

    let dict_path = base_dir.join(&cfg.dictionary);
    let frames = match FrameDict::load(&dict_path) {
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
        let is_pose = |port: &str| ctx.port_type(node, port) == Some(POSE_TYPE);

        // The frames this node's data inputs resolve to. A pose input is what
        // a transform composes with, not data it moves, so it is skipped.
        let mut inputs = BTreeSet::new();
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
            inputs.insert(frame);
        }

        let mut changes = BTreeSet::new();
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
                    changes.insert((src.clone(), out_frame.clone(), p.to_string()));
                }
            }
        }

        if let Some(ex) = &extrinsics {
            // The first failing route per node is reported.
            for (src, dst, port) in &changes {
                let failure = match ex.route(src, dst) {
                    Ok(_) => None,
                    Err(FrameError::DynamicEdge { message, edges }) => {
                        if edges.len() > 1 {
                            // The node composes with one pose, standing in for
                            // one dynamic edge; a second would drop every
                            // message at runtime.
                            Some(format!(
                                "'{src}' -> '{dst}' crosses {} dynamic edges; one pose input covers one",
                                edges.len()
                            ))
                        } else {
                            let (child, parent) = &edges[0];
                            pose_violation(&ctx, node, &message, (src, dst), (parent, child))
                        }
                    }
                    Err(e) => Some(e.to_string()),
                };
                if let Some(message) = failure {
                    push("transform", &nid, port, message);
                    break;
                }
            }
        }

        // The files the node loads must be the ones validated here.
        let pinned = [
            (FRAMES_ENV, &dict_path, "frame dictionary"),
            (EXTRINSICS_ENV, &ex_path, "calibration"),
        ];
        for (var, checked, what) in pinned {
            match ctx.env(node, var) {
                Some(named) => {
                    if let Some(message) = pin_violation(node, var, what, &named, base_dir, checked)
                    {
                        push("transform", &nid, var, message);
                    }
                }
                None if !changes.is_empty() => push(
                    "transform",
                    &nid,
                    var,
                    format!("changes frames but sets no {var}, so the node cannot load a {what}"),
                ),
                None => {}
            }
        }
    }
    out
}

/// Why `node`'s wired pose inputs cannot cover the dynamic edge
/// `parent -> child` that `message` reports, if so. A pose input covers it
/// when its upstream output's frame is `parent` or is undeclared (`any`,
/// unresolvable here): the pose's `child_frame` has no descriptor field, so
/// only its parent frame is checked.
fn pose_violation(
    ctx: &Ctx<'_>,
    node: &Node,
    message: &str,
    (src, dst): (&str, &str),
    (parent, child): (&str, &str),
) -> Option<String> {
    let nid = node.id.to_string();
    let mut wrong = Vec::new();
    for port in node.inputs.keys() {
        let p = port.as_str();
        if ctx.port_type(node, p) != Some(POSE_TYPE) {
            continue;
        }
        let Some(up) = upstream(node, p) else {
            continue;
        };
        match ctx.resolve(up.source.as_ref(), up.output.as_str()) {
            Ok(frame) if frame != "any" && frame != parent => {
                wrong.push(format!(
                    "pose input '{p}' ({}/{}) is in '{frame}'",
                    up.source, up.output
                ));
            }
            _ => return None,
        }
    }
    if wrong.is_empty() {
        return Some(format!(
            "{message}; node '{nid}' has no wired {POSE_TYPE} input to compose with"
        ));
    }
    Some(format!(
        "'{src}' -> '{dst}' crosses the dynamic edge '{parent}' -> '{child}', but {}; a pose covering that edge is in '{parent}'",
        wrong.join(", ")
    ))
}
/// Why the file `named` by the node's `var` (its `what`) is not the one at
/// `checked_path` that the dataflow was validated against, if so.
fn pin_violation(
    node: &Node,
    var: &str,
    what: &str,
    named: &str,
    base_dir: &Path,
    checked_path: &Path,
) -> Option<String> {
    let named_path = Path::new(named);
    if named_path.is_relative() && node.git.is_some() {
        return Some(format!(
            "{var} '{named}' is relative and the node runs from its git checkout, so the {what} cannot be checked before launch; use an absolute path"
        ));
    }
    let launched = resolve_path(&launch_dir(node, base_dir).join(named_path));
    if !launched.exists() {
        return Some(format!("{} does not exist", launched.display()));
    }
    let checked = resolve_path(checked_path);
    if launched != checked {
        return Some(format!(
            "launches with {} but was validated against {}; a robot must run the {what} it was checked with",
            launched.display(),
            checked.display()
        ));
    }
    None
}

/// The directory the daemon starts a (non-git) node in: `deploy.working_dir`
/// joined onto the dataflow directory (so an absolute one wins), else the
/// dataflow directory. Mirrors `binaries/daemon/src/dataflow_lifecycle.rs`.
fn launch_dir(node: &Node, base_dir: &Path) -> PathBuf {
    node.deploy
        .as_ref()
        .and_then(|d| d.working_dir.as_ref())
        .map_or_else(|| base_dir.to_path_buf(), |w| base_dir.join(w))
}

/// Like Python's non-strict `Path.resolve()`: absolute, `..` collapsed, and
/// symlinks followed when the file exists.
fn resolve_path(p: &Path) -> PathBuf {
    let absolute = std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf());
    dunce::canonicalize(&absolute).unwrap_or_else(|_| normalize_path(&absolute))
}
