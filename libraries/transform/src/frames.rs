use crate::{FrameError, Result};
use serde_yaml::Value;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Fixed,
    Dynamic,
    Mounted,
    None,
}

impl Kind {
    fn parse(s: &str) -> Option<Kind> {
        match s {
            "fixed" => Some(Kind::Fixed),
            "dynamic" => Some(Kind::Dynamic),
            "mounted" => Some(Kind::Mounted),
            "none" => Some(Kind::None),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Kind::Fixed => "fixed",
            Kind::Dynamic => "dynamic",
            Kind::Mounted => "mounted",
            Kind::None => "none",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub name: String,
    pub parent: Option<String>,
    pub kind: Kind,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FrameDict {
    pub frames: BTreeMap<String, Frame>,
    pub rejected: BTreeMap<String, String>,
}

fn invalid<T>(msg: String) -> Result<T> {
    Err(FrameError::Invalid(msg))
}

impl FrameDict {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| FrameError::Invalid(format!("{}: {e}", path.display())))?;
        Self::parse(&text)
    }

    pub fn parse(text: &str) -> Result<Self> {
        let doc: Value =
            serde_yaml::from_str(text).map_err(|e| FrameError::Invalid(e.to_string()))?;
        let Some(list) = doc.get("frames").and_then(Value::as_sequence) else {
            return invalid("a frame dictionary must be a mapping with a 'frames' list".into());
        };
        let mut frames = BTreeMap::new();
        for entry in list {
            if !entry.is_mapping() {
                return invalid(format!("each frame must be a mapping, got {entry:?}"));
            }
            let Some(name) = entry
                .get("name")
                .and_then(Value::as_str)
                .filter(|s| !s.is_empty())
            else {
                return invalid(format!("a frame needs a non-empty name, got {entry:?}"));
            };
            if frames.contains_key(name) {
                return invalid(format!("frame '{name}' is declared more than once"));
            }
            let kind_raw = entry.get("kind").and_then(Value::as_str).unwrap_or("");
            let Some(kind) = Kind::parse(kind_raw) else {
                return invalid(format!(
                    "frame '{name}' has kind {kind_raw:?}; expected one of fixed, dynamic, mounted, none"
                ));
            };
            let parent = match entry.get("parent") {
                None | Some(Value::Null) => None,
                Some(Value::String(p)) => Some(p.clone()),
                Some(other) => {
                    return invalid(format!(
                        "frame '{name}' has parent {other:?}; expected a frame name or null"
                    ));
                }
            };
            frames.insert(
                name.to_string(),
                Frame {
                    name: name.to_string(),
                    parent,
                    kind,
                },
            );
        }
        for frame in frames.values() {
            if let Some(p) = &frame.parent
                && !frames.contains_key(p)
            {
                return invalid(format!("frame '{}' names unknown parent '{p}'", frame.name));
            }
            match (frame.kind, &frame.parent) {
                (Kind::Dynamic | Kind::Mounted, None) => {
                    return invalid(format!(
                        "{} frame '{}' needs a parent",
                        frame.kind.as_str(),
                        frame.name
                    ));
                }
                (Kind::Fixed | Kind::None, Some(_)) => {
                    return invalid(format!(
                        "{} frame '{}' cannot have a parent",
                        frame.kind.as_str(),
                        frame.name
                    ));
                }
                _ => {}
            }
        }
        for start in frames.keys() {
            let mut seen = BTreeSet::new();
            let mut node = Some(start.clone());
            while let Some(n) = node {
                if !seen.insert(n.clone()) {
                    return invalid(format!("the frame tree has a cycle through '{n}'"));
                }
                node = frames[&n].parent.clone();
            }
        }
        let mut rejected = BTreeMap::new();
        match doc.get("rejected_names") {
            None | Some(Value::Null) => {}
            Some(Value::Mapping(map)) => {
                for (bad, good) in map {
                    let (Some(bad), Some(good)) = (bad.as_str(), good.as_str()) else {
                        return invalid("rejected_names must map an upstream name to a frame".into());
                    };
                    if frames.contains_key(bad) {
                        return invalid(format!("'{bad}' is both a frame and a rejected name"));
                    }
                    if !frames.contains_key(good) {
                        return invalid(format!(
                            "rejected name '{bad}' points at unknown frame '{good}'"
                        ));
                    }
                    rejected.insert(bad.to_string(), good.to_string());
                }
            }
            Some(_) => {
                return invalid("rejected_names must map an upstream name to a frame".into());
            }
        }
        Ok(FrameDict { frames, rejected })
    }

    pub fn require(&self, name: &str) -> Result<&Frame> {
        if let Some(good) = self.rejected.get(name) {
            return Err(FrameError::UnknownFrame(format!(
                "frame '{name}' is an upstream name the dictionary rejects; use '{good}'"
            )));
        }
        self.frames.get(name).ok_or_else(|| {
            FrameError::UnknownFrame(format!("frame '{name}' is not in the frame dictionary"))
        })
    }

    /// `name` and its ancestors, nearest first, ending at a root.
    pub fn chain(&self, name: &str) -> Result<Vec<String>> {
        self.require(name)?;
        let mut out = Vec::new();
        let mut node = Some(name.to_string());
        while let Some(n) = node {
            node = self.frames[&n].parent.clone();
            out.push(n);
        }
        Ok(out)
    }
}
