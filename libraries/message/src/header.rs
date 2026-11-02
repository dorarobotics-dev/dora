//! The contract header carried in dora metadata.
//!
//! A message's frame, capture time, sequence, source and validity travel as
//! metadata parameters under the keys below. A degraded message is forwarded,
//! never silently dropped, and never upgraded except by an operator that
//! counts the upgrade (see [`derive`]).
//!
//! Header keys are unprefixed names, except `header_seq`, which avoids dora's
//! streaming `seq`. User code must not reuse these keys for other meanings.

use crate::metadata::{MetadataParameters, Parameter};
use std::fmt;

/// Coordinate frame the payload is expressed in.
pub const FRAME_ID: &str = "frame_id";
/// Per-output counter >= 0, so a gap means loss on the edge.
pub const SEQ: &str = "header_seq";
/// Capture time in integer nanoseconds on the producer's clock.
pub const STAMP_NS: &str = "stamp_ns";
/// Name of the producing node, or the sorted `+`-joined parent sources.
pub const SOURCE: &str = "source";
/// One of the [`Validity`] values, in snake case.
pub const VALIDITY: &str = "validity";
/// Why the message is degraded; absent when validity is `ok`.
pub const REASON: &str = "reason";
/// The frame whose pose a Pose/Transform message gives.
pub const CHILD_FRAME: &str = "child_frame";

/// Error for a malformed or inconsistent header.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct HeaderError(String);

impl HeaderError {
    /// The human-readable description.
    pub fn message(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for HeaderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for HeaderError {}

fn err<T>(msg: impl Into<String>) -> Result<T, HeaderError> {
    Err(HeaderError(msg.into()))
}

/// Whether a message can be trusted as-is.
///
/// Deliberately exhaustive: the contract's set of validity values is closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Validity {
    /// Fully valid.
    Ok,
    /// The data could not be produced.
    Missing,
    /// The data is older than allowed.
    Stale,
    /// Inputs that should pair up did not.
    Unpaired,
    /// Produced, but with low confidence.
    LowConfidence,
    /// Refused by a check; do not use.
    Rejected,
}

impl Validity {
    pub const ALL: [Validity; 6] = [
        Validity::Ok,
        Validity::Missing,
        Validity::Stale,
        Validity::Unpaired,
        Validity::LowConfidence,
        Validity::Rejected,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Validity::Ok => "ok",
            Validity::Missing => "missing",
            Validity::Stale => "stale",
            Validity::Unpaired => "unpaired",
            Validity::LowConfidence => "low_confidence",
            Validity::Rejected => "rejected",
        }
    }

    pub fn parse(s: &str) -> Result<Self, HeaderError> {
        Self::ALL.into_iter().find(|v| v.as_str() == s).map_or_else(
            || {
                let expected = Self::ALL.map(Validity::as_str).join(", ");
                err(format!(
                    "unknown validity {s:?}; expected one of {expected}"
                ))
            },
            Ok,
        )
    }
}

/// The contract header. Build it with [`Header::new`], [`Header::read`] or [`derive`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct Header {
    /// Coordinate frame of the payload; non-empty.
    pub frame_id: String,
    /// Per-output counter, >= 0.
    pub seq: i64,
    /// Capture time in nanoseconds, >= 1.
    pub stamp_ns: i64,
    /// Producing node or joined parent sources; non-empty.
    pub source: String,
    /// Whether the payload can be trusted.
    pub validity: Validity,
    /// Required and non-empty when degraded, absent when `ok`.
    pub reason: Option<String>,
}

impl Header {
    pub fn new(
        frame_id: impl Into<String>,
        seq: i64,
        stamp_ns: i64,
        source: impl Into<String>,
        validity: Validity,
        reason: Option<String>,
    ) -> Result<Self, HeaderError> {
        let h = Header {
            frame_id: frame_id.into(),
            seq,
            stamp_ns,
            source: source.into(),
            validity,
            reason,
        };
        if h.frame_id.is_empty() {
            return err("frame_id must be a non-empty string");
        }
        if h.source.is_empty() {
            return err("source must be a non-empty string");
        }
        if h.seq < 0 {
            return err(format!("seq must be an int >= 0, got {}", h.seq));
        }
        if h.stamp_ns < 1 {
            return err(format!("stamp_ns must be an int >= 1, got {}", h.stamp_ns));
        }
        match (&h.validity, &h.reason) {
            (Validity::Ok, Some(_)) => return err("reason must be absent when validity is ok"),
            (v, None) if *v != Validity::Ok => {
                return err(format!(
                    "validity {:?} requires a non-empty reason",
                    v.as_str()
                ));
            }
            (v, Some(r)) if *v != Validity::Ok && r.is_empty() => {
                return err(format!(
                    "validity {:?} requires a non-empty reason",
                    v.as_str()
                ));
            }
            _ => {}
        }
        Ok(h)
    }

    /// Write the header into `params`. `reason` is omitted when absent.
    pub fn write(&self, params: &mut MetadataParameters) {
        params.insert(FRAME_ID.into(), Parameter::String(self.frame_id.clone()));
        params.insert(SEQ.into(), Parameter::Integer(self.seq));
        params.insert(STAMP_NS.into(), Parameter::Integer(self.stamp_ns));
        params.insert(SOURCE.into(), Parameter::String(self.source.clone()));
        params.insert(
            VALIDITY.into(),
            Parameter::String(self.validity.as_str().into()),
        );
        match &self.reason {
            Some(r) => {
                params.insert(REASON.into(), Parameter::String(r.clone()));
            }
            None => {
                params.remove(REASON);
            }
        }
    }

    /// Read a header back. A missing or wrongly typed key is an error naming it.
    pub fn read(params: &MetadataParameters) -> Result<Self, HeaderError> {
        let string = |key: &str| match params.get(key) {
            Some(Parameter::String(s)) => Ok(s.clone()),
            Some(other) => err(format!(
                "metadata key {key:?} must be a string, got {other:?}"
            )),
            None => err(format!("incomplete contract header; missing {key:?}")),
        };
        let integer = |key: &str| match params.get(key) {
            Some(Parameter::Integer(i)) => Ok(*i),
            Some(other) => err(format!(
                "metadata key {key:?} must be an integer, got {other:?}"
            )),
            None => err(format!("incomplete contract header; missing {key:?}")),
        };
        let reason = match params.get(REASON) {
            Some(Parameter::String(s)) => Some(s.clone()),
            Some(other) => {
                return err(format!(
                    "metadata key {REASON:?} must be a string, got {other:?}"
                ));
            }
            None => None,
        };
        Header::new(
            string(FRAME_ID)?,
            integer(SEQ)?,
            integer(STAMP_NS)?,
            string(SOURCE)?,
            Validity::parse(&string(VALIDITY)?)?,
            reason,
        )
    }
}

/// A counted upgrade: the only way an output may be `ok` while a parent is not.
pub struct Upgrade<'a> {
    /// Why the upgrade is justified; passed to `count`.
    pub reason: &'a str,
    /// Called once with `reason` when the upgrade is applied.
    pub count: &'a mut dyn FnMut(&str),
}

/// The header of a message computed from `parents`.
///
/// The caller states the output validity. It cannot be `ok` while a parent is
/// degraded unless an [`Upgrade`] is given, which is counted. The stamp is the
/// earliest parent stamp; the source is the sorted, `+`-joined parent sources.
pub fn derive(
    parents: &[Header],
    frame_id: &str,
    seq: i64,
    validity: Validity,
    reason: Option<String>,
    upgrade: Option<Upgrade<'_>>,
) -> Result<Header, HeaderError> {
    if parents.is_empty() {
        return err("derive() needs at least one parent; source nodes build a Header directly");
    }
    if upgrade.as_ref().is_some_and(|up| up.reason.is_empty()) {
        return err("an upgrade needs a non-empty reason");
    }
    let mut degraded: Vec<&str> = parents
        .iter()
        .filter(|p| p.validity != Validity::Ok)
        .map(|p| p.validity.as_str())
        .collect();
    degraded.sort_unstable();
    degraded.dedup();
    match (validity, &upgrade) {
        (Validity::Ok, None) if !degraded.is_empty() => {
            return err(format!(
                "invariant 2: the output cannot be ok while its inputs are {degraded:?} without a counted upgrade"
            ));
        }
        (Validity::Ok, Some(_)) if degraded.is_empty() => {
            return err("an upgrade was offered but every input is already ok");
        }
        (v, Some(_)) if v != Validity::Ok => {
            return err("an upgrade is only meaningful when the output is ok");
        }
        _ => {}
    }
    let mut sources: Vec<&str> = parents.iter().map(|p| p.source.as_str()).collect();
    sources.sort_unstable();
    sources.dedup();
    let header = Header::new(
        frame_id,
        seq,
        parents
            .iter()
            .map(|p| p.stamp_ns)
            .min()
            .expect("parents is non-empty"),
        sources.join("+"),
        validity,
        reason,
    )?;
    if let Some(up) = upgrade {
        (up.count)(up.reason);
    }
    Ok(header)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metadata::{MetadataParameters, Parameter};

    fn ok(frame: &str, stamp: i64, source: &str) -> Header {
        Header::new(frame, 0, stamp, source, Validity::Ok, None).unwrap()
    }

    #[test]
    fn validity_round_trips_as_snake_case() {
        for v in Validity::ALL {
            assert_eq!(Validity::parse(v.as_str()).unwrap(), v);
        }
        assert_eq!(Validity::LowConfidence.as_str(), "low_confidence");
        assert!(Validity::parse("fine").is_err());
    }

    #[test]
    fn ok_with_reason_is_refused() {
        assert!(Header::new("map", 0, 1, "s", Validity::Ok, Some("x".into())).is_err());
    }

    #[test]
    fn degraded_without_reason_is_refused() {
        assert!(Header::new("map", 0, 1, "s", Validity::Stale, None).is_err());
        assert!(Header::new("map", 0, 1, "s", Validity::Stale, Some(String::new())).is_err());
    }

    #[test]
    fn empty_frame_or_source_and_zero_stamp_are_refused() {
        assert!(Header::new("", 0, 1, "s", Validity::Ok, None).is_err());
        assert!(Header::new("map", 0, 1, "", Validity::Ok, None).is_err());
        assert!(Header::new("map", 0, 0, "s", Validity::Ok, None).is_err());
        assert!(Header::new("map", -1, 1, "s", Validity::Ok, None).is_err());
    }

    #[test]
    fn metadata_round_trip_omits_reason_when_ok() {
        let h = ok("rslidar", 5, "lidar");
        let mut params = MetadataParameters::new();
        h.write(&mut params);
        assert!(!params.contains_key(REASON));
        assert_eq!(Header::read(&params).unwrap(), h);
    }

    #[test]
    fn read_names_the_missing_key() {
        let mut params = MetadataParameters::new();
        ok("map", 1, "s").write(&mut params);
        params.remove(FRAME_ID);
        let err = Header::read(&params).unwrap_err().to_string();
        assert!(err.contains("frame_id"), "{err}");
    }

    #[test]
    fn read_refuses_a_wrongly_typed_key() {
        let mut params = MetadataParameters::new();
        ok("map", 1, "s").write(&mut params);
        params.insert(SEQ.into(), Parameter::String("3".into()));
        let err = Header::read(&params).unwrap_err().to_string();
        assert!(err.contains("seq"), "{err}");
    }

    #[test]
    fn derive_takes_min_stamp_and_joined_sources() {
        let out = derive(
            &[ok("a", 9, "lidar"), ok("a", 4, "cam")],
            "map",
            2,
            Validity::Ok,
            None,
            None,
        )
        .unwrap();
        assert_eq!(out.stamp_ns, 4);
        assert_eq!(out.source, "cam+lidar");
        assert_eq!(out.frame_id, "map");
        assert_eq!(out.seq, 2);
    }

    #[test]
    fn derive_refuses_ok_from_degraded_without_upgrade() {
        let stale = Header::new("a", 0, 1, "s", Validity::Stale, Some("old".into())).unwrap();
        assert!(
            derive(
                std::slice::from_ref(&stale),
                "a",
                0,
                Validity::Ok,
                None,
                None
            )
            .is_err()
        );
        let out = derive(&[stale], "a", 0, Validity::Stale, Some("old".into()), None).unwrap();
        assert_eq!(out.validity, Validity::Stale);
    }

    #[test]
    fn derive_counts_an_upgrade() {
        let stale = Header::new("a", 0, 1, "s", Validity::Stale, Some("old".into())).unwrap();
        let mut counted = Vec::new();
        let mut count = |r: &str| counted.push(r.to_string());
        derive(
            &[stale],
            "a",
            0,
            Validity::Ok,
            None,
            Some(Upgrade {
                reason: "refit",
                count: &mut count,
            }),
        )
        .unwrap();
        assert_eq!(counted, vec!["refit".to_string()]);
    }

    #[test]
    fn derive_refuses_upgrade_with_no_degraded_parent() {
        let mut count = |_: &str| {};
        assert!(
            derive(
                &[ok("a", 1, "s")],
                "a",
                0,
                Validity::Ok,
                None,
                Some(Upgrade {
                    reason: "x",
                    count: &mut count
                })
            )
            .is_err()
        );
    }

    #[test]
    fn degraded_header_round_trips() {
        let h = Header::new(
            "map",
            3,
            7,
            "s",
            Validity::LowConfidence,
            Some("blur".into()),
        )
        .unwrap();
        let mut params = MetadataParameters::new();
        h.write(&mut params);
        assert_eq!(Header::read(&params).unwrap(), h);
    }

    #[test]
    fn writing_ok_over_degraded_clears_reason() {
        let mut params = MetadataParameters::new();
        Header::new("map", 0, 1, "s", Validity::Stale, Some("old".into()))
            .unwrap()
            .write(&mut params);
        ok("map", 2, "s").write(&mut params);
        let back = Header::read(&params).unwrap();
        assert_eq!(back.validity, Validity::Ok);
        assert_eq!(back.reason, None);
    }

    #[test]
    fn derive_refuses_upgrade_when_output_is_degraded() {
        let stale = Header::new("a", 0, 1, "s", Validity::Stale, Some("old".into())).unwrap();
        let mut count = |_: &str| {};
        assert!(
            derive(
                &[stale],
                "a",
                0,
                Validity::Stale,
                Some("old".into()),
                Some(Upgrade {
                    reason: "x",
                    count: &mut count
                })
            )
            .is_err()
        );
    }

    #[test]
    fn derive_does_not_count_when_it_fails() {
        let stale = Header::new("a", 0, 1, "s", Validity::Stale, Some("old".into())).unwrap();
        let mut counted = 0;
        let mut count = |_: &str| counted += 1;
        assert!(
            derive(
                &[stale],
                "",
                0,
                Validity::Ok,
                None,
                Some(Upgrade {
                    reason: "x",
                    count: &mut count
                })
            )
            .is_err()
        );
        assert_eq!(counted, 0);
    }

    #[test]
    fn derive_dedups_duplicate_sources() {
        let out = derive(
            &[ok("a", 1, "s"), ok("a", 2, "s")],
            "a",
            0,
            Validity::Ok,
            None,
            None,
        )
        .unwrap();
        assert_eq!(out.source, "s");
    }

    #[test]
    fn derive_needs_a_parent() {
        assert!(derive(&[], "a", 0, Validity::Ok, None, None).is_err());
    }

    // Parity with the reference suite (docs/contract-core/test-mapping.md).

    fn stale() -> Header {
        Header::new("a", 0, 1, "s", Validity::Stale, Some("old".into())).unwrap()
    }

    #[test]
    fn validity_vocabulary_is_closed() {
        assert_eq!(
            Validity::ALL.map(Validity::as_str),
            [
                "ok",
                "missing",
                "stale",
                "unpaired",
                "low_confidence",
                "rejected"
            ]
        );
        for bad in ["broken", "OK", "", "none"] {
            assert!(Validity::parse(bad).is_err(), "{bad:?} accepted");
        }
    }

    #[test]
    fn every_degraded_validity_needs_a_reason() {
        for v in Validity::ALL.into_iter().filter(|v| *v != Validity::Ok) {
            assert!(Header::new("map", 0, 1, "s", v, None).is_err());
            assert!(Header::new("map", 0, 1, "s", v, Some(String::new())).is_err());
            let h = Header::new("map", 0, 1, "s", v, Some("why".into())).unwrap();
            assert_eq!(h.reason.as_deref(), Some("why"));
        }
    }

    #[test]
    fn seq_zero_is_legal_and_negative_stamp_is_not() {
        assert_eq!(ok("map", 1, "s").seq, 0);
        assert!(Header::new("map", 0, -1, "s", Validity::Ok, None).is_err());
    }

    #[test]
    fn read_refuses_bool_and_float_seq_and_stamp() {
        for key in [SEQ, STAMP_NS] {
            for bad in [Parameter::Bool(true), Parameter::Float(1.0)] {
                let mut params = MetadataParameters::new();
                ok("map", 1, "s").write(&mut params);
                params.insert(key.into(), bad);
                let err = Header::read(&params).unwrap_err().to_string();
                assert!(err.contains(key), "{err}");
            }
        }
    }

    #[test]
    fn read_applies_the_header_rules() {
        let mut params = MetadataParameters::new();
        ok("map", 1, "s").write(&mut params);
        params.insert(STAMP_NS.into(), Parameter::Integer(0));
        let err = Header::read(&params).unwrap_err().to_string();
        assert!(err.contains("stamp_ns"), "{err}");
    }

    #[test]
    fn read_names_each_missing_key() {
        for key in [FRAME_ID, SEQ, STAMP_NS, SOURCE, VALIDITY] {
            let mut params = MetadataParameters::new();
            ok("map", 1, "s").write(&mut params);
            params.remove(key);
            let err = Header::read(&params).unwrap_err().to_string();
            assert!(
                err.contains(key) && err.contains("incomplete contract header"),
                "{err}"
            );
        }
    }

    #[test]
    fn header_keys_are_the_contract_fields() {
        assert_eq!(
            [FRAME_ID, SEQ, STAMP_NS, SOURCE, VALIDITY, REASON],
            [
                "frame_id",
                "header_seq",
                "stamp_ns",
                "source",
                "validity",
                "reason"
            ]
        );
        // `seq` is dora's streaming chunk counter, never the header's.
        assert_ne!(SEQ, crate::metadata::SEQ);
    }

    #[test]
    fn written_header_is_flat_scalars() {
        let mut params = MetadataParameters::new();
        Header::new("map", 3, 7, "s", Validity::Stale, Some("old".into()))
            .unwrap()
            .write(&mut params);
        assert_eq!(params.len(), 6);
        assert!(
            params
                .values()
                .all(|v| matches!(v, Parameter::String(_) | Parameter::Integer(_))),
            "{params:?}"
        );
    }

    #[test]
    fn other_metadata_passes_through() {
        let mut params = MetadataParameters::new();
        params.insert("encoding".into(), Parameter::String("jointstate".into()));
        params.insert("success".into(), Parameter::Bool(true));
        let h = ok("map", 1, "s");
        h.write(&mut params);
        assert_eq!(
            params.get("encoding"),
            Some(&Parameter::String("jointstate".into()))
        );
        assert_eq!(params.get("success"), Some(&Parameter::Bool(true)));
        assert_eq!(Header::read(&params).unwrap(), h);
    }

    #[test]
    fn writing_the_header_replaces_shadowing_values() {
        let mut params = MetadataParameters::new();
        for key in [FRAME_ID, SEQ, STAMP_NS, SOURCE, VALIDITY, REASON] {
            params.insert(key.into(), Parameter::String("hijacked".into()));
        }
        let h = ok("map", 1, "s");
        h.write(&mut params);
        assert_eq!(Header::read(&params).unwrap(), h);
        assert!(!params.contains_key(REASON));
    }

    #[test]
    fn derive_allows_any_degraded_output() {
        for v in [Validity::Unpaired, Validity::LowConfidence] {
            let out = derive(&[stale()], "a", 0, v, Some("why".into()), None).unwrap();
            assert_eq!(out.validity, v);
        }
    }

    #[test]
    fn one_degraded_parent_among_several_needs_an_upgrade() {
        let err = derive(
            &[ok("a", 1, "s"), stale(), ok("a", 1, "t")],
            "a",
            0,
            Validity::Ok,
            None,
            None,
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("invariant 2"), "{err}");
    }

    #[test]
    fn the_error_names_the_offending_validity_values() {
        let err = derive(&[stale()], "a", 0, Validity::Ok, None, None)
            .unwrap_err()
            .to_string();
        assert!(err.contains("stale"), "{err}");
    }

    #[test]
    fn each_granted_upgrade_is_counted_once() {
        let mut counted = 0;
        let mut count = |_: &str| counted += 1;
        for seq in 0..3 {
            derive(
                &[stale()],
                "a",
                seq,
                Validity::Ok,
                None,
                Some(Upgrade {
                    reason: "dropped_stale",
                    count: &mut count,
                }),
            )
            .unwrap();
        }
        assert_eq!(counted, 3);
    }

    #[test]
    fn an_upgrade_needs_a_reason_and_is_not_counted_when_refused() {
        let mut counted = 0;
        let mut count = |_: &str| counted += 1;
        let err = derive(
            &[stale()],
            "a",
            0,
            Validity::Ok,
            None,
            Some(Upgrade {
                reason: "",
                count: &mut count,
            }),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("non-empty reason"), "{err}");
        let mut count = |_: &str| counted += 1;
        assert!(
            derive(
                &[ok("a", 1, "s")],
                "a",
                0,
                Validity::Ok,
                None,
                Some(Upgrade {
                    reason: "unnecessary",
                    count: &mut count
                })
            )
            .is_err()
        );
        assert_eq!(counted, 0);
    }
}
