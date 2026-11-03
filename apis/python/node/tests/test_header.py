"""`dora.header`: the Rust contract header (`dora_message::header`) from Python.

Every rule lives in Rust; these tests pin that the binding reaches each one,
ported from `libraries/message/src/header.rs` tests.
"""

import pytest

from dora import header as hd
from dora.header import Header, Upgrade, derive

KEYS = ("frame_id", "header_seq", "stamp_ns", "source", "validity")


def ok(frame="map", stamp=1, source="s", seq=0):
    return Header(frame, seq, stamp, source)


def stale(source="s", stamp=1):
    return Header("a", 0, stamp, source, "stale", "old")


def meta(**override):
    m = {"frame_id": "map", "header_seq": 1, "stamp_ns": 7, "source": "s", "validity": "ok"}
    m.update(override)
    return m


# --- constants -----------------------------------------------------------------

def test_key_constants_are_the_contract_fields():
    assert (hd.FRAME_ID, hd.HEADER_SEQ, hd.STAMP_NS, hd.SOURCE, hd.VALIDITY_KEY, hd.REASON,
            hd.CHILD_FRAME) == ("frame_id", "header_seq", "stamp_ns", "source", "validity",
                                "reason", "child_frame")
    assert hd.KEYS == (hd.FRAME_ID, hd.HEADER_SEQ, hd.STAMP_NS, hd.SOURCE, hd.VALIDITY_KEY,
                       hd.REASON, hd.CHILD_FRAME)


def test_validity_vocabulary_is_closed():
    assert hd.VALIDITY == ("ok", "missing", "stale", "unpaired", "low_confidence", "rejected")


def test_header_error_is_a_value_error():
    assert issubclass(hd.HeaderError, ValueError)


# --- construction ---------------------------------------------------------------

def test_fields_are_read_only_properties():
    h = Header("map", 3, 7, "s", "stale", "old", "base_link")
    assert (h.frame_id, h.seq, h.stamp_ns, h.source, h.validity, h.reason, h.child_frame) == (
        "map", 3, 7, "s", "stale", "old", "base_link")
    for name in ("frame_id", "seq", "stamp_ns", "source", "validity", "reason", "child_frame"):
        with pytest.raises(AttributeError):
            setattr(h, name, getattr(h, name))


def test_defaults_are_ok_with_no_reason_and_no_child_frame():
    h = ok()
    assert (h.validity, h.reason, h.child_frame) == ("ok", None, None)


def test_keywords_match_the_field_names():
    h = Header(frame_id="map", seq=0, stamp_ns=1, source="s", validity="missing", reason="gap",
               child_frame="c")
    assert h == Header("map", 0, 1, "s", "missing", "gap", "c")
    assert h != Header("map", 0, 1, "s", "missing", "gap")


def test_ok_with_reason_is_refused():
    with pytest.raises(ValueError, match="reason"):
        Header("map", 0, 1, "s", "ok", "x")


@pytest.mark.parametrize("v", ["missing", "stale", "unpaired", "low_confidence", "rejected"])
def test_every_degraded_validity_needs_a_non_empty_reason(v):
    for reason in (None, ""):
        with pytest.raises(ValueError, match="non-empty reason"):
            Header("map", 0, 1, "s", v, reason)
    assert Header("map", 0, 1, "s", v, "why").reason == "why"


@pytest.mark.parametrize("bad", ["broken", "OK", "", "none"])
def test_unknown_validity_is_refused(bad):
    with pytest.raises(ValueError, match="unknown validity"):
        Header("map", 0, 1, "s", bad)


@pytest.mark.parametrize("args, key", [
    (("", 0, 1, "s"), "frame_id"),
    (("map", 0, 1, ""), "source"),
    (("map", 0, 0, "s"), "stamp_ns"),
    (("map", 0, -1, "s"), "stamp_ns"),
    (("map", -1, 1, "s"), "seq"),
])
def test_empty_strings_and_out_of_range_ints_are_refused(args, key):
    with pytest.raises(ValueError, match=key):
        Header(*args)


def test_seq_zero_is_legal():
    assert ok().seq == 0


@pytest.mark.parametrize("bad", [True, False, 1.0, "1", None])
@pytest.mark.parametrize("pos, key", [(1, "seq"), (2, "stamp_ns")])
def test_ints_must_be_ints_not_bools_or_floats(pos, key, bad):
    args = ["map", 1, 1, "s"]
    args[pos] = bad
    with pytest.raises(ValueError, match=key):
        Header(*args)


@pytest.mark.parametrize("bad", [None, 3, b"map"])
@pytest.mark.parametrize("pos, key", [(0, "frame_id"), (3, "source")])
def test_strings_must_be_strings(pos, key, bad):
    args = ["map", 1, 1, "s"]
    args[pos] = bad
    with pytest.raises(ValueError, match=key):
        Header(*args)


def test_an_int_too_large_for_the_wire_is_refused():
    with pytest.raises(ValueError, match="seq"):
        Header("map", 2**63, 1, "s")


def test_child_frame_must_be_non_empty_when_given():
    with pytest.raises(ValueError, match="child_frame"):
        Header("odom", 0, 1, "s", child_frame="")


# --- metadata -------------------------------------------------------------------

def test_metadata_round_trip_omits_reason_and_child_frame_when_unset():
    h = ok("rslidar", 5, "lidar")
    m = h.to_metadata()
    assert m == {"frame_id": "rslidar", "header_seq": 0, "stamp_ns": 5, "source": "lidar",
                 "validity": "ok"}
    assert Header.from_metadata(m) == h


def test_degraded_header_with_child_frame_round_trips():
    h = Header("odom", 3, 7, "s", "low_confidence", "blur", "base_footprint")
    m = h.to_metadata()
    assert (m["reason"], m["child_frame"]) == ("blur", "base_footprint")
    assert all(type(v) in (str, int) for v in m.values())
    assert Header.from_metadata(m) == h


def test_from_metadata_reads_child_frame_when_present():
    assert Header.from_metadata(meta(child_frame="base_footprint")).child_frame == "base_footprint"
    assert Header.from_metadata(meta()).child_frame is None


@pytest.mark.parametrize("bad", ["", 3, True])
def test_from_metadata_refuses_a_bad_child_frame(bad):
    with pytest.raises(ValueError, match="child_frame"):
        Header.from_metadata(meta(child_frame=bad))


@pytest.mark.parametrize("key", KEYS)
def test_from_metadata_names_each_missing_key(key):
    m = meta()
    del m[key]
    with pytest.raises(ValueError, match=f"incomplete contract header.*{key}"):
        Header.from_metadata(m)


@pytest.mark.parametrize("key", ["header_seq", "stamp_ns"])
@pytest.mark.parametrize("bad", [True, False, 1.0, "3"])
def test_from_metadata_refuses_bool_float_and_str_ints(key, bad):
    with pytest.raises(ValueError, match=key):
        Header.from_metadata(meta(**{key: bad}))


@pytest.mark.parametrize("key", ["frame_id", "source", "validity", "reason"])
@pytest.mark.parametrize("bad", [3, True, None, 1.5])
def test_from_metadata_refuses_non_string_strings(key, bad):
    with pytest.raises(ValueError, match=key):
        Header.from_metadata(meta(**{key: bad}))


def test_from_metadata_applies_the_header_rules():
    with pytest.raises(ValueError, match="stamp_ns"):
        Header.from_metadata(meta(stamp_ns=0))
    with pytest.raises(ValueError, match="unknown validity"):
        Header.from_metadata(meta(validity="fine"))
    with pytest.raises(ValueError, match="non-empty reason"):
        Header.from_metadata(meta(validity="stale"))
    with pytest.raises(ValueError, match="reason must be absent"):
        Header.from_metadata(meta(reason="x"))


def test_from_metadata_ignores_other_keys():
    m = meta(encoding="jointstate", success=True, seq=9, timestamp=object())
    assert Header.from_metadata(m) == Header("map", 1, 7, "s")


@pytest.mark.parametrize("bad", [None, [], "frame_id"])
def test_from_metadata_needs_a_dict(bad):
    with pytest.raises(ValueError, match="dict"):
        Header.from_metadata(bad)


# --- derive ---------------------------------------------------------------------

def test_derive_takes_min_stamp_and_sorted_joined_sources():
    out = derive([ok("a", 9, "lidar"), ok("a", 4, "cam")], "map", 2, "ok")
    assert (out.stamp_ns, out.source, out.frame_id, out.seq, out.validity) == (
        4, "cam+lidar", "map", 2, "ok")


def test_derive_dedups_duplicate_sources():
    assert derive([ok("a", 1, "s"), ok("a", 2, "s")], "a", 0, "ok").source == "s"


def test_derive_needs_a_parent():
    with pytest.raises(ValueError, match="at least one parent"):
        derive([], "a", 0, "ok")


def test_derive_refuses_ok_from_degraded_without_upgrade():
    with pytest.raises(ValueError, match=r"invariant 2.*stale"):
        derive([ok("a", 1, "s"), stale(), ok("a", 1, "t")], "a", 0, "ok")
    out = derive([stale()], "a", 0, "stale", "old")
    assert (out.validity, out.reason) == ("stale", "old")


@pytest.mark.parametrize("v", ["unpaired", "low_confidence"])
def test_derive_allows_any_degraded_output(v):
    assert derive([stale()], "a", 0, v, "why").validity == v


def test_derive_applies_the_header_rules():
    with pytest.raises(ValueError, match="frame_id"):
        derive([ok()], "", 0, "ok")
    with pytest.raises(ValueError, match="non-empty reason"):
        derive([stale()], "a", 0, "stale")


def test_each_granted_upgrade_is_counted_once():
    counted = []
    for seq in range(3):
        out = derive([stale()], "a", seq, "ok", upgrade=Upgrade("dropped_stale", counted.append))
        assert (out.validity, out.reason) == ("ok", None)
    assert counted == ["dropped_stale"] * 3


def test_an_upgrade_is_refused_when_no_parent_is_degraded():
    counted = []
    with pytest.raises(ValueError, match="already ok"):
        derive([ok()], "a", 0, "ok", upgrade=Upgrade("unnecessary", counted.append))
    assert counted == []


def test_an_upgrade_is_refused_when_the_output_is_degraded():
    counted = []
    with pytest.raises(ValueError, match="only meaningful"):
        derive([stale()], "a", 0, "stale", "old", Upgrade("x", counted.append))
    assert counted == []


def test_an_upgrade_needs_a_non_empty_reason():
    counted = []
    with pytest.raises(ValueError, match="non-empty reason"):
        derive([stale()], "a", 0, "ok", upgrade=Upgrade("", counted.append))
    assert counted == []


def test_an_upgrade_is_not_counted_when_derive_fails():
    counted = []
    with pytest.raises(ValueError, match="frame_id"):
        derive([stale()], "", 0, "ok", upgrade=Upgrade("x", counted.append))
    assert counted == []


def test_an_exception_from_the_count_callback_propagates():
    def boom(reason):
        raise RuntimeError(f"counter down: {reason}")

    with pytest.raises(RuntimeError, match="counter down: x"):
        derive([stale()], "a", 0, "ok", upgrade=Upgrade("x", boom))


def test_derive_needs_headers():
    with pytest.raises(TypeError):
        derive([{"frame_id": "a"}], "a", 0, "ok")
