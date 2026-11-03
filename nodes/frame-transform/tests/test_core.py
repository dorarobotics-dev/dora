from pathlib import Path

import numpy as np
import pyarrow as pa
import pytest

from dora import transform as tf
from dora.header import Header
from dora_frame_transform import core
from dora_frame_transform.core import (
    POSE_MAX_AGE_MS, Pose, cloud_from_arrow, cloud_to_arrow, on_pose, process,
)

FIX = Path(__file__).resolve().parents[3] / "libraries/transform/tests/fixtures"


@pytest.fixture
def ex():
    return tf.Extrinsics.load(str(FIX / "extrinsics.yml"), tf.FrameDict.load(str(FIX / "frames.yml")))


def hdr(frame="rslidar", stamp=1_000_000_000, validity="ok", reason=None, source="lidar"):
    return Header(frame_id=frame, seq=0, stamp_ns=stamp, source=source, validity=validity, reason=reason)


def pose_meta(stamp=1_000_000_000, validity="ok", reason=None):
    m = {"frame_id": "odom", "child_frame": "base_footprint", "header_seq": 0, "stamp_ns": stamp,
         "source": "odom", "validity": validity}
    if reason:
        m["reason"] = reason
    return m


def pose_arrow(x=1.0, y=2.0):
    return pa.array([{"position": {"x": x, "y": y, "z": 0.0},
                      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}])


def seqs():
    n = iter(range(100))
    return lambda: next(n)


def pts():
    return np.array([[0, 0, 0, 7]], dtype="<f4")


def test_cloud_round_trip():
    a = cloud_to_arrow(pts())
    assert np.array_equal(cloud_from_arrow(a, 4), pts())


def test_static_transform_keeps_intensity_stamp_source(ex):
    out, h, drops = process(pts(), hdr(), extrinsics=ex, target_frame="base_footprint", take_seq=seqs())
    assert np.allclose(out, [[0.25, 0, 0.35, 7]]) and drops == {}
    assert (h.frame_id, h.stamp_ns, h.source, h.validity) == ("base_footprint", 1_000_000_000, "lidar", "ok")


def test_pose_composes_into_odom(ex):
    pose = on_pose(pose_arrow(), pose_meta(), ex.frames)
    out, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=seqs(), pose=pose)
    assert np.allclose(out[0, :3], [1.25, 2.0, 0.35]) and h.validity == "ok"


def test_no_pose_is_a_counted_drop_not_a_wrong_frame(ex):
    out, h, drops = process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=seqs())
    assert out is None and h is None and drops == {"no_pose": 1}


@pytest.mark.parametrize("delta_ms", [POSE_MAX_AGE_MS + 1, -(POSE_MAX_AGE_MS + 1)])
def test_stale_pose_either_direction(ex, delta_ms):
    pose = on_pose(pose_arrow(), pose_meta(stamp=1_000_000_000 + delta_ms * 1_000_000), ex.frames)
    _, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=seqs(), pose=pose)
    assert (h.validity, h.reason) == ("stale", "stale_pose")


def test_degraded_pose_passes_its_validity(ex):
    pose = on_pose(pose_arrow(), pose_meta(validity="low_confidence", reason="fit"), ex.frames)
    _, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=seqs(), pose=pose)
    assert (h.validity, h.reason) == ("low_confidence", "pose_fit")


def test_degraded_cloud_is_never_upgraded(ex):
    pose = on_pose(pose_arrow(), pose_meta(), ex.frames)
    _, h, _ = process(pts(), hdr(validity="stale", reason="old"), extrinsics=ex,
                      target_frame="odom", take_seq=seqs(), pose=pose)
    assert (h.validity, h.reason) == ("stale", "old")


def test_bad_pose_needs_child_frame(ex):
    m = pose_meta()
    del m["child_frame"]
    with pytest.raises(ValueError, match="child_frame"):
        on_pose(pose_arrow(), m, ex.frames)


def test_seq_taken_only_when_published(ex):
    take = seqs()
    process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=take)  # dropped
    _, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="base_link", take_seq=take)
    assert h.seq == 0


def test_header_metadata_round_trip():
    h = hdr(validity="stale", reason="x")
    assert Header.from_metadata(h.to_metadata()) == h
    with pytest.raises(ValueError, match="frame_id"):
        Header.from_metadata({k: v for k, v in h.to_metadata().items() if k != "frame_id"})


# --- parity with the reference suite (docs/contract-core/test-mapping.md) ----

TWO = np.array([[1.0, 2.0, 3.0, 42.0], [0.0, 0.0, 0.0, 7.0]], dtype=np.float32)


def test_static_change_to_camera_front_where_the_hand_says(ex):
    """(1,2,3) in rslidar -> (-2, -2.9, 0.85) in camera_front, worked by hand."""
    out, h, drops = process(TWO, hdr(), extrinsics=ex, target_frame="camera_front", take_seq=seqs())
    assert drops == {}
    assert out[0, :3] == pytest.approx([-2.0, -2.9, 0.85], abs=1e-6)
    assert out[:, 3].tolist() == [42.0, 7.0]
    assert out.dtype == np.float32
    assert (h.frame_id, h.seq, h.validity) == ("camera_front", 0, "ok")


def test_the_input_array_is_not_modified(ex):
    before = TWO.copy()
    process(TWO, hdr(), extrinsics=ex, target_frame="camera_front", take_seq=seqs())
    assert np.array_equal(TWO, before)


def test_a_pose_exactly_at_the_limit_keeps_the_cloud_ok(ex):
    pose = on_pose(pose_arrow(), pose_meta(stamp=1_000_000_000 - POSE_MAX_AGE_MS * 1_000_000), ex.frames)
    _, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="odom", take_seq=seqs(), pose=pose)
    assert h.validity == "ok"


def test_a_degraded_cloud_keeps_its_own_reason_over_a_stale_pose(ex):
    pose = on_pose(pose_arrow(), pose_meta(stamp=2_000_000_000), ex.frames)
    _, h, _ = process(pts(), hdr(validity="missing", reason="gap"), extrinsics=ex,
                      target_frame="odom", take_seq=seqs(), pose=pose)
    assert (h.validity, h.reason) == ("missing", "gap")


def test_a_pose_is_ignored_when_the_path_is_static(ex):
    pose = on_pose(pose_arrow(), pose_meta(stamp=2_000_000_000), ex.frames)
    _, h, _ = process(pts(), hdr(), extrinsics=ex, target_frame="camera_front", take_seq=seqs(), pose=pose)
    assert h.validity == "ok"


def test_a_pose_with_a_non_unit_quaternion_is_refused(ex):
    bad = pa.array([{"position": {"x": 1.0, "y": 2.0, "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": 1.0, "w": 1.0}}])
    with pytest.raises(ValueError, match="unit quaternion"):
        on_pose(bad, pose_meta(), ex.frames)


def test_a_pose_quaternion_is_read_by_name_not_by_position(ex):
    """Upstream writes orientation as w, x, y, z; the fields are read by name."""
    s = float(np.sin(np.pi / 4))
    wxyz = pa.array([{"position": {"x": 1.0, "y": 2.0, "z": 0.0},
                      "orientation": {"w": s, "x": 0.0, "y": 0.0, "z": s}}])
    assert wxyz.type.field("orientation").type.field(0).name == "w"
    pose = on_pose(wxyz, pose_meta(), ex.frames)
    assert pose.transform.apply([1.0, 0.0, 0.0]) == pytest.approx([1.0, 3.0, 0.0])


# --- configuration, read before the first message ------------------------------

from dora_frame_transform import main as node_main  # noqa: E402


@pytest.fixture
def calib_env(monkeypatch):
    monkeypatch.setenv("DORA_FRAMES", str(FIX / "frames.yml"))
    monkeypatch.setenv("DORA_EXTRINSICS", str(FIX / "extrinsics.yml"))
    monkeypatch.setenv("TARGET_FRAME", "base_link")
    return monkeypatch


def test_config_needs_a_target_frame(calib_env):
    calib_env.delenv("TARGET_FRAME")
    with pytest.raises(KeyError, match="TARGET_FRAME"):
        node_main.config()


@pytest.mark.parametrize("target, match", [
    ("world", "use 'map'"), ("none", "no spatial meaning"),
    ("any", "not in the frame dictionary"), ("", "not in the frame dictionary"),
])
def test_config_refuses_a_target_that_is_not_a_spatial_frame(calib_env, target, match):
    calib_env.setenv("TARGET_FRAME", target)
    with pytest.raises((ValueError, SystemExit), match=match):
        node_main.config()


def test_config_refuses_a_bad_calibration(calib_env, tmp_path):
    bad = tmp_path / "extrinsics.yml"
    bad.write_text("contract_version: 1.0.0\n", encoding="utf-8")
    calib_env.setenv("DORA_EXTRINSICS", str(bad))
    with pytest.raises(tf.FrameError, match="missing the required key"):
        node_main.config()


def test_config_reads_a_good_environment(calib_env):
    extrinsics, target, step, diag_every = node_main.config()
    assert (target, step, diag_every) == ("base_link", 4, 10)
    assert extrinsics.path == str(FIX / "extrinsics.yml")


# --- malformed messages are counted drops, never a crash -----------------------

def _pose_row(**override):
    row = {"position": {"x": 1.0, "y": 2.0, "z": 0.0},
           "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
    for path, v in override.items():
        part, key = path.split("__")
        if v is _DROP:
            del row[part][key]
        else:
            row[part][key] = v
    return row


_DROP = object()


def test_a_pose_missing_a_component_is_a_value_error(ex):
    with pytest.raises(ValueError, match=r"orientation\.w"):
        on_pose(pa.array([_pose_row(orientation__w=_DROP)]), pose_meta(), ex.frames)


def test_a_pose_with_a_null_component_is_a_value_error(ex):
    with pytest.raises(ValueError, match=r"position\.y"):
        on_pose(pa.array([_pose_row(position__y=None)]), pose_meta(), ex.frames)


def test_an_empty_pose_message_is_a_value_error(ex):
    empty = pa.array([], type=pose_arrow().type)
    with pytest.raises(ValueError, match="empty"):
        on_pose(empty, pose_meta(), ex.frames)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_a_pose_with_a_non_finite_value_is_a_value_error(ex, bad):
    with pytest.raises(ValueError, match="finite"):
        on_pose(pa.array([_pose_row(position__x=bad)]), pose_meta(), ex.frames)


def _cloud(points, width, height):
    p = np.ascontiguousarray(points, dtype="<f4")
    return pa.array([{"points": p.tobytes(), "width": width, "height": height}],
                    type=cloud_to_arrow(pts()).type)


def test_a_cloud_without_points_is_a_value_error():
    a = pa.array([{"width": 1, "height": 1}],
                 type=pa.struct([("width", pa.uint32()), ("height", pa.uint32())]))
    with pytest.raises(ValueError, match="points"):
        cloud_from_arrow(a, 4)


def test_an_xyz_cloud_read_as_xyzi_is_refused_by_width_times_height():
    """4 xyz points are 12 floats, which also divide into 3 xyzi points."""
    four_xyz = np.arange(12, dtype="<f4").reshape(4, 3)
    with pytest.raises(ValueError, match=r"width\*height"):
        cloud_from_arrow(_cloud(four_xyz, 4, 1), 4)
    assert cloud_from_arrow(_cloud(four_xyz, 4, 1), 3).shape == (4, 3)


def test_an_organised_cloud_counts_width_times_height():
    grid = np.zeros((6, 4), dtype="<f4")
    assert cloud_from_arrow(_cloud(grid, 3, 2), 4).shape == (6, 4)
    with pytest.raises(ValueError, match=r"width\*height"):
        cloud_from_arrow(_cloud(grid, 3, 3), 4)


def test_a_cloud_not_divisible_by_the_stride_is_a_value_error():
    with pytest.raises(ValueError, match="whole number"):
        cloud_from_arrow(_cloud(np.zeros(5, dtype="<f4"), 1, 1), 4)


def test_an_empty_cloud_message_is_a_value_error():
    with pytest.raises(ValueError, match="empty"):
        cloud_from_arrow(pa.array([], type=cloud_to_arrow(pts()).type), 4)


# --- drop labels: bad_frame only for frame faults ------------------------------

def _on_cloud(ex, value, metadata, target="base_footprint", step=4):
    r = node_main.on_cloud(value, metadata, step=step, extrinsics=ex, target=target,
                           take=seqs(), pose=None)
    return r.moved, r.out, r.drops, r.error


def test_a_good_cloud_is_published(ex):
    moved, out, drops, error = _on_cloud(ex, cloud_to_arrow(pts()), hdr().to_metadata())
    assert moved is not None and out.frame_id == "base_footprint" and drops == {} and error is None


def test_a_stripped_frame_id_is_a_bad_frame(ex):
    m = hdr().to_metadata()
    del m["frame_id"]
    moved, _, drops, error = _on_cloud(ex, cloud_to_arrow(pts()), m)
    assert moved is None and drops == {"bad_frame": 1} and "frame_id" in str(error)


def test_an_unknown_frame_is_a_bad_frame(ex):
    moved, _, drops, _ = _on_cloud(ex, cloud_to_arrow(pts()), hdr(frame="lidar_link").to_metadata())
    assert moved is None and drops == {"bad_frame": 1}


def test_a_header_that_does_not_parse_is_a_bad_header(ex):
    m = hdr().to_metadata()
    m["validity"] = "fine"
    moved, _, drops, _ = _on_cloud(ex, cloud_to_arrow(pts()), m)
    assert moved is None and drops == {"bad_header": 1}


def test_a_misshapen_cloud_is_malformed(ex):
    four_xyz = np.zeros((4, 3), dtype="<f4")
    moved, _, drops, error = _on_cloud(ex, _cloud(four_xyz, 4, 1), hdr().to_metadata())
    assert moved is None and drops == {"malformed": 1} and "width*height" in str(error)


def test_a_cloud_without_points_is_malformed(ex):
    a = pa.array([{"width": 1, "height": 1}],
                 type=pa.struct([("width", pa.uint32()), ("height", pa.uint32())]))
    _, _, drops, _ = _on_cloud(ex, a, hdr().to_metadata())
    assert drops == {"malformed": 1}


def test_the_manifest_declares_the_point_layout_it_reads():
    import yaml
    manifest = yaml.safe_load((Path(__file__).resolve().parents[1] / "dora-node.yml").read_text())
    assert manifest["inputs"]["cloud"]["type"] == "std/media/v1/PointCloud[fields=xyzi]"
    assert manifest["outputs"]["cloud"]["type"] == "std/media/v1/PointCloud[fields=xyzi]"
    assert manifest["env"]["POINT_FIELDS"]["default"] == "xyzi"


def test_pyproject_declares_dora():
    import tomllib
    meta = tomllib.loads((Path(__file__).resolve().parents[1] / "pyproject.toml").read_text())
    assert any(d.replace(" ", "").startswith("dora-rs") for d in meta["project"]["dependencies"])


# --- one header: the Rust one ---------------------------------------------------

def test_the_node_uses_the_rust_header():
    from dora import header as hd
    assert core.Header is hd.Header
    assert not hasattr(core, "VALIDITY")


def test_the_output_header_is_derived_from_the_input(ex):
    """derive(): stamp and source come from the parent, frame and seq are the output's."""
    _, h, _ = process(pts(), hdr(stamp=5, source="lidar"), extrinsics=ex,
                      target_frame="base_footprint", take_seq=seqs())
    assert (h.frame_id, h.seq, h.stamp_ns, h.source, h.child_frame) == (
        "base_footprint", 0, 5, "lidar", None)


def test_the_pose_child_frame_is_read_from_its_header(ex):
    pose = on_pose(pose_arrow(), pose_meta(), ex.frames)
    assert pose.header.child_frame == "base_footprint"
    assert pose.transform.src == "base_footprint" and pose.transform.dst == "odom"


def test_a_plain_value_error_from_process_is_a_counted_bad_frame(ex, monkeypatch):
    def boom(*_, **__):
        raise ValueError("not a FrameError")
    monkeypatch.setattr(node_main, "process", boom)
    moved, _, drops, error = _on_cloud(ex, cloud_to_arrow(pts()), hdr().to_metadata())
    assert moved is None and drops == {"bad_frame": 1} and "not a FrameError" in str(error)


def test_on_cloud_reports_the_points_it_read(ex):
    two = np.zeros((2, 4), dtype="<f4")
    r = node_main.on_cloud(cloud_to_arrow(two), hdr().to_metadata(), step=4, extrinsics=ex,
                           target="base_footprint", take=seqs(), pose=None)
    assert r.items == 2
    bad = hdr().to_metadata()
    bad["validity"] = "fine"
    r = node_main.on_cloud(cloud_to_arrow(two), bad, step=4, extrinsics=ex,
                           target="base_footprint", take=seqs(), pose=None)
    assert r.items == 0


# --- diag: a full contract header, per-period counts ----------------------------

import json  # noqa: E402


def _diag_payload(blob):
    return json.loads(bytes(blob))


def test_diag_carries_a_full_header_of_its_own():
    d = node_main.Diag("ft", every=2)
    _, m0 = d.message(out=0, now_ns=123)
    _, m1 = d.message(out=0, now_ns=456)
    h0, h1 = Header.from_metadata(m0), Header.from_metadata(m1)
    assert (h0.frame_id, h0.seq, h0.stamp_ns, h0.source, h0.validity) == ("none", 0, 123, "ft", "ok")
    assert (h1.seq, h1.stamp_ns) == (1, 456)


def test_diag_stamps_now_by_default():
    import time
    before = time.time_ns()
    _, m = node_main.Diag("ft", every=1).message(out=0)
    assert before <= m["stamp_ns"] <= time.time_ns()


def test_diag_counts_items_bytes_and_proc_time_per_period():
    d = node_main.Diag("ft", every=2)
    d.observe(items=3, nbytes=48, proc_ns=10, drops={})
    assert not d.due()
    d.observe(items=0, nbytes=16, proc_ns=5, drops={"malformed": 1})
    assert d.due()
    blob, _ = d.message(out=1, now_ns=1)
    p = _diag_payload(blob)
    assert (p["items"], p["bytes"], p["proc_ns"]) == (3, 64, 15)
    assert (p["in"], p["out"], p["drops"]) == (2, 1, {"malformed": 1})
    d.observe(items=1, nbytes=16, proc_ns=2, drops={"bad_frame": 1})
    d.observe(items=1, nbytes=16, proc_ns=2, drops={})
    p = _diag_payload(d.message(out=2, now_ns=2)[0])
    assert (p["items"], p["bytes"], p["proc_ns"]) == (2, 32, 4), "per period, not cumulative"
    assert (p["in"], p["drops"]) == (4, {"malformed": 1, "bad_frame": 1}), "in and drops stay cumulative"


def test_diag_counts_a_bad_pose_drop():
    d = node_main.Diag("ft", every=1)
    d.drop("bad_pose")
    assert _diag_payload(d.message(out=0, now_ns=1)[0])["drops"] == {"bad_pose": 1}


def test_diag_is_an_arrow_byte_array():
    blob, _ = node_main.Diag("ft", every=1).message(out=0, now_ns=1)
    assert blob.type == pa.uint8()


def test_node_id_comes_from_the_dora_node_config(monkeypatch):
    monkeypatch.delenv("NODE_ID", raising=False)
    monkeypatch.setenv("DORA_NODE_CONFIG", "dataflow_id: 0000\nnode_id: frame_transform\nrun_config: {}\n")
    assert node_main.node_id() == "frame_transform"
    monkeypatch.setenv("NODE_ID", "forced")
    assert node_main.node_id() == "forced"


def test_node_id_is_required(monkeypatch):
    monkeypatch.delenv("NODE_ID", raising=False)
    monkeypatch.delenv("DORA_NODE_CONFIG", raising=False)
    with pytest.raises(SystemExit, match="node id"):
        node_main.node_id()
