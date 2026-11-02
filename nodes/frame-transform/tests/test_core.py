from pathlib import Path

import numpy as np
import pyarrow as pa
import pytest

from dora import transform as tf
from dora_frame_transform.core import (
    POSE_MAX_AGE_MS, Header, Pose, cloud_from_arrow, cloud_to_arrow, on_pose, process,
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
