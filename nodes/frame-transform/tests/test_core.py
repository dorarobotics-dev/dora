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
