"""dora.transform: the Rust transform library seen from Python."""
from pathlib import Path

import numpy as np
import pytest

from dora import transform as tf

FIX = Path(__file__).resolve().parents[4] / "libraries/transform/tests/fixtures"


@pytest.fixture
def ex():
    frames = tf.FrameDict.load(str(FIX / "frames.yml"))
    return tf.Extrinsics.load(str(FIX / "extrinsics.yml"), frames)


def test_lookup_and_apply_packed(ex):
    t = tf.lookup(ex, "rslidar", "base_footprint")
    out = np.frombuffer(t.apply_packed(np.zeros(4, dtype="<f4").tobytes(), 4), dtype="<f4")
    assert np.allclose(out, [0.25, 0.0, 0.35, 0.0])


def test_errors_are_typed(ex):
    with pytest.raises(tf.DynamicEdgeError) as e:
        tf.lookup(ex, "rslidar", "map")
    assert len(e.value.args[1]) == 2
    with pytest.raises(tf.UnknownFrameError, match="'rslidar'"):
        tf.lookup(ex, "lidar_link", "base_link")
    assert issubclass(tf.MissingExtrinsicError, tf.FrameError)
    assert issubclass(tf.FrameError, ValueError)


def test_resolve_with_pose(ex):
    pose = tf.pose_transform([1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0], "odom", "base_footprint", ex.frames)
    t, used = tf.resolve(ex, "rslidar", "odom", pose)
    assert used
    assert np.allclose(t.apply([0.0, 0.0, 0.0]), [1.25, 2.0, 0.35])
    assert tf.resolve(ex, "rslidar", "odom", None) == (None, False)


def test_frame_kind(ex):
    assert ex.frames.kind("none") == "none"
    assert ex.frames.kind("rslidar") == "mounted"
