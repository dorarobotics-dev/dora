"""Change the frame a point cloud is expressed in, explicitly and checked.

`dora validate` has already confirmed that the calibration holds a route from
the input frame to TARGET_FRAME, or that a pose input is wired when the route
crosses one dynamic edge. Only x, y, z move; intensity, capture stamp and
source pass through, because the cloud still describes the same instant from
the same sensor.
"""
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import numpy as np
import pyarrow as pa

from dora import transform as tf

VALIDITY = ("ok", "missing", "stale", "unpaired", "low_confidence", "rejected")
POSE_MAX_AGE_MS = 100


@dataclass(frozen=True)
class Header:
    frame_id: str
    seq: int
    stamp_ns: int
    source: str
    validity: str = "ok"
    reason: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError(f"frame_id must be a non-empty str, got {self.frame_id!r}")
        if not isinstance(self.source, str) or not self.source:
            raise ValueError(f"source must be a non-empty str, got {self.source!r}")
        for name, value, floor in (("seq", self.seq, 0), ("stamp_ns", self.stamp_ns, 1)):
            if isinstance(value, bool) or not isinstance(value, int) or value < floor:
                raise ValueError(f"{name} must be an int >= {floor}, got {value!r}")
        if self.validity not in VALIDITY:
            raise ValueError(f"unknown validity {self.validity!r}; expected one of {VALIDITY}")
        if self.validity == "ok" and self.reason is not None:
            raise ValueError("reason must be None when validity is ok")
        if self.validity != "ok" and not self.reason:
            raise ValueError(f"validity {self.validity!r} requires a non-empty reason")

    def to_metadata(self) -> dict:
        m = {"frame_id": self.frame_id, "header_seq": self.seq, "stamp_ns": self.stamp_ns,
             "source": self.source, "validity": self.validity}
        if self.reason is not None:
            m["reason"] = self.reason
        return m

    @classmethod
    def from_metadata(cls, m) -> "Header":
        if not isinstance(m, dict):
            raise ValueError(f"metadata must be a mapping, got {m!r}")
        missing = [k for k in ("frame_id", "header_seq", "stamp_ns", "source", "validity") if k not in m]
        if missing:
            raise ValueError(f"metadata carries no contract header; missing {missing}")
        return cls(m["frame_id"], m["header_seq"], m["stamp_ns"], m["source"], m["validity"], m.get("reason"))


@dataclass(frozen=True)
class Pose:
    transform: "tf.Transform"
    header: Header


def cloud_from_arrow(value, item_step: int) -> np.ndarray:
    row = value[0].as_py() if isinstance(value, (pa.Array, pa.ChunkedArray)) else value
    flat = np.frombuffer(row["points"], dtype="<f4")
    if flat.size % item_step:
        raise ValueError(f"{flat.size} floats is not a whole number of {item_step}-float points")
    return flat.reshape(-1, item_step)


def cloud_to_arrow(points: np.ndarray) -> pa.Array:
    p = np.ascontiguousarray(points, dtype="<f4")
    return pa.array([{"points": p.tobytes(), "width": len(p), "height": 1}],
                    type=pa.struct([("points", pa.large_binary()), ("width", pa.uint32()), ("height", pa.uint32())]))


def on_pose(value, metadata, frames) -> Pose:
    """One pose off the wire. Raises ValueError on a bad message; the caller
    counts a `bad_pose` drop and keeps the previous pose."""
    child = (metadata or {}).get("child_frame")
    if not isinstance(child, str) or not child:
        raise ValueError("a pose message needs a 'child_frame' metadata entry naming the frame whose pose it is")
    header = Header.from_metadata(metadata)
    row = value[0].as_py()
    p, q = row["position"], row["orientation"]
    values = [p["x"], p["y"], p["z"], q["x"], q["y"], q["z"], q["w"]]  # read by name, never by position
    return Pose(tf.pose_transform(values, header.frame_id, child, frames), header)


def _pose_quality(cloud: Header, pose: Pose, max_age_ms: int) -> Tuple[str, Optional[str]]:
    if pose.header.validity != "ok":
        return pose.header.validity, f"pose_{pose.header.reason}"
    if abs(cloud.stamp_ns - pose.header.stamp_ns) > max_age_ms * 1_000_000:
        return "stale", "stale_pose"
    return "ok", None


def process(points: np.ndarray, header: Header, *, extrinsics, target_frame: str,
            take_seq: Callable[[], int], pose: Optional[Pose] = None,
            max_age_ms: int = POSE_MAX_AGE_MS) -> Tuple[Optional[np.ndarray], Optional[Header], Dict[str, int]]:
    """Transform one cloud. A cloud that cannot be placed in `target_frame` is a
    counted drop, never published under a frame it is not in. Raises
    tf.FrameError on an unknown frame (the caller counts `bad_frame`)."""
    transform, used_pose = tf.resolve(extrinsics, header.frame_id, target_frame,
                                      pose.transform if pose else None)
    if transform is None:
        return None, None, {"no_pose": 1}
    validity, reason = header.validity, header.reason
    if used_pose and validity == "ok":
        validity, reason = _pose_quality(header, pose, max_age_ms)
    out = Header(target_frame, take_seq(), header.stamp_ns, header.source, validity, reason)
    step = points.shape[1]
    moved = np.frombuffer(transform.apply_packed(np.ascontiguousarray(points, dtype="<f4").tobytes(), step),
                          dtype="<f4").reshape(-1, step)
    return moved, out, {}
