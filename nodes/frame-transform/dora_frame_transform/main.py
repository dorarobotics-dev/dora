"""The dora node loop. Configuration is read and checked before the first message."""
import json
import os
import re
import time
from collections import Counter
from typing import NamedTuple, Optional

import pyarrow as pa
from dora import Node
from dora import transform as tf
from dora.header import Header

from dora_frame_transform.core import cloud_from_arrow, cloud_to_arrow, on_pose, process

ITEM_STEP = {"xyz": 3, "xyzi": 4}
DIAG_FRAME = "none"


def config():
    frames = tf.FrameDict.load(os.environ["DORA_FRAMES"])
    extrinsics = tf.Extrinsics.load(os.environ["DORA_EXTRINSICS"], frames)
    target = os.environ["TARGET_FRAME"]
    if frames.kind(target) == "none":
        raise SystemExit("TARGET_FRAME is 'none', which has no spatial meaning")
    fields = os.environ.get("POINT_FIELDS", "xyzi")
    if fields not in ITEM_STEP:
        raise SystemExit(f"POINT_FIELDS must be one of {sorted(ITEM_STEP)}, got {fields!r}")
    return extrinsics, target, ITEM_STEP[fields], int(os.environ.get("DIAG_EVERY", "10"))


def node_id() -> str:
    """This node's dataflow id, the `source` of its diag messages. dora puts it
    in DORA_NODE_CONFIG for every node it spawns; NODE_ID overrides it."""
    forced = os.environ.get("NODE_ID")
    if forced:
        return forced
    found = re.search(r"^node_id:\s*['\"]?([^'\"\s]+)", os.environ.get("DORA_NODE_CONFIG", ""), re.M)
    if not found:
        raise SystemExit("cannot find this node's node id: neither DORA_NODE_CONFIG nor NODE_ID "
                         "names it, so its diagnostics would be unattributable")
    return found.group(1)


class Diag:
    """The `diag` output: a contract header of its own (frame `none`, its own
    seq, stamped now, source = this node) over JSON counters. `items` (points
    read), `bytes` (input bytes) and `proc_ns` cover the period since the last
    message; `in`, `out` and `drops` are running totals."""

    def __init__(self, source: str, every: int):
        self.source, self.every = source, every
        self.n_in, self.seq, self.drops = 0, 0, Counter()
        self.items = self.bytes = self.proc_ns = 0

    def observe(self, *, items: int, nbytes: int, proc_ns: int, drops) -> None:
        self.n_in += 1
        self.items += items
        self.bytes += nbytes
        self.proc_ns += proc_ns
        self.drops.update(drops)

    def drop(self, label: str) -> None:
        self.drops[label] += 1

    def due(self) -> bool:
        return self.n_in > 0 and self.n_in % self.every == 0

    def message(self, *, out: int, now_ns: Optional[int] = None):
        payload = {"in": self.n_in, "out": out, "drops": dict(self.drops),
                   "items": self.items, "bytes": self.bytes, "proc_ns": self.proc_ns}
        header = Header(DIAG_FRAME, self.seq, time.time_ns() if now_ns is None else now_ns, self.source)
        self.seq += 1
        self.items = self.bytes = self.proc_ns = 0
        blob = json.dumps(payload).encode()
        return pa.array(list(blob), type=pa.uint8()), header.to_metadata()


class CloudResult(NamedTuple):
    moved: object
    out: Optional[Header]
    drops: dict
    error: Optional[Exception]
    items: int = 0


def on_cloud(value, metadata, *, step, extrinsics, target, take, pose) -> CloudResult:
    """One cloud off the wire. Every fault is a counted drop under its own
    label: `bad_frame` for frame faults (no or unknown frame_id, no route),
    `bad_header` for a header that does not parse, `malformed` for a cloud
    that does not decode. `items` is the number of points read."""
    frame = metadata.get("frame_id") if isinstance(metadata, dict) else None
    if not isinstance(frame, str) or not frame:
        return CloudResult(None, None, {"bad_frame": 1}, ValueError(f"cloud carries no frame_id (got {frame!r})"))
    try:
        header = Header.from_metadata(metadata)
    except ValueError as error:
        return CloudResult(None, None, {"bad_header": 1}, error)
    try:
        points = cloud_from_arrow(value, step)
    except ValueError as error:
        return CloudResult(None, None, {"malformed": 1}, error)
    try:
        moved, out, drops = process(points, header, extrinsics=extrinsics, target_frame=target,
                                    take_seq=take, pose=pose)
    except ValueError as error:  # tf.FrameError is a ValueError
        return CloudResult(None, None, {"bad_frame": 1}, error, len(points))
    return CloudResult(moved, out, drops, None, len(points))


def main():
    extrinsics, target, step, diag_every = config()
    diag = Diag(node_id(), diag_every)
    node = Node()
    pose, seq = None, 0
    logged = set()

    def take():
        nonlocal seq
        seq += 1
        return seq - 1

    for event in node:
        if event["type"] != "INPUT":
            continue
        if event["id"] == "pose":
            try:
                pose = on_pose(event["value"], event["metadata"], extrinsics.frames)
            except ValueError:
                diag.drop("bad_pose")
            continue
        if event["id"] != "cloud":
            continue
        started = time.perf_counter_ns()
        r = on_cloud(event["value"], event["metadata"], step=step,
                     extrinsics=extrinsics, target=target, take=take, pose=pose)
        proc_ns = time.perf_counter_ns() - started
        if r.error is not None:
            for label in r.drops.keys() - logged:  # log the first drop of each kind
                node.log("info", f"dropping clouds as {label}: {r.error}")
                logged.add(label)
        if r.moved is not None:
            node.send_output("cloud", cloud_to_arrow(r.moved), r.out.to_metadata())
        diag.observe(items=r.items, nbytes=getattr(event["value"], "nbytes", 0),
                     proc_ns=proc_ns, drops=r.drops)
        if diag.due():
            blob, metadata = diag.message(out=seq)
            node.send_output("diag", blob, metadata)


if __name__ == "__main__":
    main()
