"""The dora node loop. Configuration is read and checked before the first message."""
import json
import os
import time
from collections import Counter

import pyarrow as pa
from dora import Node
from dora import transform as tf

from dora_frame_transform.core import Header, cloud_from_arrow, cloud_to_arrow, on_pose, process

ITEM_STEP = {"xyz": 3, "xyzi": 4}


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


def on_cloud(value, metadata, *, step, extrinsics, target, take, pose):
    """One cloud off the wire -> (moved, out_header, drops, error). Every fault
    is a counted drop under its own label: `bad_frame` for frame faults (no or
    unknown frame_id, no route), `bad_header` for a header that does not parse,
    `malformed` for a cloud that does not decode."""
    frame = metadata.get("frame_id") if isinstance(metadata, dict) else None
    if not isinstance(frame, str) or not frame:
        return None, None, {"bad_frame": 1}, ValueError(f"cloud carries no frame_id (got {frame!r})")
    try:
        header = Header.from_metadata(metadata)
    except ValueError as error:
        return None, None, {"bad_header": 1}, error
    try:
        points = cloud_from_arrow(value, step)
    except ValueError as error:
        return None, None, {"malformed": 1}, error
    try:
        moved, out, drops = process(points, header, extrinsics=extrinsics, target_frame=target,
                                    take_seq=take, pose=pose)
    except tf.FrameError as error:
        return None, None, {"bad_frame": 1}, error
    return moved, out, drops, None


def main():
    extrinsics, target, step, diag_every = config()
    node = Node()
    pose, seq, n_in = None, 0, 0
    drops, logged = Counter(), set()

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
                drops["bad_pose"] += 1
            continue
        if event["id"] != "cloud":
            continue
        n_in += 1
        started = time.perf_counter_ns()
        moved, out, d, error = on_cloud(event["value"], event["metadata"], step=step,
                                        extrinsics=extrinsics, target=target, take=take, pose=pose)
        if error is not None:
            for label in d.keys() - logged:  # log the first drop of each kind
                node.log("info", f"dropping clouds as {label}: {error}")
                logged.add(label)
        drops.update(d)
        if moved is not None:
            node.send_output("cloud", cloud_to_arrow(moved), out.to_metadata())
        if n_in % diag_every == 0:
            blob = json.dumps({"in": n_in, "out": seq, "drops": dict(drops),
                               "proc_ns": time.perf_counter_ns() - started}).encode()
            node.send_output("diag", pa.array(list(blob), type=pa.uint8()), {"frame_id": "none"})


if __name__ == "__main__":
    main()
