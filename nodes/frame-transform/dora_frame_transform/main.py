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


def main():
    extrinsics, target, step, diag_every = config()
    node = Node()
    pose, seq, n_in = None, 0, 0
    drops, logged = Counter(), False

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
        try:
            header = Header.from_metadata(event["metadata"])
            moved, out, d = process(cloud_from_arrow(event["value"], step), header,
                                    extrinsics=extrinsics, target_frame=target, take_seq=take, pose=pose)
        except ValueError as error:  # tf.FrameError is a ValueError
            moved, out, d = None, None, {"bad_frame": 1}
            if not logged:
                node.log("info", f"dropping clouds as bad_frame: {error}")
                logged = True
        drops.update(d)
        if moved is not None:
            node.send_output("cloud", cloud_to_arrow(moved), out.to_metadata())
        if n_in % diag_every == 0:
            blob = json.dumps({"in": n_in, "out": seq, "drops": dict(drops),
                               "proc_ns": time.perf_counter_ns() - started}).encode()
            node.send_output("diag", pa.array(list(blob), type=pa.uint8()), {"frame_id": "none"})


if __name__ == "__main__":
    main()
