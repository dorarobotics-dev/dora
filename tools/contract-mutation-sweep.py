#!/usr/bin/env python3
"""Apply one mutation at a time, run the suites, restore, report.

A mutation no test notices is a hole in the suite. Nothing else may run tests
in this copy while this runs. Exit status is non-zero if any mutation survives.
"""
import os
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
VENV = "/home/demo/Public/github_dimos/.venv-exp"
VENV_PY = f"{VENV}/bin/python"
FT = "nodes/frame-transform/dora_frame_transform/core.py"

ENV = {k: v for k, v in os.environ.items() if k != "CONDA_PREFIX"}
ENV.update(PYTHONPATH="", PYO3_PYTHON=VENV_PY, VIRTUAL_ENV=VENV)

MUTATIONS = [
    ("3. output publishes its input's frame", FT,
     "    out = derive([header], target_frame,", "    out = derive([header], header.frame_id,"),
    ("4. an upgrade is granted but never counted", "libraries/message/src/header.rs",
     "        (up.count)(up.reason);", "        let _ = up;"),
    ("7. a degraded validity is laundered into ok", FT,
     "    validity, reason = header.validity, header.reason", '    validity, reason = "ok", None'),
    ("H1. a degraded validity needs no reason", "libraries/message/src/header.rs",
     "            (v, None) if *v != Validity::Ok => {", "            (v, None) if false && *v != Validity::Ok => {"),
    ("H2. a bool is accepted as an int", "libraries/message/src/header.rs",
     "            Some(Parameter::Integer(i)) => Ok(*i),",
     "            Some(Parameter::Integer(i)) => Ok(*i),\n            Some(Parameter::Bool(b)) => Ok(i64::from(*b)),"),
    ("10. a missing extrinsic is quietly treated as identity", "libraries/transform/src/extrinsics.rs",
     "                Some(edge) => out.push(edge.clone()),\n                None => {",
     "                Some(edge) => out.push(edge.clone()),\n                None if true => continue,\n                None => {"),
    ("11. transforms compose even when their frames do not meet", "libraries/transform/src/rigid.rs",
     "        if self.dst != other.src {", "        if false && self.dst != other.src {"),
    ("12. rotation_xyzw is read as w, x, y, z", "libraries/transform/src/rigid.rs",
     "    let [x, y, z, w] = q;", "    let [w, x, y, z] = q;"),
    ("13. a calibration file may carry a dynamic edge", "libraries/transform/src/extrinsics.rs",
     "            if frame.kind != Kind::Mounted {", "            if frame.kind != Kind::Mounted && frame.kind != Kind::Dynamic {"),
    ("14. a cloud composed with a stale pose is forwarded as ok", FT,
     '        return "stale", "stale_pose"', "        return \"ok\", None"),
    ("15. the validator skips the transform check", "libraries/core/src/descriptor/frames_check.rs",
     "                match ex.route(src, dst) {", "                match Ok::<(), FrameError>(()) {"),
    ("16. a node may launch with a different calibration than was checked", "libraries/core/src/descriptor/frames_check.rs",
     "    if launched != checked {", "    if false && launched != checked {"),
]

SUITES = [
    ["cargo", "test", "-q", "-p", "dora-message", "header::"],
    ["cargo", "test", "-q", "-p", "dora-transform"],
    ["cargo", "test", "-q", "-p", "dora-core", "--test", "frames_check"],
    ["bash", "-c", f"cd apis/python/node && {VENV}/bin/maturin develop -q --release && "
                   f"{VENV_PY} -m pytest -q -p no:cacheprovider tests/test_transform.py tests/test_header.py"],
    ["bash", "-c", f"cd nodes/frame-transform && {VENV_PY} -B -m pytest -q -p no:cacheprovider tests"],
]


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, env=ENV, capture_output=True, text=True)


def caught() -> bool:
    for cmd in SUITES:
        if run(cmd).returncode != 0:
            return True
    return False


def main() -> int:
    if "--baseline" in sys.argv:
        for cmd in SUITES:
            r = run(cmd)
            print(f"rc={r.returncode} {' '.join(cmd)[:90]}")
            if r.returncode:
                print(r.stdout[-2000:], r.stderr[-2000:])
                return 1
        return 0
    survivors = []
    try:
        for label, rel, anchor, replacement in MUTATIONS:
            path = ROOT / rel
            original = path.read_text()
            if original.count(anchor) != 1:
                print(f"ANCHOR {label}: found {original.count(anchor)} times in {rel}")
                return 2
            path.write_text(original.replace(anchor, replacement))
            try:
                ok = caught()
            finally:
                path.write_text(original)
            print(f"{'caught  ' if ok else 'SURVIVED'} {label}", flush=True)
            if not ok:
                survivors.append(label)
    finally:
        # rebuild the unmutated binding so the venv is left clean
        run(SUITES[3])
    print(f"{len(survivors)}/{len(MUTATIONS)} survive")
    return 1 if survivors else 0


if __name__ == "__main__":
    sys.exit(main())
