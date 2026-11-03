#!/usr/bin/env python3
"""Apply one mutation at a time, run every suite, restore, report.

A mutation no test notices is a hole in the suite. Every suite runs for every
mutation and the per-suite result is printed, so "suite X catches it" is
evidence, not inference. A mutation is caught when a suite's tests fail; a
suite that does not build is reported as `build` and does not count. Some
mutations must be caught by a named suite (REQUIRED). Nothing else may run
tests in this copy while this runs. Exit status is non-zero if any mutation
survives, only breaks the build, or misses its required suite.
"""
import os
import pathlib
import re
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
    ("D1. the node builds its output header by hand, always ok, bypassing derive", FT,
     "    out = derive([header], target_frame, take_seq(), validity, reason)",
     '    out = Header(target_frame, take_seq(), header.stamp_ns, header.source, "ok")'),
    ("14. a cloud composed with a stale pose is forwarded as ok", FT,
     '        return "stale", "stale_pose"', "        return \"ok\", None"),
    ("15. the validator skips the transform check", "libraries/core/src/descriptor/frames_check.rs",
     "                let failure = match ex.route(src, dst) {", "                let failure = match Ok::<(), FrameError>(()) {"),
    ("16. a node may launch with a different calibration than was checked", "libraries/core/src/descriptor/frames_check.rs",
     "    if launched != checked {", "    if false && launched != checked {"),
    ("F1. a node may launch with a different frame dictionary than was checked", "libraries/core/src/descriptor/frames_check.rs",
     "            (FRAMES_ENV, &dict_path, \"frame dictionary\"),\n", ""),
    ("P1. a pose in another frame than the dynamic edge's parent covers it", "libraries/core/src/descriptor/frames_check.rs",
     "            Ok(frame) if frame != \"any\" && frame != parent => {", "            Ok(frame) if false && frame != \"any\" && frame != parent => {"),
]

SUITES = {
    "rust-header": ["cargo", "test", "-q", "-p", "dora-message", "header::"],
    "rust-transform": ["cargo", "test", "-q", "-p", "dora-transform"],
    "rust-frames-check": ["cargo", "test", "-q", "-p", "dora-core", "--test", "frames_check"],
    "py-binding": ["bash", "-c", f"cd apis/python/node && {VENV}/bin/maturin develop -q --release && "
                                 f"{VENV_PY} -m pytest -q -p no:cacheprovider tests/test_transform.py tests/test_header.py"],
    "py-node": ["bash", "-c", f"cd nodes/frame-transform && {VENV_PY} -B -m pytest -q -p no:cacheprovider tests"],
}

# Mutations a specific suite must catch (label prefix -> suite).
REQUIRED = {
    "H1.": "py-binding",
    "H2.": "py-binding",
}

BUILD_ERROR = re.compile(r"error\[E\d{4}\]|error: could not compile|maturin failed")


def run(cmd):
    return subprocess.run(cmd, cwd=ROOT, env=ENV, capture_output=True, text=True)


def outcome(cmd) -> str:
    """`pass`, `FAIL` (tests failed) or `build` (the suite did not build)."""
    r = run(cmd)
    if r.returncode == 0:
        return "pass"
    return "build" if BUILD_ERROR.search(r.stdout + r.stderr) else "FAIL"


def main() -> int:
    if "--baseline" in sys.argv:
        for name, cmd in SUITES.items():
            r = run(cmd)
            print(f"rc={r.returncode} {name}")
            if r.returncode:
                print(r.stdout[-2000:], r.stderr[-2000:])
                return 1
        return 0
    problems = []
    try:
        for label, rel, anchor, replacement in MUTATIONS:
            path = ROOT / rel
            original = path.read_text()
            if original.count(anchor) != 1:
                print(f"ANCHOR {label}: found {original.count(anchor)} times in {rel}")
                return 2
            path.write_text(original.replace(anchor, replacement))
            try:
                results = {name: outcome(cmd) for name, cmd in SUITES.items()}
            finally:
                path.write_text(original)
            caught_by = [n for n, r in results.items() if r == "FAIL"]
            required = next((s for p, s in REQUIRED.items() if label.startswith(p)), None)
            if not caught_by:
                verdict = "BUILD   " if "build" in results.values() else "SURVIVED"
            elif required and required not in caught_by:
                verdict = "MISSED  "
            else:
                verdict = "caught  "
            print(f"{verdict} {label}", flush=True)
            print("         " + "  ".join(f"{n}={r}" for n, r in results.items())
                  + (f"  (requires {required})" if required else ""), flush=True)
            if verdict != "caught  ":
                problems.append(f"{verdict.strip()} {label}")
    finally:
        # rebuild the unmutated binding so the venv is left clean
        run(SUITES["py-binding"])
    for p in problems:
        print(p)
    print(f"{len(problems)}/{len(MUTATIONS)} not caught as required")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
