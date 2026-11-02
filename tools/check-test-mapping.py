#!/usr/bin/env python3
"""Every reference test in the slice's files must appear in test-mapping.md,
mapped to a new test or marked n/a with a reason."""
import pathlib
import re
import subprocess
import sys

REF = pathlib.Path("/home/demo/Public/github_dimos/open-robot-stack-operator-contract")
PYTHON = "/home/demo/Public/github_dimos/.venv-exp/bin/python"
FILES = [
    "packages/open-robot-operator/tests/test_envelope.py",
    "packages/open-robot-operator/tests/test_derive.py",
    "packages/open-robot-operator/tests/test_frames.py",
    "packages/open-robot-operator/tests/test_transform.py",
    "packages/open-robot-operator/tests/test_resolve.py",
    "packages/open-robot-operator/tests/test_validate.py",
    "packages/open-robot-operator/tests/test_wire.py",
    "capabilities/perception/tests/test_frame_transform.py",
]
MAPPING = pathlib.Path(__file__).resolve().parents[1] / "docs/contract-core/test-mapping.md"


def reference_tests():
    # --rootdir keeps every node id relative to REF; without it the perception
    # pack's conftest makes pytest print its ids without the file path.
    out = subprocess.run(
        [PYTHON, "-m", "pytest", "--collect-only", "-q", "--rootdir", str(REF), *FILES],
        cwd=REF, capture_output=True, text=True,
        env={"PYTHONPATH": "packages/open-robot-operator:capabilities/perception:capabilities/data-processing",
             "PATH": "/usr/bin:/bin"})
    if out.returncode != 0:
        sys.exit(f"collecting the reference tests failed:\n{out.stdout}\n{out.stderr}")
    return sorted({line.split("[")[0] for line in out.stdout.splitlines() if "::" in line})


def main():
    text = MAPPING.read_text()
    rows = {}
    for line in text.splitlines():
        m = re.match(r"\|\s*`([^`]+)`\s*\|\s*(.+?)\s*\|", line)
        if m:
            rows[m.group(1)] = m.group(2)
    tests = reference_tests()
    missing = [t for t in tests if t not in rows]
    empty = [t for t, v in rows.items() if not v or v in ("-", "TBD")]
    stray = [t for t in rows if t not in tests]
    for t in missing:
        print(f"UNMAPPED {t}")
    for t in empty:
        print(f"EMPTY    {t}")
    for t in stray:
        print(f"STRAY    {t}")
    na = sum(1 for t in tests if rows.get(t, "").startswith("n/a"))
    print(f"{len(rows)} mapped ({len(rows) - na} to tests, {na} n/a), "
          f"{len(missing)} unmapped, {len(empty)} empty, {len(stray)} stray")
    return 1 if missing or empty or stray else 0


if __name__ == "__main__":
    sys.exit(main())
