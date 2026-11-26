#!/usr/bin/env bash
# F-04.1 acceptance runner.
#
# Runs the two-node example dataflow, captures the full stdout/stderr, extracts
# the sent/received counts from the node logs and writes evidence into
# `evidence/run-<UTC timestamp>/`.
#
# Usage:
#   ./run.sh                 # default: 10 messages, stop after 10s
#   TOTAL=10 STOP_AFTER=8s ./run.sh
#   DORA_BIN=/path/to/dora ./run.sh
#
# DORA_BIN defaults to `dora` on PATH. Use it to point at a CLI built from this
# checkout (`cargo build -p dora-cli`) when the globally installed `dora` was
# built from a different source tree; see README "Prerequisites".
#
# Exit code is 0 only when sent == received == TOTAL and the counter's
# sequence-integrity check passed.
set -euo pipefail

cd "$(dirname "$0")"

TOTAL="${TOTAL:-10}"
# `--stop-after` is measured from CLI start and therefore includes the
# dataflow's build/start phase; 10s leaves ample room for a cold cargo check.
STOP_AFTER="${STOP_AFTER:-10s}"
DORA_BIN="${DORA_BIN:-dora}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="evidence/run-${RUN_ID}"
mkdir -p "$RUN_DIR"

# Cargo needs a writable CARGO_HOME. Respect an existing one, fall back to the
# standard location.
CARGO_HOME="${CARGO_HOME:-${HOME}/.cargo}"
export CARGO_HOME

# The dataflow's `build:` step (and the pre-build below) run `cargo build`
# inside this checkout; nothing is downloaded from git, only crates.io.
echo "==> [1/4] building example nodes (cargo build --manifest-path nodes/Cargo.toml --bins)"
BUILD_START=$(date +%s)
cargo build --manifest-path nodes/Cargo.toml --bins 2>&1 | tee "$RUN_DIR/build.log"
BUILD_SECONDS=$(( $(date +%s) - BUILD_START ))

# Keep an exact copy of the launch configuration that was executed.
cp dataflow.yml "$RUN_DIR/dataflow.yml"

DORA_VERSION="$("$DORA_BIN" --version 2>&1 | head -1 || true)"
DORA_BIN_PATH="$(command -v "$DORA_BIN" 2>/dev/null || true)"
DORA_BIN_SHA="$( [ -n "$DORA_BIN_PATH" ] && shasum -a 256 "$DORA_BIN_PATH" 2>/dev/null | cut -d' ' -f1 || true)"

cat > "$RUN_DIR/commands.md" <<EOF
# F-04.1 run ${RUN_ID}

Working directory: \`$(pwd)\`

## Commands executed

\`\`\`bash
# 1. build the two nodes (Rust, dora-node-api from this checkout)
cargo build --manifest-path nodes/Cargo.toml --bins

# 2. run the dataflow locally in isolation (no coordinator)
${DORA_BIN} run dataflow.yml --stop-after ${STOP_AFTER}

# 3. extract the counts (the extraction is done by run.sh, see extract.md)
grep -o '\[SENT_TOTAL=[0-9]*\]' stdout.log | tail -1
grep -o '\[RECV_TOTAL=[0-9]*\]' stdout.log | tail -1
\`\`\`

- dora CLI: \`${DORA_VERSION}\` (binary: \`${DORA_BIN_PATH:-${DORA_BIN}}\`)
- dataflow: \`dataflow.yml\` (2 nodes: sender, counter)
- expected messages: ${TOTAL}
EOF

echo "==> [2/4] running: ${DORA_BIN} run dataflow.yml --stop-after ${STOP_AFTER}"
RUN_START=$(date +%s)
set +e
"$DORA_BIN" run dataflow.yml --stop-after "$STOP_AFTER" 2>&1 | tee "$RUN_DIR/stdout.log"
DORA_EXIT=${PIPESTATUS[0]}
set -e
RUN_SECONDS=$(( $(date +%s) - RUN_START ))

echo "==> [3/4] extracting sent/received counts from $RUN_DIR/stdout.log"

python3 - "$RUN_DIR" "$TOTAL" "$DORA_EXIT" "$BUILD_SECONDS" "$RUN_SECONDS" "$DORA_VERSION" "$RUN_ID" "$DORA_BIN" "${DORA_BIN_PATH:-}" "${DORA_BIN_SHA:-}" "$STOP_AFTER" <<'PY'
import json
import os
import platform
import re
import subprocess
import sys

(
    run_dir,
    total,
    dora_exit,
    build_seconds,
    run_seconds,
    dora_version,
    run_id,
    dora_bin,
    dora_bin_path,
    dora_bin_sha,
    stop_after,
) = sys.argv[1:12]
total = int(total)
log_path = os.path.join(run_dir, "stdout.log")
log = open(log_path, encoding="utf-8", errors="replace").read()

sent_hits = re.findall(r"\[SENT_TOTAL=(\d+)\]", log)
recv_hits = re.findall(r"\[RECV_TOTAL=(\d+)\]", log)
seq_hits = re.findall(r"\[RECV_TOTAL=\d+\]\s+sequence_ok=(true|false)", log)

sent = int(sent_hits[-1]) if sent_hits else 0
received = int(recv_hits[-1]) if recv_hits else 0
sequence_ok = bool(seq_hits) and seq_hits[-1] == "true"

try:
    git_commit = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"], text=True, stderr=subprocess.DEVNULL
    ).strip()
except Exception:
    git_commit = "unknown"

verdict = "pass" if (sent == total and received == total and sequence_ok) else "fail"

summary = {
    "run_id": run_id,
    "timestamp_utc": run_id[:4] + "-" + run_id[4:6] + "-" + run_id[6:8]
    + "T" + run_id[9:11] + ":" + run_id[11:13] + ":" + run_id[13:15] + "Z",
    "command": f"{dora_bin} run dataflow.yml --stop-after {stop_after}",
    "dataflow": "dataflow.yml",
    "nodes": ["sender", "counter"],
    "dora_version": dora_version,
    "dora_bin": dora_bin_path or dora_bin,
    "dora_bin_sha256": dora_bin_sha or None,
    "git_commit": git_commit,
    "expected": total,
    "sent": sent,
    "received": received,
    "sequence_ok": sequence_ok,
    "verdict": verdict,
    "dora_exit_code": int(dora_exit),
    "build_seconds": int(build_seconds),
    "run_seconds": int(run_seconds),
    "host": f"{platform.system()} {platform.machine()} {platform.release()}",
    "python_executable": sys.executable,
    "evidence": {
        "stdout": "stdout.log",
        "build": "build.log",
        "dataflow_copy": "dataflow.yml",
        "commands": "commands.md",
        "extraction": "extract.md",
    },
}

with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2)
    f.write("\n")

extract_md = f"""# Count extraction — {run_id}

The counts come from the nodes' own stdout, captured verbatim in
`stdout.log`. They are not written by `run.sh` itself and not hard-coded
anywhere: `sender` prints `[SENT_TOTAL=<n>]` after its n-th actual
`send_output` call, `counter` prints `[RECV_TOTAL=<n>] sequence_ok=<bool>`
after receiving that many `INPUT` events (plus a 1..=n sequence check).

Equivalent manual extraction:

```bash
grep -o '\\[SENT_TOTAL=[0-9]*\\]' stdout.log | tail -1   # -> [SENT_TOTAL={sent}]
grep -o '\\[RECV_TOTAL=[0-9]*\\]' stdout.log | tail -1   # -> [RECV_TOTAL={received}]
grep 'sequence_ok' stdout.log                            # sequence integrity
```

Raw lines found in `stdout.log`:

```
{chr(10).join(line for line in log.splitlines() if "[SENT_TOTAL=" in line or "[RECV_TOTAL=" in line or "[COUNTER_DONE]" in line or "[SENDER_DONE]" in line) or "(none)"}
```

Verdict: **{verdict}** (sent={sent}, received={received}, expected={total}, sequence_ok={sequence_ok}).
"""
with open(os.path.join(run_dir, "extract.md"), "w", encoding="utf-8") as f:
    f.write(extract_md)

print(json.dumps(summary, indent=2))
with open(os.path.join(run_dir, ".verdict"), "w") as f:
    f.write(verdict)
PY

VERDICT="$(cat "$RUN_DIR/.verdict")"
rm -f "$RUN_DIR/.verdict"

echo "==> [4/4] evidence written to $RUN_DIR"
echo "    verdict: ${VERDICT}"

if [ "$VERDICT" != "pass" ]; then
    echo "F-04.1 FAILED: expected sent=received=${TOTAL} with sequence_ok=true; see $RUN_DIR/summary.json" >&2
    exit 1
fi

echo "F-04.1 PASS: sent=${TOTAL}, received=${TOTAL}, sequence_ok=true"
