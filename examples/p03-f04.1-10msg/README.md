# p03-f04.1 — two-node, 10-send / 10-receive acceptance dataflow

Acceptance example for **v1.8_mod F-04.1**:

> load one launch configuration, run a Dora dataflow with **≥ 2 nodes**,
> complete **10 sample messages sent and received**, verify both counts are
> **10**, and save a run record.

It is a small, self-contained dataflow plus a runner that captures the evidence
and fails loudly unless the numbers actually add up.

**Result of the recorded run** (`evidence/run-20261126T033342Z/`):
`sent = 10`, `received = 10`, `verdict = pass`, `dora-cli 1.1.0`, run time 11 s.

## Architecture

```
dora/timer/millis/100  ──tick──▶  sender  ──message──▶  counter
                                  (sends 10)            (counts 10, verifies 1..10)
```

| Node | Kind | Role |
|------|------|------|
| `sender` | Rust binary (`nodes/src/bin/sender.rs`) | On each `tick`, emits **exactly one** `u64` message carrying sequence `1..=10`; prints `[SENT_TOTAL=10]` after the 10th actual `send_output` call |
| `counter` | Rust binary (`nodes/src/bin/counter.rs`) | Counts real `INPUT` events, prints one `[RECV]` line per message, prints `[RECV_TOTAL=10] sequence_ok=true` when 10 arrived, and re-checks the full 1..=10 sequence on `STOP` |

The timer is the built-in `dora/timer/millis/100` source, so the two **user
nodes** are `sender` and `counter`; the launch configuration is a single
`dataflow.yml`.

Both nodes are built from `nodes/` against the in-repo `dora-node-api`
(`../../../apis/rust/node`) with the same feature set as
`examples/rust-dataflow/node` (`arrow-v59`, `tracing`, `metrics`). The crate is
its own standalone workspace, so nothing in dora's root build configuration or
CI is touched.

### Why a CLI built from this checkout is required

The CLI installed in this environment (`/Users/echo/.bin/dora`,
`dora-cli 1.0.0-rc1`) is **not** built from this repository. It is
byte-identical (sha256 `5606564d…`) to
`/Volumes/EchoMacData/Dev/Dora/Dora/dora/target/release/dora`, a build of a
*different* fork (`LeonRust/dora`, version `1.0.0-rc1`). Its node↔daemon
handshake is incompatible with this checkout, so every node fails during
registration, e.g.:

```
Error: internal error
Caused by:
   0: failed to init event stream
   1: failed to send register request to dora-daemon
   2: server disconnected unexpectedly
```

The published PyPI wheels are incompatible too: `dora-rs==1.0.0rc1` fails with
`failed to deserialize DaemonReply: invalid value: integer 0, expected a
nonzero u128`, and `dora-rs==1.1.0` fails registration with `server disconnected
unexpectedly`. Captured logs are in `evidence/blocked-attempts/`.

Running the CLI built from this checkout (`cargo build -p dora-cli`, reporting
`dora-cli 1.1.0`) together with nodes built from this checkout's
`dora-node-api` is the only self-consistent combination, and it is what the
recorded run uses.

## Files

```
examples/p03-f04.1-10msg/
├── dataflow.yml            # the single launch configuration (2 nodes + timer)
├── nodes/                  # standalone Rust crate, two binaries
│   ├── Cargo.toml
│   └── src/bin/{sender,counter}.rs
├── run.sh                  # builds, runs, extracts counts, writes evidence
├── README.md
└── evidence/
    ├── run-<UTC timestamp>/ # commands.md, stdout.log, build.log,
    │                        # dataflow.yml copy, extract.md, summary.json
    └── blocked-attempts/    # why the preinstalled CLI / PyPI wheels don't work
```

## Prerequisites

- a `dora` CLI built from this checkout:

  ```bash
  cargo build -p dora-cli          # -> <repo>/target/debug/dora
  ```

  Pass it with `DORA_BIN=/path/to/dora` if it is not on `PATH`. See the section
  above for why the preinstalled CLI cannot be used.
- Rust toolchain (the workspace edition/MSRV)
- network access to crates.io for the first build only

No Python packages are required.

## Run

One command, from this directory:

```bash
DORA_BIN=<repo>/target/debug/dora ./run.sh
```

(if the CLI is on `PATH` and matches this checkout, plain `./run.sh` works).

It performs, in order:

```bash
cargo build --manifest-path nodes/Cargo.toml --bins
<repo>/target/debug/dora run dataflow.yml --stop-after 10s
```

and writes `evidence/run-<UTC timestamp>/` with the full stdout, the exact
commands, a copy of the dataflow, `extract.md` and `summary.json`. The script
exits non-zero unless `sent == received == 10` **and** the counter's sequence
check passed.

To run it by hand instead:

```bash
cargo build --manifest-path nodes/Cargo.toml --bins
<repo>/target/debug/dora run dataflow.yml --stop-after 10s
```

`--stop-after` is measured from **CLI start**, so it also covers the dataflow's
build/start phase; 10 s leaves ample room even for a cold `cargo` check. The 10
messages themselves are emitted within ~1 s of the dataflow starting (one per
100 ms tick).

### Expected output

```
stdout sender: [SENT] seq=1
stdout sender: [SENT] seq=2
...
stdout sender: [SENT] seq=10
stdout sender: [SENT_TOTAL=10]
stdout counter: [RECV] #1 seq=1
...
stdout counter: [RECV_TOTAL=10] sequence_ok=true
stdout counter: [COUNTER_DONE] received=10 expected=10 sequence_ok=true
stdout sender: [SENDER_DONE] sent=10
```

## How the counts are verified

`run.sh` does **not** hard-code the result. It parses the nodes' own stdout:

```bash
grep -o '\[SENT_TOTAL=[0-9]*\]' stdout.log | tail -1   # -> [SENT_TOTAL=10]
grep -o '\[RECV_TOTAL=[0-9]*\]' stdout.log | tail -1   # -> [RECV_TOTAL=10] sequence_ok=true
```

- `sent` is printed by `sender` after its 10th actual `send_output` call.
- `received` is printed by `counter` after counting 10 real `INPUT` events;
  the same line reports whether the received sequence was exactly `1,2,...,10`.
- `summary.json` carries `{"sent": 10, "received": 10, "verdict": "pass"}`
  plus the command, `dora` version, CLI binary path/hash, git commit, exit
  codes and durations.
- If no total line is found (fewer messages than expected, node crash, …), the
  extraction yields `0` and the verdict is `fail`.
- `summary.json` records the checkout's `git_commit` at run time. The recorded
  run was made while the example files were still untracked, so it shows the
  base commit `db6fe52`; the committed files are identical.

## Scope and boundaries

- This is an **acceptance example**, not a benchmark: it proves configuration
  loading, node startup, cross-node delivery and counted send/receive. It makes
  no latency, throughput or delivery-guarantee claims.
- The message is a single `u64` on purpose — the point is the count and the
  sequence check, not the payload.
- The timer-driven sender is throttled to 100 ms so the 10 messages are easy to
  follow in the log; nothing here is tuned for speed.
- The example deliberately stays self-contained: it adds no entry to
  `tests/example-smoke.rs` or `scripts/smoke-all.sh`, and does not modify
  dora's build configuration or CI.
- Machine-specific build artifacts (`nodes/target/`, `nodes/Cargo.lock`,
  `out/`) are git-ignored.
