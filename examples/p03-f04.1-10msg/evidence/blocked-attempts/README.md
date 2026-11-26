# Blocked attempts — why the preinstalled CLI / PyPI wheels don't work

These logs document the dead ends hit before the example could run. They are
kept so nobody re-investigates the same protocol mismatches.

The `dora` CLI preinstalled at `/Users/echo/.bin/dora` (`dora-cli 1.0.0-rc1`)
is **not** built from this repository. It is byte-identical (sha256
`5606564d86f8554790ac1862b37e4e76d423b6c599d3c85d0344979acfb6ad1c`) to
`/Volumes/EchoMacData/Dev/Dora/Dora/dora/target/release/dora`, a build of the
`LeonRust/dora` fork at version `1.0.0-rc1` (its `Cargo.toml` says `1.0.0-rc1`,
this checkout says `1.1.0`).

| Log | What was tried | Symptom |
|-----|----------------|---------|
| `python-echo-with-installed-cli.log` | Upstream `examples/python-echo` with `dora-rs==1.0.0rc1` from PyPI, run with the preinstalled CLI | Nodes start, then receive an immediate `ERROR` event and exit; `Total PASS: 0` |
| `dora-rs-1.1.0-wheel.log` | `dora-rs==1.1.0` from PyPI with the preinstalled CLI | `RuntimeError: Could not initiate node from environment variable … failed to send register request to dora-daemon: server disconnected unexpectedly` |
| `rust-node-vs-installed-cli.log` | Node built from **this** checkout's `dora-node-api` (1.1.0), run with the preinstalled CLI | `Error: internal error … failed to send register request to dora-daemon: server disconnected unexpectedly` |

The `dora-rs==1.0.0rc1` wheel additionally fails to decode the daemon reply:

```
failed to deserialize DaemonReply
Caused by: invalid value: integer `0`, expected a nonzero u128
```

Conclusion: the node API must come from the same source tree as the CLI. The
recorded passing run uses the CLI built from this checkout
(`cargo build -p dora-cli`, `dora-cli 1.1.0`) with nodes built from the same
checkout's `dora-node-api`.
