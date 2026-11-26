# F-04.1 run 20261126T033342Z

Working directory: `/Volumes/EchoMacData/Dev/Dora/DoRobot-org/dora/examples/p03-f04.1-10msg`

## Commands executed

```bash
# 1. build the two nodes (Rust, dora-node-api from this checkout)
cargo build --manifest-path nodes/Cargo.toml --bins

# 2. run the dataflow locally in isolation (no coordinator)
/Volumes/EchoMacData/Dev/Dora/DoRobot-org/dora/target/debug/dora run dataflow.yml --stop-after 10s

# 3. extract the counts (the extraction is done by run.sh, see extract.md)
grep -o '\[SENT_TOTAL=[0-9]*\]' stdout.log | tail -1
grep -o '\[RECV_TOTAL=[0-9]*\]' stdout.log | tail -1
```

- dora CLI: `dora-cli 1.1.0` (binary: `/Volumes/EchoMacData/Dev/Dora/DoRobot-org/dora/target/debug/dora`)
- dataflow: `dataflow.yml` (2 nodes: sender, counter)
- expected messages: 10
