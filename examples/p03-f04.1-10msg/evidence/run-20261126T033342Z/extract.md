# Count extraction — 20261126T033342Z

The counts come from the nodes' own stdout, captured verbatim in
`stdout.log`. They are not written by `run.sh` itself and not hard-coded
anywhere: `sender` prints `[SENT_TOTAL=<n>]` after its n-th actual
`send_output` call, `counter` prints `[RECV_TOTAL=<n>] sequence_ok=<bool>`
after receiving that many `INPUT` events (plus a 1..=n sequence check).

Equivalent manual extraction:

```bash
grep -o '\[SENT_TOTAL=[0-9]*\]' stdout.log | tail -1   # -> [SENT_TOTAL=10]
grep -o '\[RECV_TOTAL=[0-9]*\]' stdout.log | tail -1   # -> [RECV_TOTAL=10]
grep 'sequence_ok' stdout.log                            # sequence integrity
```

Raw lines found in `stdout.log`:

```
11:33:48 stdout sender: [SENT_TOTAL=10]
11:33:48 stdout counter: [RECV_TOTAL=10] sequence_ok=true
11:33:54 stdout sender: [SENDER_DONE] sent=10
11:33:54 stdout counter: [COUNTER_DONE] received=10 expected=10 sequence_ok=true
```

Verdict: **pass** (sent=10, received=10, expected=10, sequence_ok=True).
