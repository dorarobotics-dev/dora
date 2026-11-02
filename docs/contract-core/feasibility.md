## Baseline

`cargo test -p dora-message -p dora-core` on master 1016eab0a, before any change:

```
test result: ok. 378 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.18s
test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.02s
test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 190 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 1.06s
test result: ok. 5 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 7 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 24 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s
```

Totals: 619 passed, 0 failed across all test binaries of the two crates.

## Friction

- Task 5: `TypeDef.metadata` (required metadata keys) is parsed but never read anywhere in dora-core, the CLI or the daemon, so adding `frame_id`/`stamp_ns`/`child_frame` to `Image`, `PointCloud`, `Pose` and `Transform` breaks no existing test or dataflow (`examples/hub-dataflow` still validates). It also means those keys are documentation only for now: nothing checks them at validate time or on a running node. The Step 4 fallback was not needed; enforcement must come from the frame check (Task 7) or a run-time check.
