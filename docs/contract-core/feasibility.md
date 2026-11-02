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
- Task 10: the node manifest validator refuses env entries whose names start with `DORA_` (`ENV_DENY_PREFIX` in `libraries/core/src/manifest/validate.rs`: "security-sensitive and cannot be declared by a node manifest"). Deviation 5 puts calibration in `DORA_FRAMES` and `DORA_EXTRINSICS`, so `dora-node.yml` cannot document them in `env:`; they are noted in a YAML comment and the dataflow sets them. Either the contract needs another prefix (e.g. `CONTRACT_FRAMES`) or the deny list needs an allowance. Everything else in the manifest (`type` under ports, `required`, env `default`/`description`) validated as written.
- Task 13 (a): DORA_NAV's tracked `mickrobot.xml` has absolute `/home/mickrobot` mesh paths, so the sim cannot load it; `make_model.sh` generates the model from `mickrobot_3d.urdf` instead, and that model's geometry differs (lidar at (0, -0.02, 0.4) on `base_link` instead of (0.25, 0, 0.25); `base_link` at 0 instead of 0.1 above `base_footprint`, and the free body rests 0.0896 m above the ground). The extrinsics had been copied from the tracked file, which gave a 0.14 m ground error in the first live run. Calibration must come from the model actually used; fixed in DORA_NAV 9376bd1.
- Task 13 (b): a yaw-only odometry pose (z = 0, no roll/pitch) and a rigidly mounted `base_link` cannot represent chassis motion off the ground plane: pitch, or the body's drop and bounce after spawn. The checker therefore judges the run's median per-cloud ground error and only reports p95 and max (decided 2026-11-02). Good run with corrected extrinsics: 579 clouds, median 0.0029 m, p95 0.0041 m, max 0.0815 m. The max is from the spawn transient (the body starts at z = 0, bounces to 0.17 m and settles at 0.0896 m within ~0.5 s; its orientation stays level), not from pitch while driving.
- Task 13 (c): in dora 1.1.0 `dora run` runs the dataflow on an embedded daemon with no coordinator: it never contacts a coordinator port, its zenoh session listens on ephemeral loopback ports, and multicast scouting is off unless a node is dynamic (the dynamic-node listener is the only fixed port, and it is only opened for dynamic nodes). Isolating the live runs from a coordinator on the default port 6013 therefore needed no port configuration and no `dora up`/`destroy`; `ss` during the runs showed only ephemeral loopback listeners and the 6013 PIDs unchanged.
