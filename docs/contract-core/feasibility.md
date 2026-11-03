# Contract core in dora: feasibility report

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
- Task 13 (b): the checker judges a run on its median per-cloud ground error only; p95 and max are reported, not judged (decision of 2026-11-02). Good run with corrected extrinsics: 579 clouds, median 0.0029 m, p95 0.0041 m, max 0.0815 m. The measured max is the spawn settling transient: the body starts at z = 0, bounces to 0.17 m and settles at 0.0896 m within ~0.5 s with its orientation level, a height change a planar odometry pose (z = 0) cannot represent. Pitch is a real limitation of a yaw-only odom pose with a rigidly mounted `base_link` (clouds taken while the chassis pitches would carry error the chain cannot remove), but no pitch was observed in this run. (Corrected in Task 15: an earlier version of this entry, and the checker's docstring, attributed the max to pitch.) Superseded by the final review: the checker now also judges every cloud from 1 s after the first one, which excludes only the spawn transient, and recomputes every cloud from the raw cloud and the pose; see Result, section 3.
- Task 13 (c): in dora 1.1.0 `dora run` runs the dataflow on an embedded daemon with no coordinator: it never contacts a coordinator port, its zenoh session listens on ephemeral loopback ports, and multicast scouting is off unless a node is dynamic (the dynamic-node listener is the only fixed port, and it is only opened for dynamic nodes). Isolating the live runs from a coordinator on the default port 6013 therefore needed no port configuration and no `dora up`/`destroy`; `ss` during the runs showed only ephemeral loopback listeners and the 6013 PIDs unchanged.

## Result

Verdict: the operator contract fits dora. Header metadata, validity rules,
frames, the transform library and the pre-launch frame checks now live in
dora (`dora-message`, a new `dora-transform` crate, `dora-core`, `dora
validate`, the C and Python APIs and a `frame-transform` node). They run end to
end from DORA_NAV's MuJoCo sim through the transform node. Nothing in dora's
architecture had to change; the frictions below are small API and manifest gaps,
each with a concrete upstream change.

| Exit criterion (spec §9) | Evidence | Met |
|---|---|---|
| 1. Tests pass, no regression | cargo and pytest totals below, at f70c3de9c | yes |
| 2. `dora validate` accepts good, refuses each bad with its reason | DORA_NAV `examples/contract-core/run_validate.sh` output below | yes |
| 3. Live run meets §7 | DORA_NAV `examples/contract-core/records/contract-core.json`, `contract-core-stripped.json`, `c-header.log` | yes (isolated with `dora run`, not a coordinator on 6113; see Friction 13 (c)) |
| 4. Mutation sweep 0 survivors | `tools/contract-mutation-sweep.py`, output below | yes |
| 5. This report | this file; `docs/contract-core/test-mapping.md` | yes |

### 1. Tests

`cargo test -p dora-message -p dora-core -p dora-transform -p dora-cli -p dora-node-api-c`
at f70c3de9c (after the final review's fixes): 1336 passed, 0 failed, 1 ignored
(the ignored one is an existing `dora-cli` doc test), no `FAILED`, no panic. Per
crate:

| Crate | Test binaries | Passed |
|---|---|---:|
| dora-message | lib 228, node_exit_status 5, uhlc_wire_format 7, doc 24, compile_fail doc 1 | 265 |
| dora-core | lib 456, generate_schema 3, descriptor-validation 2, frames_check 48, doc 10 | 519 |
| dora-transform | frames_extrinsics 32, rigid 30 | 62 |
| dora-cli | lib 476, cli_surface 2, logs_rejection_fallback 2, doc 2 (+1 ignored) | 482 |
| dora-node-api-c | lib 8 | 8 |

Against the baseline: `cargo test -p dora-message -p dora-core` alone, the
command the baseline used, gives 712 passed, 0 failed (baseline 619). The
difference is exactly the added tests: `dora-message` lib 190 → 228 (+38),
`dora-core` lib 378 → 384 (+6), the new `frames_check` binary (48) and the
`compile_fail` doc test that pins `Header`'s private fields (1); every other
binary has its baseline count. (In the five-crate run `dora-core`'s lib shows
456 because `dora-cli` unifies extra `dora-core` features; that count has no
baseline from Task 1.) `dora-cli` and `dora-node-api-c` had no recorded
baseline; this slice added 1 and 2 tests to them, and all their tests pass.

Python, each run as `PYTHONPATH= .venv-exp/bin/python -m pytest -q tests`
(the last one as `pytest -q test_checker.py`):

- `apis/python/node`: 152 passed (includes 95 `dora.header` tests in
  `test_header.py` and 5 `dora.transform` tests)
- `nodes/frame-transform`: 56 passed
- DORA_NAV `modules/simulation/mujoco_mickrobot`: 27 passed
- DORA_NAV `examples/contract-core/test_checker.py`: 19 passed

No existing test failed, so nothing needed an unrelated-failure note.

Test mapping (`tools/check-test-mapping.py`): 215 collected reference tests,
176 mapped to the new tests that carry their promise and 39 marked n/a with a
reason, 0 unmapped, 0 empty, 0 stray. When the mapping was first made (at
d625b4ebf: 174 mapped, 41 n/a) it exposed reference promises no new test held;
59 tests were added to close them (57 in dora, 2 in DORA_NAV), and one was an
implementation gap: `derive()` accepted a counted upgrade with an empty reason,
which the reference refuses; it now refuses it too (d625b4ebf). Two rows moved
from n/a to tests after the final review bound the Rust header into Python
(header immutability, and a non-mapping metadata value). The reference tests
collect with the experiment venv as is; no reference package had to be
installed.

### 2. `dora validate`

```
ok   good.yml
ok   bad-frame-mismatch.yml
ok   bad-unknown-frame.yml
ok   bad-missing-calibration.yml
ok   bad-two-dynamic-edges.yml
```

`good.yml` prints `Frames OK.` and exits 0; each bad file exits non-zero with its
reason (`requires frame 'base_link'`, `use 'rslidar'`, `never assumed to be
identity`, `crosses 2 dynamic edges`). Release `dora` rebuilt from the tree
committed as f70c3de9c and the five files re-checked: 5/5 ok.

### 3. Live run

Re-run on 2026-11-02, 17:55-17:58, with dora at adfb98c52 (release CLI built
at 6c1afd206; adfb98c52 changed only the Python binding and the sweep) and
DORA_NAV at 51ff415. `run_live.sh` exited 0. The records are DORA_NAV
`examples/contract-core/records/contract-core.json`,
`contract-core-stripped.json` and the three `.log` files (committed in
DORA_NAV 431cf4b).

The checker (DORA_NAV `examples/contract-core/checker.py`, its own tests in
`test_checker.py`) passes the good run only if all of these hold:

- Frame and header: every cloud is in `odom` and carries a strictly increasing
  `header_seq`.
- Validity: at least one `ok` cloud; at most 3 `stale` clouds, all before the
  first `ok` one.
- Independent recomputation of every cloud: the raw cloud with the same
  `stamp_ns` is moved to `odom` in plain numpy (its own quaternion code, not
  `dora.transform`) through the static chain read from
  `robots/mickrobot-sim/extrinsics.yml` and a sim pose the node could have
  held (stamped at most 100 ms before the cloud for an `ok` cloud). Every point
  must match within 1e-3 m, intensity unchanged.
- Motion: the pose moves at least 1.0 m in x/y and turns at least 0.5 rad, so
  the recomputation depends on the pose.
- Ground: the median per-cloud ground error is at most 0.03 m, and so is every
  cloud stamped 1 s or more after the first cloud (only the spawn transient is
  excluded).
- Drops: published ≥ raw − 3, counting only clouds stamped at least 200 ms
  before the last raw cloud (newer ones may still be in flight); the diag's
  drops hold only `no_pose`, at most 3.

Results:

- Good run (60 s): CHECKER_OK. 580 raw clouds in, 579 published, all 579
  `ok`. Drop count with in-flight clouds excluded: raw 578, published 577.
  Diag drops `{no_pose: 1}`.
  - Recomputation: 579 of 579 clouds checked, max point error 1.3e-6 m (limit
    1e-3 m); the matched pose was 20 ms older than the cloud on every cloud.
  - Motion: 2899 poses, 6.98 m travelled in x/y, 20.7 rad of yaw.
  - Ground error: median 0.0029 m, p95 0.0042 m, max 0.0814 m (the spawn
    transient, Friction 13 (b)), max after the 1 s settle 0.0064 m (limit
    0.03 m).
  - Last diag message: `in` 580, `out` 579; over its period `items` 115066
    points, `bytes` 1841216, `proc_ns` 7861057.
- Stripped run (`frame_id` removed from the sim's cloud, 20 s): CHECKER_OK;
  180 raw clouds in, 0 published, diag drops `{bad_frame: 180}` of `in` 180.
- C header example (`examples/c-header`): C_HEADER_OK; a C node's header
  arrives intact in a Python receiver.
- Isolation: `dora run` uses an embedded daemon and no coordinator; the other
  project's coordinator on 6013 (pid 3671210, daemon 3671247) was untouched.

What the earlier checker could not prove. The final review found that the
checker used for the first version of this report (DORA_NAV a6a8008) judged
each transformed cloud's frame, `header_seq` order and validity, and the
median ground error; nothing it judged depended on the pose. The ground check
cannot: the sim's odometry pose is
planar (z = 0, yaw only), so a node that ignored the pose, inverted it or took
it from the wrong time would still put the ground at z ≈ 0 in `odom`. It also
had no drop cap, so a node that dropped most clouds would have passed. The
headline numbers did not change (579 `ok`, median 0.0029 m); what changed is
what they prove: the pose is now verified on every cloud against an independent
recomputation over 6.98 m and 20.7 rad of motion, drops are capped, and the
ground error after the spawn transient is judged (0.0064 m), not only
reported.

### 4. Mutation sweep

15 mutations: the ten reference ones in this slice (3, 4, 7, 10-16) and five
added by the final review (H1, H2, D1, F1, P1; listed in `test-mapping.md`).
Every suite runs for every mutation and its result is printed; H1 and H2 must
be caught by the Python binding suite. Run at adfb98c52, exit 0 (it was 0/10 at
37be3cc9b and at d625b4ebf, before the review added mutations):

```
caught   3. output publishes its input's frame
         rust-header=pass  rust-transform=pass  rust-frames-check=pass  py-binding=pass  py-node=FAIL
caught   4. an upgrade is granted but never counted
         rust-header=FAIL  rust-transform=pass  rust-frames-check=pass  py-binding=FAIL  py-node=pass
caught   7. a degraded validity is laundered into ok
         rust-header=pass  rust-transform=pass  rust-frames-check=pass  py-binding=pass  py-node=FAIL
caught   H1. a degraded validity needs no reason
         rust-header=FAIL  rust-transform=pass  rust-frames-check=pass  py-binding=FAIL  py-node=pass  (requires py-binding)
caught   H2. a bool is accepted as an int
         rust-header=FAIL  rust-transform=pass  rust-frames-check=pass  py-binding=FAIL  py-node=pass  (requires py-binding)
caught   10. a missing extrinsic is quietly treated as identity
         rust-header=pass  rust-transform=FAIL  rust-frames-check=FAIL  py-binding=pass  py-node=pass
caught   11. transforms compose even when their frames do not meet
         rust-header=pass  rust-transform=FAIL  rust-frames-check=pass  py-binding=pass  py-node=pass
caught   12. rotation_xyzw is read as w, x, y, z
         rust-header=pass  rust-transform=FAIL  rust-frames-check=pass  py-binding=FAIL  py-node=FAIL
caught   13. a calibration file may carry a dynamic edge
         rust-header=pass  rust-transform=FAIL  rust-frames-check=pass  py-binding=pass  py-node=pass
caught   D1. the node builds its output header by hand, always ok, bypassing derive
         rust-header=pass  rust-transform=pass  rust-frames-check=pass  py-binding=pass  py-node=FAIL
caught   14. a cloud composed with a stale pose is forwarded as ok
         rust-header=pass  rust-transform=pass  rust-frames-check=pass  py-binding=pass  py-node=FAIL
caught   15. the validator skips the transform check
         rust-header=pass  rust-transform=pass  rust-frames-check=FAIL  py-binding=pass  py-node=pass
caught   16. a node may launch with a different calibration than was checked
         rust-header=pass  rust-transform=pass  rust-frames-check=FAIL  py-binding=pass  py-node=pass
caught   F1. a node may launch with a different frame dictionary than was checked
         rust-header=pass  rust-transform=pass  rust-frames-check=FAIL  py-binding=pass  py-node=pass
caught   P1. a pose in another frame than the dynamic edge's parent covers it
         rust-header=pass  rust-transform=pass  rust-frames-check=FAIL  py-binding=pass  py-node=pass
0/15 not caught as required
```

f70c3de9c changed the line P1 mutates (the pose check now skips a pose frame
the dictionary does not know, which check 1 already reports). P1 was
re-anchored and run by hand against `rust-frames-check`: caught, 2 tests fail.
At f70c3de9c every one of the 15 anchors occurs exactly once in its file. The
full sweep was not re-run after f70c3de9c.

The seven reference mutations outside this slice (1, 2, 5a, 5b, 6, 8, 9) are
listed with their reasons at the end of `test-mapping.md`.

## Size of each change

From `git diff --numstat 1016eab0a..d625b4ebf` (dora) and
`git diff --numstat a9257f5..a6a8008` (DORA_NAV), grouped by component. Test
lines include inline `#[cfg(test)]` modules, fixtures and run records. This
report's own commit (feasibility.md, test-mapping.md, check-test-mapping.py)
adds about 530 lines of docs and tooling on top.

The tables are at d625b4ebf and a6a8008, before the final review. Its fixes
add, in dora, 107d2d488..f70c3de9c (5 commits, docs excluded): 24 files,
+1738 / −208 lines, mostly the Python header binding, the node's use of it,
its diag and the validator's pose and dictionary checks; and in DORA_NAV,
a6a8008..431cf4b (6 commits, including the new run records): 27 files,
+1317 / −334 lines, mostly the checker's recomputation and its tests. From
the base, dora is now 55 files, +6966 / −5 (docs excluded) and DORA_NAV 30
files, +2234 / −36.

dora:

| Component | Files | Lines added (code/config) | Lines added (tests, fixtures, records) | Lines removed |
|---|---:|---:|---:|---:|
| Header (`dora-message` header.rs) | 3 | 297 | 451 | 0 |
| Transform library (`dora-transform`, workspace Cargo) | 11 | 837 | 1116 | 0 |
| Frame annotations and `frames:` key (descriptor, JSON schema) | 7 | 143 | 138 | 0 |
| Frame check (`dora validate`) | 6 | 438 | 772 | 2 |
| Spatial std types | 3 | 28 | 44 | 1 |
| Manifest wiring (test only) | 1 | 0 | 28 | 0 |
| C API and C example | 6 | 230 | 72 | 2 |
| Python `dora.transform` | 5 | 188 | 46 | 0 |
| frame-transform node | 7 | 243 | 206 | 0 |
| Tools and docs (sweep, earlier feasibility entries) | 2 | 122 | 0 | 0 |
| **Total** | 51 | 2526 | 2873 | 5 |

DORA_NAV:

| Component | Files | Lines added (code/config) | Lines added (tests, fixtures, records) | Lines removed |
|---|---:|---:|---:|---:|
| Sim headers and typed outputs | 9 | 145 | 195 | 16 |
| Robot description (frames, extrinsics) | 3 | 170 | 0 | 0 |
| Example dataflows, checker, scripts | 10 | 510 | 0 | 0 |
| Live-run records | 6 | 0 | 211 | 0 |
| **Total** | 28 | 825 | 406 | 16 |

Everything in dora is additive (5 lines removed in total, still 5 at f70c3de9c). The only edits to
existing dora behaviour are `dora validate` reporting frame errors alongside
`--strict` type warnings, and `metadata.rs`'s key-uniqueness test listing the
header's sequence key.

## What did not fit dora's model

Each item: what, why, and the upstream change it implies.

1. **Header sequence key.** dora's metadata already uses `seq` for streaming
   chunk numbers (`metadata::SEQ`), so the header's sequence key is
   `header_seq` (deviation 7), constant `dora_message::header::HEADER_SEQ`.
   Upstream: list the contract keys (`frame_id`,
   `header_seq`, `stamp_ns`, `source`, `validity`, `reason`, `child_frame`)
   beside `request_id`/`goal_id` as well-known metadata keys, so no feature
   reuses them.
2. **No `schema` key at run time.** dora's port type URNs replace the
   reference's schema header and its version carets (deviation 3; reference
   mutations 5a/5b do not apply). The major version is the URN path
   (`std/media/v1/...`). Upstream: none; this fits.
3. **Required metadata keys are not enforced.** `TypeDef.metadata` in the std
   type files is parsed but read nowhere in dora (Friction, Task 5), so adding
   `frame_id`/`stamp_ns`/`child_frame` to `Image`, `PointCloud`, `Pose` and
   `Transform` documents them only. The frame check covers declared frames
   before launch; nothing checks a running node. Upstream: enforce a type's
   required metadata keys, at least in `dora validate` for declared producers
   and optionally on send in debug builds.
4. **Header keys are not protected.** dora metadata is an open map the node
   owns; user code can overwrite a header key after `Header::write`, where the
   reference refused shadowing extras. Upstream: a typed header API on the
   node APIs (`send_output_with_header` in Rust, Python and C) that writes the
   keys last and refuses shadowing.
5. **Python header binding (built after the final review).** The first
   version of this report had none: the frame-transform node re-implemented
   `Header` in Python. Now `dora.header` binds the Rust `dora_message::header`
   (`Header` with read-only properties, `derive`, `Upgrade`, the key
   constants; `VALIDITY` is the tuple of values, `VALIDITY_KEY` the key), and
   the node builds its output header with `derive` and its diag header with
   `Header`, so there is one header implementation. Upstream: ship
   `dora.header` in the `dora` Python package.
6. **Calibration env names.** The node manifest validator refuses env entries
   starting with `DORA_` (`ENV_DENY_PREFIX`, "security-sensitive"), so
   `dora-node.yml` cannot declare `DORA_FRAMES`/`DORA_EXTRINSICS` (deviation 5).
   Since the final review, `dora validate` requires both on every node that
   changes frames, and pins each to the file the dataflow's `frames:` key
   names (`DORA_FRAMES` to `frames.dictionary`, mutation F1;
   `DORA_EXTRINSICS` to `frames.extrinsics`, mutation 16).
   Upstream: use a non-`DORA_` prefix for the contract's env (for example
   `FRAMES_DICTIONARY`/`FRAMES_EXTRINSICS`), or better, have the daemon inject
   both from the dataflow's `frames:` key so no node or dataflow names them.
   The second option also removes the class of fault mutations 16 and F1 test
   (a node launched with a calibration or dictionary other than the checked
   one).
7. **Launch directory is daemon logic.** To compare a node's
   `DORA_FRAMES` and `DORA_EXTRINSICS` with the checked files, the frame check mirrors the
   daemon's working-directory rule (`deploy.working_dir` over the dataflow
   directory), and a git-sourced node must give an absolute path because its
   checkout directory is unknown before launch. Upstream: one shared
   `launch_dir` helper in `dora-core` used by daemon and validator (or the
   injection in item 6, which makes it unnecessary).
8. **Frames are declared on dataflow nodes, not manifests.** Port frames are
   node keys in the dataflow (`input_frames`, `output_frames`) and a top-level
   `frames:` key; `dora-node.yml` already declares typed ports but has no field
   for frames (port frames, the dictionary or the calibration), and none for
   rate, max-age or diagnostics. Upstream (F.03): `frame:`, `rate_hz`,
   `max_age_ms` on manifest ports, injected into the descriptor like `type:`.
   Two reference tests are n/a for want of these fields (diagnostics port,
   pose max-age declaration).
9. **Env values are passed verbatim.** dora hands a node its env untrimmed,
   so the frame check uses the raw value and refuses `' odom'` instead of
   trimming it as the reference did (inverted by decision); a null env value
   is refused when the dataflow is parsed. Upstream: none.
10. **Pose inputs by type.** A pose input is recognised by its
    `std/math/v1/Pose` URN, from the node's `input_types` or the upstream's
    `output_types`, instead of a contract schema name. Upstream: none; it fits,
    and argues for typed ports everywhere.
11. **`--strict` hid frame errors.** `dora validate --strict` bailed on type
    warnings before any later check ran; frame errors are now reported in the
    same run and fail regardless of `--strict`. Upstream: collect all
    validation findings and fail once.
12. **C API had no metadata.** `dora_send_output` took no parameters (F.02).
    Added `dora_send_output_with_params` with a `DoraParam` array. The kind is
    read as a `u32` and checked, so an unknown kind is an error. A bool is read
    as its first byte and not checked: any non-zero byte reads as true, so a C
    `_Bool` holding a value other than 0 or 1 is not undefined behaviour. Upstream: this function, plus
    reading metadata in the C input API.
13. **Wiring faults are dora's.** Unknown nodes and outputs, timer syntax and
    duplicate ids are checked by dora's own wiring check and parser, which
    stop at the first fault; the reference reported every edge. Those
    reference tests map to dora's tests or n/a (reference mutations 8 and 9).
    Upstream: none needed for the contract.
14. **Manifest categories are a closed list.** `categories:` in
    `dora-node.yml` accepts only dora's fixed set (sensor, transform, filter,
    simulator, ...). The contract nodes fit `transform` and `simulator`, but
    there is no `perception` category for the row-13 nodes. Upstream: add
    `perception`, or allow free tags beside the fixed set.
15. **Quaternion handling.** `from_parts` normalises and `then()`/`inverse()`
    do not re-validate (deviation 9). Re-validating composed matrices rejected
    long chains of accepted inputs at some orientations, a false rejection the
    Python reference also has. Upstream: none in dora; the fix belongs in the
    reference too.
16. **Sim model and calibration.** DORA_NAV's tracked `mickrobot.xml` has
    absolute `/home/mickrobot` mesh paths, so the sim loads a model generated
    from the URDF, whose geometry differs; the extrinsics are measured from
    the generated model (DORA_NAV 9376bd1). Upstream (DORA_NAV, feature map
    22.02): relative mesh paths in the tracked model.
17. **Planar odometry.** A yaw-only odom pose with a rigidly mounted
    `base_link` cannot represent the spawn settling transient (the measured
    max) or chassis pitch (a real limitation, not observed here). The run is
    judged on the median and on every cloud from 1 s after the first one,
    which excludes the spawn transient only. Upstream (DORA_NAV): publish a full 6-DoF pose, or a
    mounted-frame pose, when the chassis is not planar.
18. **Isolation came free.** `dora run` uses an embedded daemon with no
    coordinator (Friction 13 (c)). Upstream: document it; no change.
19. **A plain `PointCloud` matches any layout.** dora's type check treats an
    unparameterised `std/media/v1/PointCloud` as a wildcard against any
    `PointCloud[fields=...]`, although the type's declared default is
    `fields=xyz` (`libraries/core/src/descriptor/validate.rs` test
    `type_check_point_cloud_same_layout_or_unparameterised_side_matches`). An
    `xyz` cloud on a port that expects `xyzi` therefore passes `dora validate`.
    The frame-transform node catches it at run time: `width*height` must equal
    the number of points the buffer holds at the configured layout, else the
    cloud is a counted `malformed` drop. The DORA_NAV dataflows declare
    `PointCloud[fields=xyzi]` on every cloud port. Upstream: treat an
    unparameterised side as its declared default, or warn.
20. **Check 4 is partial, and the validator is stricter than the runtime.**
    Deviation 4 as built: a route that crosses one dynamic edge needs a wired
    `std/math/v1/Pose` input, and when that pose's upstream declares a frame it
    must be the edge's parent frame (mutation P1). The pose's `child_frame` is
    not checked, because no descriptor field declares a port's
    `child_frame`; an undeclared pose frame is accepted. At run time the node
    accepts a pose in either direction (an inverted pose, the child given as
    `frame_id`, is composed the other way), which the validator refuses when
    the frame is declared. This is a deviation from the reference, recorded
    here. Upstream: a `child_frame` field on ports (with item 8), after which
    the two sides can be checked and the runtime made as strict as the
    validator.
21. **The Python `Node` does not expose its id.** Rust `Node::id()` exists,
    but the Python binding's `id` is not a `#[pymethods]` method, and dora
    sets no `DORA_NODE_ID`. The frame-transform node parses `node_id:` from
    `DORA_NODE_CONFIG` for its diag `source` (refusing to start if neither it
    nor `NODE_ID` names one); the sim uses a fixed producer name,
    `mujoco_mickrobot`. Upstream: expose `Node.id` in Python.

## Effort for later slices

Basis. This slice ported a working, tested reference: 18 dora commits before
this report and 7 DORA_NAV commits, about 2.5k lines of code and 2.9k lines of tests in dora and 0.8k plus
0.4k in DORA_NAV. The final review after this report found gaps (a checker
that could not prove the pose was used, a Python header duplicating the Rust
one, unchecked point layout, an unpinned frame dictionary) whose fixes took
5 dora and 6 DORA_NAV commits, about 1 engineer-day. Five of the nine dora feature commits needed a review-fix
commit (header key clash, private fields, orientation-dependent false rejection,
`--strict` hiding frame errors, undefined behaviour in the C API), and mapping
the reference tests afterwards found 59 missing tests and one behaviour gap. For
an engineer doing the same work by hand, with review, we estimate this slice at
12-15 engineer-days, about 400-450 lines of code plus tests per day, plus the
final review's fixes, about 1 engineer-day: 13-16 in all. That rate is
used below for items with a reference implementation ("built, to move" or
"basis exists" with code to wrap). Items with no reference ("new") are
estimated from their scope at roughly half that rate, plus time for upstream
design discussion where the feature map says it is needed. Estimates are for
one engineer and exclude the delivery-sheet records unless named.

### What this slice delivered against the feature map

| Item | Delivered | Not delivered |
|---|---|---|
| F.01 Design proposal | partly: the spec, deviations and this report are the material | the proposal to dora maintainers is not written or sent |
| F.02 C API metadata | fully, in the local copy: `dora_send_output_with_params`, C example | not upstreamed; C input-side metadata read not added |
| F.03 Port declarations | partly: frames declared per port (`input_frames`/`output_frames`, dataflow `frames:`) | frame, rate and max-age in `dora-node.yml` itself |
| F.04 `dora validate` checks | partly: frame match, unknown and rejected frames, transform route, dynamic edges and pose wiring, the pose's parent frame when declared (check 4 in part, item 20), launch calibration and frame dictionary pinned, the first failing route per node reported; schema match is dora's existing type check | declared validity handling; the pose's `child_frame` |
| F.06 Conformance kit | partly: degraded-forwarding tests and a 15-mutation sweep (every suite run per mutation) for the header, its Python binding, transform library, validator and frame-transform node | a reusable kit for node authors |
| F.07 Robot descriptions | partly: `robots/mickrobot-sim` (frames, extrinsics) in DORA_NAV | the reference robots (agibot-d1-max, agibot-x2, aegis-ultra) |
| 13.01 Frames and transform library | fully for the Rust core and Python binding | the C++ binding |
| 13.02 frame_transform node | fully: `nodes/frame-transform`, static or pose-composed, run live | none |
| 13.17 Sim header metadata | fully for the MuJoCo mobile base: every output (pointcloud, cloud, image, imu, Odometry, odom_pose, gnss) carries the header; `cloud` and `odom_pose` are typed | typed image output |

Feature-map corrections found on the way:

- **F.05** says "No action concept in dora". dora already has service
  (request/reply) and action (goal/feedback/result, cancel) patterns:
  `docs/patterns.md`, the well-known metadata keys `request_id`, `goal_id`,
  `goal_status` in `libraries/message/src/metadata.rs`,
  `recv_service_response`/`recv_action_result` in the Rust node API, and
  `examples/service-example`, `examples/action-example`. F.05 should read
  "basis exists": what remains is task IDs and progress over the client API
  and gateway, and the same helpers in Python.
- **F.03** says to extend `dora-node.yml` ports "with schema, frame and rate".
  Manifests already declare typed ports (`type:` with a std type URN, and
  `required:`), and `dora build`/`validate` inject them; schema is done. What
  remains is frame and rate (and max-age, item 8 above).
- **13.17** listed as "new" is now delivered for the sim (above); **22.02**'s
  absolute mesh paths are confirmed and worked around, not fixed.

### Perception (row 13)

| Item | Status after this slice | Engineer-days |
|---|---|---:|
| 13.01 C++ binding of the transform library | Rust core and Python done | 2 |
| 13.03 Perception schemas (fused frame, obstacle list, 3-D detections) | new | 3 |
| 13.04 Camera intrinsics and image↔3-D projection | new | 4 |
| 13.05 / 19.07 Time-align node (pairing window; missing, stale, unpaired) | new; validity values exist | 5 |
| 13.06 Lidar + RGB fusion node (DORA_NAV) | new; needs 13.04, 13.05 | 5 |
| 13.08 Depth rendering in the sim camera | new | 2 |
| 13.09 Ground segmentation (restore from fac6f21, C API headers) | new | 4 |
| 13.10 Clustering to obstacle list | new | 3 |
| 13.12 2-D → 3-D lifting | new; needs 13.04 | 3 |
| 13.13 Association and velocity | new | 3 |
| 13.14 Runnable examples | basis: the contract-core chain runs | 2 |
| 13.16 Interface documentation | basis | 1 |
| 13.07, 13.11 RealSense/Orbbec and YOLO headers (dora-hub patches) | basis; pattern from the sim's `contract.py` | 3 |
| **Total** | 13.02 and 13.17 done | **40** |

The transform library, frame checks and headers this slice built are the
foundation every row-13 node uses, so the remaining work is node algorithms,
not contract plumbing. 13.15's accuracy records are delivery work on top.

### Agent path (octos, octos-dora bridge, client API, actions)

| Item | Status after this slice | Engineer-days |
|---|---|---:|
| F.05 Actions over the client API: task IDs, progress, cancel; Python helpers | basis exists (correction above) | 3 |
| 24.01 Python client API: discovery, command call, state subscription, async tasks | new; builds on `ws_protocol.rs` and the action pattern; upstream design discussion | 10 |
| 24.03 Unified API messages, error codes, task IDs | basis: header and validity | 2 |
| 21.01 Gateway, minimal (identity, role check) that the bridge calls through | new; upstream design discussion | 8 |
| F.08 Bridge forwards tool calls through the client API and awaits the result | basis: crate exists, no dora dependency | 3 |
| F.09 Bridge reports undelivered calls as failures | basis | 1 |
| F.10 Tool specs from `dora-node.yml` | new; typed manifest ports exist (F.03 correction) | 2 |
| 24.02 Second-language client | new | 6 |
| 24.05 Cross-language consistency suite | new | 3 |
| 24.06 Client examples, navigation and arm (DORA_NAV, dora-moveit2) | new | 3 |
| **Total** | | **41** |

Nothing in this slice is on this path except the header and validity it can
carry, so the estimate rests on scope, not on a ported reference, and is the
least certain of the four. Security hardening beyond a minimal gateway
(21.02-21.07) is another 12-15 days and is not included.

### Data processing (row 16)

| Item | Status after this slice | Engineer-days |
|---|---|---:|
| Contract runtime helpers in Python: per-input `max_age_ms` age rule, seq source, general counters and diag, runtime `same_as:` | built, to move (reference `wire.py`, `node.py`); the reference tests marked "later slice" here. The header binding (item 5) is done, and frame-transform's diag is a pattern for the counters | 2 |
| 16.01 Filtering (point cloud, image, state) | built, to move | 3 |
| 16.02 Compression | built, to move | 3 |
| 16.03 Cleaning (missing, outlier, out-of-order, duplicate; counted) | built, to move | 3 |
| 16.04 ROS 2 ↔ dora conversion into `ros2-bridge` (Rust, a different home) | built, to move | 5 |
| 16.05 Per-stage ratio and latency stats, A/B compare | built, to move | 3 |
| Test mapping and the three data-processing mutations (1, 2, 6) | | 2 |
| **Total** | | **21** |

This is the closest to this slice: a tested Python reference (349 tests) ported
onto the header and validity rules already in dora. The manifest fields in
item 8 (max-age, diagnostics) are worth settling first, since the operators
declare both.

### Navigation and arm adoption (rows 14 and 19)

| Item | Status after this slice | Engineer-days |
|---|---|---:|
| 19.03 DORA_NAV build fix (CMake needs the dora C library) | basis | 2 |
| 14.13 Navigation nodes declare ports and emit headers (about 12 C/C++ nodes) | new; unblocked by F.02 | 6 |
| 19.01 Lidar/IMU drivers: frame string on the cloud, IMU sign | basis | 2 |
| 14.15 Arm nodes declare ports and emit headers (Python) | new | 3 |
| 14.12 / 19.09 Control and state schemas (units, frames, sampling time) | basis | 3 |
| 14.11 Refuse when `num_waypoints` is missing | basis | 1 |
| 14.03 Localization status and quality output | new | 2 |
| 14.01, 14.02 Map save/load, relocalization | basis | 4 |
| 14.04, 14.05, 14.08 Planner and controller wrapping with config | basis | 3 |
| 14.06 Behaviour decision node | new | 5 |
| 14.07 Trajectory smoothing and time parameterization | new | 4 |
| 14.09 IK wrapping; 14.10 limit layer | basis; new | 6 |
| 19.02, 19.05, 19.08 Localization, arm integration, chassis configs | basis | 6 |
| 19.04 Multi-waypoint missions; 19.06 arm task status (on the action pattern) | new; basis exists for actions | 6 |
| 14.14 Docs and examples | new | 2 |
| **Total** | | **55** |

Header adoption itself (19.03, 14.13, 19.01, 14.15, 14.12/19.09) is about 16 of
these days; the sim's `contract.py` and the C API from this slice are the
pattern for it. The rest is navigation and arm features the feature map lists
under the same rows. Integration records (19.10, 14.14's records) are delivery
work on top.

### Summary

| Slice | Engineer-days | Confidence |
|---|---:|---|
| This slice (contract core), for reference | 13-16 | measured from the work done, including about 1 day of fixes after the final review |
| Perception (row 13) | 40 | medium: foundation done, algorithms new |
| Agent path | 41 (+12-15 security) | low: no reference implementation, upstream design needed |
| Data processing (row 16) | 21 | high: tested reference, same pattern as this slice |
| Navigation and arm adoption (rows 14, 19) | 55 (16 for headers alone) | medium |
