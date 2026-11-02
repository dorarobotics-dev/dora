use super::Executable;
use dora_core::{
    descriptor::{
        Descriptor, DescriptorExt,
        frames_check::check_frames,
        validate::{check_dataflow_static, check_type_annotations_full, check_wiring},
    },
    manifest::{NodeManifest, inject::inject_adjacent_manifests},
    types::TypeRegistry,
};
use eyre::{Context, bail};
use std::path::{Path, PathBuf};

#[derive(Debug, clap::Args)]
/// Validate a dataflow YAML file (or a node manifest) and check type annotations
pub struct Validate {
    /// Path to the dataflow descriptor file
    #[clap(
        value_name = "PATH",
        value_hint = clap::ValueHint::FilePath,
        required_unless_present = "node_manifest",
        conflicts_with = "node_manifest"
    )]
    dataflow: Option<PathBuf>,
    /// Treat type warnings as errors (non-zero exit code)
    #[clap(long, action, conflicts_with = "node_manifest")]
    strict_types: bool,
    /// Validate a node manifest (dora-node.yml) instead of a dataflow
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    node_manifest: Option<PathBuf>,
    /// Do not access the network for hub index refreshes (cache only)
    #[clap(long, action, conflicts_with = "node_manifest")]
    offline: bool,
}

impl Executable for Validate {
    fn execute(self) -> eyre::Result<()> {
        if let Some(manifest_path) = &self.node_manifest {
            return validate_node_manifest(manifest_path);
        }
        let dataflow = self
            .dataflow
            .expect("clap guarantees dataflow when --node-manifest is absent");
        validate_dataflow(&dataflow, self.strict_types, self.offline)
    }
}

fn validate_node_manifest(path: &Path) -> eyre::Result<()> {
    println!("Validating node manifest {}...", path.display());
    let manifest = NodeManifest::read(path)?;

    // Built-in std/ types only; types shipped in the manifest's `types:`
    // block are resolved by validate() against the manifest itself.
    let registry = TypeRegistry::new();
    manifest.validate_strict(&registry)?;

    println!("Node manifest OK.");
    Ok(())
}

fn validate_dataflow(dataflow: &Path, strict_types: bool, offline: bool) -> eyre::Result<()> {
    let working_dir = dataflow
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));

    println!("Validating {}...", dataflow.display());

    // Parse and expand modules (no runtime needed)
    let mut descriptor = Descriptor::blocking_read(dataflow)
        .with_context(|| {
            format!(
                "failed to read dataflow at `{}`\n\n  \
                     hint: check the file exists, is valid YAML, and matches the dataflow schema (see details below)",
                dataflow.display()
            )
        })?
        .expand(working_dir)
        .context("failed to expand modules in dataflow descriptor")?;

    let strict = strict_types || descriptor.strict_types.unwrap_or(false);

    // Load user types if a types/ directory exists
    let mut registry = TypeRegistry::new();
    let types_dir = working_dir.join("types");
    if types_dir.is_dir() {
        match registry.load_from_dir(&types_dir) {
            Ok(count) if count > 0 => {
                println!("Loaded {count} user-defined type(s) from types/");
            }
            Err(e) => {
                bail!("failed to load user types: {e}");
            }
            _ => {}
        }
    }

    // If a lockfile exists, use its pins for hub: resolution so validate
    // reflects the same versions a --locked build would use and works offline
    // against the index cache. Without a lockfile, fall through to live
    // resolution (same as before).
    let lockfile_path = super::build::lockfile::BuildLockfile::path_for_dataflow(dataflow, None);
    let lockfile = if lockfile_path.exists() {
        match super::build::lockfile::BuildLockfile::read_from(&lockfile_path) {
            Ok(lf) => {
                println!(
                    "  Using hub pins from lockfile `{}`",
                    lockfile_path.display()
                );
                Some(lf)
            }
            Err(e) => {
                eprintln!(
                    "  warning: could not read lockfile `{}`: {e} — resolving hub: nodes live",
                    lockfile_path.display()
                );
                None
            }
        }
    } else {
        None
    };
    let hub_pins = lockfile.as_ref().map(|l| l.git_sources.clone());
    let hub_binary_pins = lockfile.as_ref().map(|l| l.binary_sources.clone());

    // Resolve hub: references so their contracts take part in validation
    // (spec §6.2 ordering note).
    let hub_resolution = super::build::hub::resolve_hub_nodes(
        &mut descriptor,
        &mut registry,
        offline,
        hub_pins.as_ref(),
        hub_binary_pins.as_ref(),
        // validate is advisory, not a reproducibility gate: use lockfile pins
        // when valid, but a missing/stale pin resolves live instead of failing.
        false,
        &std::collections::BTreeMap::<String, std::path::PathBuf>::new(),
    )?;
    for note in &hub_resolution.notes {
        println!("  {note}");
    }

    // Check input/output wiring (no build required)
    check_wiring(&descriptor).context("wiring check failed")?;
    println!("Input/output wiring OK.");

    // Static whole-descriptor checks (timing fields, log config, ros2 configs) —
    // the same ones `dora run`/`dora status --dataflow` enforce, minus
    // source-path existence which needs a build first.
    check_dataflow_static(&descriptor).context("descriptor check failed")?;
    println!("Descriptor config OK.");

    // Inject contracts from node manifests adjacent to path: nodes (§6.2)
    let injection = inject_adjacent_manifests(&mut descriptor, working_dir, &mut registry);
    for note in &injection.notes {
        println!("  {note}");
    }

    // Run type annotation checks
    let result = check_type_annotations_full(&descriptor, &registry, strict);

    // Print inferences
    for inf in &result.inferences {
        println!("  {inf}");
    }

    let count = hub_resolution.warnings.len() + injection.warnings.len() + result.warnings.len();
    if count == 0 {
        println!("All type annotations OK.");
    } else {
        println!("Type warnings:");
        for w in &hub_resolution.warnings {
            println!("  - {w}");
        }
        for w in &injection.warnings {
            println!("  - {w}");
        }
        for w in &result.warnings {
            println!("  - {w}");
        }
        println!("{count} type warning(s) found.");
    }

    // Frame, route and calibration checks; only with a `frames:` key. They
    // run even when --strict will fail on type warnings, so one run reports
    // both. Frame errors fail regardless of --strict.
    let mut frame_errors = 0;
    if descriptor.frames.is_some() {
        let frame_violations = check_frames(&descriptor, working_dir);
        frame_errors = frame_violations.len();
        if frame_violations.is_empty() {
            println!("Frames OK.");
        } else {
            println!("Frame errors:");
            for v in &frame_violations {
                println!("  - {v}");
            }
        }
    }

    let type_failures = if strict { count } else { 0 };
    match (type_failures, frame_errors) {
        (0, 0) => {}
        (tw, 0) => bail!("{tw} type warning(s) found (--strict mode)"),
        (0, fe) => bail!("{fe} frame error(s) found"),
        (tw, fe) => bail!("{tw} type warning(s) (--strict) and {fe} frame error(s) found"),
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::validate_dataflow;
    use std::path::Path;

    const FRAMES: &str = "../../libraries/core/tests/frames_check/frames.yml";
    const EXTRINSICS: &str = "../../libraries/core/tests/frames_check/extrinsics.yml";

    /// sim/cloud is a PointCloud in `rslidar`; the sink declares it a Pose
    /// (a type warning) in `base_link` (a frame error).
    const DATAFLOW: &str = "\
frames: {dictionary: frames.yml, extrinsics: extrinsics.yml}
nodes:
  - id: sim
    path: sim.py
    outputs: [cloud]
    output_types: {cloud: std/media/v1/PointCloud}
    output_frames: {cloud: rslidar}
  - id: sink
    path: sink.py
    inputs: {cloud: sim/cloud}
    input_types: {cloud: std/math/v1/Pose}
    input_frames: {cloud: base_link}
";

    fn validate(yaml: &str, strict: bool) -> Result<(), String> {
        let dir = tempfile::tempdir().unwrap();
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
        std::fs::copy(manifest.join(FRAMES), dir.path().join("frames.yml")).unwrap();
        std::fs::copy(manifest.join(EXTRINSICS), dir.path().join("extrinsics.yml")).unwrap();
        let path = dir.path().join("dataflow.yml");
        std::fs::write(&path, yaml).unwrap();
        validate_dataflow(&path, strict, true).map_err(|e| e.to_string())
    }

    #[test]
    fn strict_type_warnings_do_not_hide_frame_errors() {
        assert_eq!(
            validate(DATAFLOW, true).unwrap_err(),
            "1 type warning(s) (--strict) and 1 frame error(s) found"
        );
        assert_eq!(
            validate(DATAFLOW, false).unwrap_err(),
            "1 frame error(s) found"
        );
        let frames_ok = DATAFLOW.replace("cloud: base_link", "cloud: rslidar");
        assert_eq!(
            validate(&frames_ok, true).unwrap_err(),
            "1 type warning(s) found (--strict mode)"
        );
        assert_eq!(validate(&frames_ok, false), Ok(()));
    }
}
