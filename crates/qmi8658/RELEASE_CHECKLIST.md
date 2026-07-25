# Release Checklist (ph-qmi8658)

## Preflight

- Confirm `version` and `rust-version` in `crates/qmi8658/Cargo.toml`.
- Confirm `CHANGELOG.md` lists v0.1.2 as `Unreleased` until publication.
- Review `V0.1.2_REMEDIATION_PLAN.md`; every release-scoped issue must be
  `Done` and have evidence.
- Confirm QMI8658A/C behavior against the datasheets in `docs/vendor`.
- Ensure QMI8658B is not advertised as supported.
- Ensure examples use `ph_qmi8658`.

## Variant and Compatibility Gates

```bash
cargo test -p ph-qmi8658
cargo test -p ph-qmi8658 --no-default-features
cargo test -p ph-qmi8658 --no-default-features --features qmi8658a
cargo check -p ph-qmi8658 --no-default-features --features qmi8658a,qmi8658c
```

The first three commands must pass. The final command must fail with the
mutually-exclusive feature diagnostic.

Verify the `semver_compat` integration test compiles a v0.1.1-style `Config`
literal and exhaustive `Error` match.

## Quality Gates

```bash
cargo fmt --all -- --check
cargo clippy -p ph-qmi8658 --all-targets --features defmt,fixed -- -D warnings
cargo clippy -p ph-qmi8658 --all-targets --no-default-features --features qmi8658a,defmt,fixed -- -D warnings
cargo test -p ph-qmi8658 --features fixed
cargo test -p ph-qmi8658 --no-default-features --features qmi8658a,fixed
RUSTDOCFLAGS="-D warnings" cargo doc -p ph-qmi8658 --no-deps --features defmt,fixed
cargo deny --manifest-path crates/qmi8658/Cargo.toml check
```

Desktop tests intentionally omit `defmt`: linking `defmt` requires an embedded
logger/export implementation. Clippy, rustdoc, packaging, and embedded builds
still compile the `defmt` feature.

Run the same required checks with Rust 1.92.0.

## Hardware Gate

Complete the BE-01 matrix in `V0.1.2_REMEDIATION_PLAN.md` for QMI8658A and
QMI8658C over I2C/SPI with CTRL1.BE set and cleared. Attach:

- WHO_AM_I and revision ID
- CTRL1 readback
- direct raw bytes and decoded values
- FIFO raw bytes and decoded values
- static 1 g orientation
- temperature plausibility

The release remains blocked if any matrix row is missing or unexplained.

## Target Matrix

Build both `qmi8658c` and `qmi8658a` for:

- `xtensa-esp32-none-elf`
- `xtensa-esp32s2-none-elf`
- `xtensa-esp32s3-none-elf`
- `riscv32imc-unknown-none-elf`
- `riscv32imac-unknown-none-elf`
- `thumbv6m-none-eabi`
- `thumbv7m-none-eabi`
- `thumbv7em-none-eabi`
- `thumbv7em-none-eabihf`
- `thumbv8m.base-none-eabi`
- `thumbv8m.main-none-eabi`
- `thumbv8m.main-none-eabihf`

## Packaging

```bash
cargo package -p ph-qmi8658
cargo publish -p ph-qmi8658 --dry-run
```

Inspect the package contents and ensure the crate README and feature metadata
are present. The root remediation tracker and vendor PDFs are repository review
artifacts and are intentionally excluded from the package.

## Release

- Change `0.1.2 - Unreleased` to the publication date.
- Mark QA-01 and every release gate `Done`.
- Commit the final release metadata.
- Tag v0.1.2 and publish to crates.io.
- Open a new `Unreleased` changelog section.
