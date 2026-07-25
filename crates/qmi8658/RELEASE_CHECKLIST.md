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

Use the automated collector and operator procedure in
[`apps/qa-runner/README.md`](../../apps/qa-runner/README.md). Before accepting
the gate, run:

```bash
python -m unittest discover -s apps/qa-runner/tools -p "test_*.py" -v
python apps/qa-runner/tools/collect_evidence.py matrix \
  --input-dir hardware-evidence/v0.1.2 \
  --expected-source-commit <candidate-commit-sha>
```

Commit the raw logs, per-run JSON/Markdown files, and generated
`BE-01-MATRIX.json`/`BE-01-MATRIX.md`. The matrix command must exit successfully,
the report must say `PASS`, and every selected run must come from a clean source
tree at the candidate commit.

The release remains blocked if any matrix row is missing or unexplained.

## Target Matrix

Build both `qmi8658c` and `qmi8658a` for every row. Because the driver is
HAL/PAC-independent, these builds verify CPU architecture and ABI compatibility;
they do not claim board-level peripheral integration.

| Rust target | Representative MCU coverage |
|---|---|
| `xtensa-esp32-none-elf` | ESP32 |
| `xtensa-esp32s2-none-elf` | ESP32-S2 |
| `xtensa-esp32s3-none-elf` | ESP32-S3 |
| `riscv32imc-unknown-none-elf` | ESP32-C2, ESP32-C3 |
| `riscv32imac-unknown-none-elf` | ESP32-C5/C6/C61/H2 and RP2350/RP2354 Hazard3 |
| `riscv32imafc-unknown-none-elf` | ESP32-P4 |
| `thumbv6m-none-eabi` | RP2040, STM32F0/G0/L0, nRF51, SAMD21 |
| `thumbv7m-none-eabi` | STM32F1/F2/L1, SAM3/SAM4 |
| `thumbv7em-none-eabi` | Cortex-M4 soft-float, including STM32F3/L4 configurations |
| `thumbv7em-none-eabihf` | STM32F4/F7/H7, nRF52, SAMD51 |
| `thumbv8m.base-none-eabi` | Cortex-M23, including SAM L10/L11 |
| `thumbv8m.main-none-eabi` | Cortex-M33 soft-float configurations |
| `thumbv8m.main-none-eabihf` | RP2350/RP2354 Arm, STM32H5/L5/U5, nRF53/nRF54, LPC55 |

RP2350A/B and RP2354A/B share the same two CPU choices: Arm Cortex-M33 with
hardware floating point and Hazard3 RV32IMAC. Package and stacked-flash
differences do not require different driver compilation targets.

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
